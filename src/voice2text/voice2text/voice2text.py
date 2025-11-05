#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import queue
import threading
from typing import Optional

import torch
import numpy as np
import sounddevice as sd
import scipy.io.wavfile as wav
from scipy.signal import butter, lfilter
import noisereduce as nr  # [신규] noisereduce 임포트

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

"""
voice2text.py: Voice to Text Node using Whisper and Silero VAD.

Originated from Highsky7 [Repo](https://github.com/Highsky7/COREA-_Jang_Yeongsil_Invention_and_Startup_Competition)

Need to make wav_folder in advance for pracical use:
    ```bash
    mkdir -p ./src/voice2text/wav_folder
    ```

"""


class Voice2TextNode(Node):
    """ Voice to Text Node using Whisper and Silero VAD.

    Args:
        input_voice: Microphone stereo audio input to mono input for Silero VAD model.
        output_topic: ROS2 topic(/voice2text) to publish transcribed text.
    Features:
    1. High-pass filter to remove rumble noise.
    2. DSP-based noise reduction using noisereduce.
    3. AI-based Voice Activity Detection (VAD) using Silero VAD model.
    4. Whisper model for speech-to-text transcription.
    5. Publishes transcribed text to a ROS2 topic.
    6. STT activation/deactivation via a ROS2 topic.
    7. Dynamic parameter adjustment at runtime.
    8. Designed for real-time operation with threading.
    """
    def __init__(self):
        super().__init__('voice2text_node')

        self.stt_active = False 
        self.stt_activation_sub = self.create_subscription(
            Bool, 
            '/activate_stt', 
            self._on_stt_activation, 
            10)

        # 파라미터
        self.out_topic = self.declare_parameter('out_topic', '/voice2text') \
            .get_parameter_value().string_value
        
        # Whisper 설정
        self.language = self.declare_parameter('language', 'ko') \
            .get_parameter_value().string_value
        self.active = self.declare_parameter('active', True) \
            .get_parameter_value().bool_value

        # 오디오 설정
        self.rate = int(self.declare_parameter('rate', 16000) \
            .get_parameter_value().integer_value or 16000)
        if self.rate != 16000:
            self.get_logger().error("Silero VAD requires a 16000Hz sample rate!")
        
        self.channels = int(self.declare_parameter('channels', 1) \
            .get_parameter_value().integer_value or 1)
        self.chunk = int(self.declare_parameter('chunk', 1024) \
            .get_parameter_value().integer_value or 1024)
        self.duration = float(self.declare_parameter('duration', 4.0) \
            .get_parameter_value().double_value or 4.0)
        
        self.high_pass_cutoff = self.declare_parameter('high_pass_cutoff', 90.0) \
            .get_parameter_value().double_value
        if self.high_pass_cutoff <= 0:
            self.high_pass_cutoff = None
        
        self.input_device_index = self.declare_parameter('input_device_index', -1) \
            .get_parameter_value().integer_value
        if self.input_device_index < 0:
            self.input_device_index = None

        # --- [수정됨] NS -> NR (NoiseReduce) ---
        # noisereduce (DSP 기반) 노이즈 제거 활성화
        self.nr_enabled = self.declare_parameter('nr_enabled', True) \
            .get_parameter_value().bool_value
        # AI 음성 감지(VAD) 활성화
        self.silero_vad_enabled = self.declare_parameter('silero_vad_enabled', True) \
            .get_parameter_value().bool_value
        self.silero_vad_threshold = self.declare_parameter('silero_vad_threshold', 0.7) \
            .get_parameter_value().double_value
        self.vad_debug = self.declare_parameter('vad_debug', True) \
            .get_parameter_value().bool_value
        # --- [수정됨] ---

        # 퍼블리셔
        self.pub = self.create_publisher(String, self.out_topic, 10)

        # 모델 로드 (Whisper, VAD)
        self.whisper_model = None
        self.vad_model = None
        self.vad_get_speech_timestamps = None
        
        self.device, self.model_size = self._pick_device_and_model()
        self._load_models() 

        # 오디오
        self.audio_queue: queue.Queue = queue.Queue()
        self.stream = self._start_audio_stream(self.input_device_index)

        # 백그라운드 STT 스레드
        self._stt_thread = threading.Thread(target=self._stt_loop, daemon=True)
        self._stt_thread.start()

        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"voice2text started: topic={self.out_topic}, lang={self.language}, "
            f"active={self.active}, device={self.device}, model={self.model_size}, "
            f"NR(noisereduce)={'on' if self.nr_enabled else 'off'}, " # [수정됨]
            f"VAD={'on (Silero)' if self.silero_vad_enabled else 'off'}(thr={self.silero_vad_threshold})"
        )

    # ---- 파라미터 콜백 ----
    def _on_set_params(self, params):
        for p in params:
            if p.name == 'active' and p.type_ == p.Type.BOOL:
                self.active = bool(p.value)
            elif p.name == 'language' and p.type_ == p.Type.STRING:
                self.language = str(p.value)
            elif p.name == 'nr_enabled' and p.type_ == p.Type.BOOL:
                self.nr_enabled = bool(p.value)
            elif p.name == 'silero_vad_enabled' and p.type_ == p.Type.BOOL:
                self.silero_vad_enabled = bool(p.value)
            elif p.name == 'silero_vad_threshold' and p.type_ == p.Type.DOUBLE:
                self.silero_vad_threshold = float(p.value)
            elif p.name == 'vad_debug' and p.type_ == p.Type.BOOL:
                self.vad_debug = bool(p.value)
        return rclpy.parameter.SetParametersResult(successful=True)

    def _on_stt_activation(self, msg: Bool):
        if msg.data and not self.stt_active:
            self.get_logger().info("STT 활성화 신호 수신. 음성 인식 시작.")
        elif not msg.data and self.stt_active:
            self.get_logger().info("STT 비활성화 신호 수신. 음성 인식 중지.")
        self.stt_active = msg.data

    # ---- 오디오 ----
    def _audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"audio status: {status}")
        try:
            self.audio_queue.put_nowait(indata.copy())
        except Exception:
            pass

    def _start_audio_stream(self, input_index: Optional[int]):
        if input_index is not None:
            sd.default.device = (input_index, None)
        
        stream = sd.InputStream(
            callback=self._audio_callback,
            channels=self.channels, 
            samplerate=self.rate,
            blocksize=self.chunk,
            dtype='float32',
        )
        stream.start()
        return stream

    # ---- 신호 처리 (필터) ----
    def _high_pass_filter(self, data, cutoff: Optional[float], fs: int, order: int = 5):
        if not cutoff:
            return data
        nyq = 0.5 * fs
        norm = float(cutoff) / nyq
        b, a = butter(order, norm, btype='high', analog=False)
        return lfilter(b, a, data)

    # ---- Whisper 및 AI 모델 설정 ----
    def _pick_device_and_model(self):
        device = 'cpu'
        size = 'small'
        if torch.cuda.is_available():
            device = 'cuda'
            size = 'large-v3-turbo'
        return device, size

    def _load_models(self):
        # 1. Whisper 모델 로드
        try:
            import whisper
            self.whisper_model = whisper.load_model(self.model_size, device=self.device)
            self.get_logger().info(f"Whisper model '{self.model_size}' loaded on {self.device}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper model: {e}")
            raise

        if self.rate != 16000:
            self.get_logger().warn("Silero VAD model NOT loaded (requires 16000Hz sample rate).")
            return

        # 2. Silero VAD 모델 로드
        try:
            model, utils = torch.hub.load(
                repo_or_dir='snakers4/silero-vad',
                model='silero_vad',
                force_reload=False, # NS가 없으므로 False로 두거나, VAD 문제 시 True로
                onnx=False
            )
            self.vad_model = model
            (self.vad_get_speech_timestamps, _, _, _, _) = utils
            self.get_logger().info("Silero VAD model loaded.")
        except Exception as e:
            self.get_logger().error(f"Failed to load Silero VAD model: {e}")


    # ---- STT 루프 (핵심 로직 수정) ----
    def _stt_loop(self):
        while rclpy.ok():
            try:
                if not self.stt_active:
                    try:
                        _ = self.audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        pass
                    continue
                if not self.active:
                    try:
                        _ = self.audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        pass
                    continue

                # 1. 원본 오디오 수집
                audio_frames = []
                need = max(1, int(self.rate / self.chunk * self.duration))
                for _ in range(need):
                    try:
                        chunk = self.audio_queue.get(timeout=5)
                        audio_frames.append(chunk)
                    except queue.Empty:
                        break

                if not audio_frames:
                    continue

                audio_data = np.concatenate(audio_frames, axis=0).astype(np.float32)
                
                # 2. 고역 통과 필터 (기본 럼블 제거)
                if self.high_pass_cutoff:
                    audio_data = self._high_pass_filter(audio_data, self.high_pass_cutoff, fs=self.rate)

                # 3. 스테레오 -> 모노 변환
                if audio_data.ndim > 1:
                    if self.vad_debug:
                        self.get_logger().info(f"Converting stereo input ({audio_data.shape}) to mono.")
                    audio_data = audio_data.mean(axis=1)
                
                # --- [신규] 4. DSP 노이즈 제거 (noisereduce) ---
                if self.nr_enabled:
                    try:
                        # noisereduce는 (samples,) 형태의 1D 배열과 샘플링 레이트(sr)를 요구합니다.
                        audio_data = nr.reduce_noise(
                            y=audio_data, 
                            sr=self.rate,
                            stationary=True # 배경 소음이 '정상(stationary)'이라고 가정
                        )
                    except Exception as e:
                        self.get_logger().warn(f"noisereduce failed: {e}")
                
                # --- [수정됨] 5. AI 음성 감지 (Stage 2) ---
                if self.silero_vad_enabled and self.vad_model and self.rate == 16000:
                    try:
                        speech_timestamps = self.vad_get_speech_timestamps(
                            audio_data, # 노이즈가 제거된 오디오로 VAD 수행
                            self.vad_model, 
                            sampling_rate=self.rate,
                            threshold=float(self.silero_vad_threshold)
                        )
                        
                        if not speech_timestamps:
                            if self.vad_debug:
                                self.get_logger().info("음성 미감지 (Silero VAD)")
                            continue 
                        else:
                            if self.vad_debug:
                                self.get_logger().info(f"음성 감지 (Silero VAD): {speech_timestamps}")
                    
                    except Exception as e:
                        self.get_logger().error(f"Silero VAD error: {e}")
                        continue # VAD 오류 시 전사 생략
                
                # --- [수정됨] 6. Whisper 전사 ---
                wav_path = './src/voice2text/wav_folder/temp_audio.wav'
                try:
                    wav.write(wav_path, self.rate, audio_data)
                    result = self.whisper_model.transcribe(
                        wav_path,
                        temperature=0.3,
                        best_of=1,
                        beam_size=3,
                        language=self.language,
                        suppress_tokens='-1',
                        condition_on_previous_text=False,
                    )
                    text = str(result.get('text') or '').strip()
                    if text:
                        self._publish(text)
                finally:
                    try:
                        os.remove(wav_path)
                    except Exception:
                        pass

            except Exception as e:
                self.get_logger().error(f"STT loop error: {e}")

    def _publish(self, text: str):
        msg = String()
        msg.data = text
        self.pub.publish(msg)
        self.get_logger().info(f"/voice2text: {text}")

    # ---- 라이프사이클 ----
    def destroy_node(self):
        try:
            if self.stream:
                self.stream.close()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Voice2TextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()