#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import queue
import threading
from typing import Optional

import numpy as np
import sounddevice as sd
import scipy.io.wavfile as wav
from scipy.signal import butter, lfilter

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Voice2TextNode(Node):
    def __init__(self):
        super().__init__('voice2text_node')

        # 파라미터
        self.out_topic = self.declare_parameter('out_topic', '/voice2text') \
            .get_parameter_value().string_value
        
        # Whisper 설정
        # 언어 (ko/en/ja/zh 등, auto면 자동 감지)
        self.language = self.declare_parameter('language', 'ko') \
            .get_parameter_value().string_value
        # 활성화 여부 (파라미터로 on/off 가능)
        self.active = self.declare_parameter('active', True) \
            .get_parameter_value().bool_value

        # 오디오 설정
        # 샘플링 레이트. 1초 동안 뽑을 샘플 수 (default: 16000)
        self.rate = int(self.declare_parameter('rate', 16000) \
            .get_parameter_value().integer_value or 16000)
        # 마이크 입력 채널 수 (default: 1)
        self.channels = int(self.declare_parameter('channels', 1) \
            .get_parameter_value().integer_value or 1)
        # 오디오 스트림을 얼마만큼씩 처리할지 (default: 1024)
        self.chunk = int(self.declare_parameter('chunk', 1024) \
            .get_parameter_value().integer_value or 1024)
        # 음성 인식 수행 간격 (초) (default: 3초) (너무 짧으면 인식률 저하)
        self.duration = float(self.declare_parameter('duration', 3.0) \
            .get_parameter_value().double_value or 3.0)
        
        # 고역 통과 필터 컷오프 주파수 (Hz) (default: 90Hz, 0 또는 None이면 필터링 안 함)
        self.high_pass_cutoff = self.declare_parameter('high_pass_cutoff', 90.0) \
            .get_parameter_value().double_value
        if self.high_pass_cutoff <= 0:
            self.high_pass_cutoff = None
        
        # 입력 오디오 장치 인덱스 (default: -1, None이면 기본 장치)
        self.input_device_index = self.declare_parameter('input_device_index', -1) \
            .get_parameter_value().integer_value
        if self.input_device_index < 0:
            self.input_device_index = None

        # 무음 구간 건너뛰기(VAD-lite, RMS 기반)
        # RMS는 들어온 소리의 크기를 나타내는 값 (default: True)
        self.vad_enabled = self.declare_parameter('vad_enabled', True) \
            .get_parameter_value().bool_value
        # RMS 임계값 (default: 0.01, 너무 낮으면 무음 구간이 자주 인식됨, 너무 높으면 작은 소리가 인식 안 됨)
        self.vad_rms_threshold = self.declare_parameter('vad_rms_threshold', 0.01) \
            .get_parameter_value().double_value
        self.vad_debug = self.declare_parameter('vad_debug', False) \
            .get_parameter_value().bool_value

        # 퍼블리셔
        self.pub = self.create_publisher(String, self.out_topic, 10)

        # 모델 로드 (whisper/torch 지연 임포트)
        self.model = None
        self.device, self.model_size = self._pick_device_and_model()
        self._load_model()

        # 오디오
        self.audio_queue: queue.Queue = queue.Queue()
        self.stream = self._start_audio_stream(self.input_device_index)

        # 백그라운드 STT 스레드
        self._stt_thread = threading.Thread(target=self._stt_loop, daemon=True)
        self._stt_thread.start()

        # 파라미터 업데이트로 on/off 토글 허용
        self.add_on_set_parameters_callback(self._on_set_params)

        self.get_logger().info(
            f"voice2text started: topic={self.out_topic}, lang={self.language}, "
            f"active={self.active}, device={self.device}, model={self.model_size}, "
            f"VAD={'on' if self.vad_enabled else 'off'}(thr={self.vad_rms_threshold})"
        )

    # ---- 파라미터 콜백 ----
    def _on_set_params(self, params):
        for p in params:
            if p.name == 'active' and p.type_ == p.Type.BOOL:
                self.active = bool(p.value)
            elif p.name == 'language' and p.type_ == p.Type.STRING:
                self.language = str(p.value)
        return rclpy.parameter.SetParametersResult(successful=True)

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

    # ---- 신호 처리 ----
    def _high_pass_filter(self, data, cutoff: Optional[float], fs: int, order: int = 5):
        if not cutoff:
            return data
        nyq = 0.5 * fs
        norm = float(cutoff) / nyq
        b, a = butter(order, norm, btype='high', analog=False)
        return lfilter(b, a, data)

    # ---- Whisper 설정 ----
    def _pick_device_and_model(self):
        device = 'cpu'
        size = 'small'
        try:
            import torch  # noqa: F401
            import torch as _torch
            if _torch.cuda.is_available():
                device = 'cuda'
                size = 'medium'
        except Exception:
            pass
        return device, size

    def _load_model(self):
        try:
            import whisper
            self.model = whisper.load_model(self.model_size, device=self.device)
        except Exception as e:
            self.get_logger().error(f"Failed to load Whisper model: {e}")
            raise

    # ---- STT 루프 ----
    # speech to text (음성 인식)
    def _stt_loop(self):
        while rclpy.ok():
            try:
                if not self.active:
                    # 무한 성장 방지를 위해 큐를 조금씩 비움
                    try:
                        _ = self.audio_queue.get(timeout=0.2)
                    except queue.Empty:
                        pass
                    continue

                # duration 초 동안 프레임 수집
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
                if self.high_pass_cutoff:
                    audio_data = self._high_pass_filter(audio_data, self.high_pass_cutoff, fs=self.rate)

                # 무음(VAD) 체크: RMS가 임계값보다 낮으면 전사 건너뜀
                if self.vad_enabled:
                    # float32 신호의 RMS 계산
                    rms = float(np.sqrt(np.mean(np.square(audio_data, dtype=np.float32), dtype=np.float64)))
                    if self.vad_debug:
                        self.get_logger().info(f"RMS={rms:.6f} (thr={self.vad_rms_threshold})")
                    if rms < float(self.vad_rms_threshold):
                        # 무음 구간으로 판단하고 스킵
                        if self.vad_debug:
                            self.get_logger().info("무음으로 판단하여 전사 생략")
                        continue

                wav_path = '/home/yoo/workspace/dolchair_ws/src/voice2text/wav_folder/temp_audio.wav'
                try:
                    wav.write(wav_path, self.rate, audio_data)
                    result = self.model.transcribe(
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
