#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import pvporcupine
import pyaudio
import struct
import threading
import os
import time

# --- [추가] 필터링을 위한 라이브러리 ---
import numpy as np
import noisereduce as nr
from scipy.signal import butter, lfilter
from typing import Optional
# --- [끝] ---

class WakeWordDetector(Node):
    def __init__(self):
        super().__init__('wake_word_detector')

        # 1. 파라미터 선언 및 로드
        self.declare_parameter('picovoice_access_key', '')
        self.declare_parameter('ppn_file_path', '')
        self.declare_parameter('stt_timeout_sec', 10.0)
        self.declare_parameter('pv_model_path', '') # .pv 모델 파일 경로

        # --- [추가] voice2text.py의 필터 파라미터 ---
        self.declare_parameter('high_pass_cutoff', 90.0)
        self.declare_parameter('nr_enabled', True)
        # --- [끝] ---

        self.picovoice_access_key = self.get_parameter('picovoice_access_key').get_parameter_value().string_value
        self.ppn_file_path = self.get_parameter('ppn_file_path').get_parameter_value().string_value
        self.stt_timeout_sec = self.get_parameter('stt_timeout_sec').get_parameter_value().double_value
        self.pv_model_path = self.get_parameter('pv_model_path').get_parameter_value().string_value

        # --- [추가] 필터 파라미터 로드 ---
        self.high_pass_cutoff = self.get_parameter('high_pass_cutoff').get_parameter_value().double_value
        if self.high_pass_cutoff <= 0:
            self.high_pass_cutoff = None
        self.nr_enabled = self.get_parameter('nr_enabled').get_parameter_value().bool_value
        # --- [끝] ---

        if not self.picovoice_access_key:
            self.get_logger().error("Picovoice Access Key가 설정되지 않았습니다.")
            raise ValueError("Picovoice Access Key is not set.")
            
        if not self.ppn_file_path or not os.path.exists(self.ppn_file_path):
            self.get_logger().error(f"PPN 파일 경로가 유효하지 않습니다: {self.ppn_file_path}")
            raise ValueError("PPN file path is invalid.")
        
        if not self.pv_model_path or not os.path.exists(self.pv_model_path):
            self.get_logger().error(f"PV 모델 파일 경로가 유효하지 않습니다: {self.pv_model_path}")
            raise ValueError("PV model file path is invalid.")

        # 2. Publisher 생성
        self.pub_activate = self.create_publisher(Bool, '/activate_stt', 10)

        # 3. Porcupine 초기화
        self.porcupine = None
        try:
            self.porcupine = pvporcupine.create(
                access_key=self.picovoice_access_key,
                keyword_paths=[self.ppn_file_path],
                model_path=self.pv_model_path
            )
            self.get_logger().info(f"Porcupine 초기화 성공. PPN: {self.ppn_file_path}, PV: {self.pv_model_path}")
            self.get_logger().info(f"필터 설정: High-pass={self.high_pass_cutoff}Hz, NoiseReduce={'On' if self.nr_enabled else 'Off'}")
        except Exception as e:
            self.get_logger().error(f"Porcupine 초기화 실패: {e}")
            raise e
        
        # 4. PyAudio 초기화 및 스트림 설정
        self.py_audio = None
        self.audio_stream = None
        try:
            self.py_audio = pyaudio.PyAudio()
            self.audio_stream = self.py_audio.open(
                rate=self.porcupine.sample_rate,
                channels=1,
                format=pyaudio.paInt16,
                input=True,
                frames_per_buffer=self.porcupine.frame_length
            )
            self.get_logger().info("PyAudio 스트림 열기 성공.")
        except Exception as e:
            self.get_logger().error(f"PyAudio 스트림 초기화 실패: {e}")
            if self.porcupine:
                self.porcupine.delete()
            raise e

        # 5. STT 비활성화 타이머
        self.stt_active_timer = None
        self.stt_active = False # 현재 STT 활성화 상태

        # 6. 오디오 처리 스레드 시작
        self._audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self._audio_thread.start()
        self.get_logger().info("WakeWordDetector 노드 시작. 오디오 감지 대기 중...")

    # --- [추가] voice2text.py의 고역 통과 필터 함수 ---
    def _high_pass_filter(self, data: np.ndarray, cutoff: Optional[float], fs: int, order: int = 5) -> np.ndarray:
        """
        주어진 오디오 데이터에 고역 통과 필터를 적용합니다.
        데이터는 float32 Numpy 배열이어야 합니다.
        """
        if cutoff is None:
            return data
        try:
            nyq = 0.5 * fs
            norm = float(cutoff) / nyq
            b, a = butter(order, norm, btype='high', analog=False)
            return lfilter(b, a, data)
        except Exception as e:
            self.get_logger().warn(f"High-pass filter 적용 실패: {e}")
            return data
    # --- [끝] ---

    # --- [수정] 오디오 처리 루프에 필터링 파이프라인 적용 ---
    def _audio_loop(self):
        """
        오디오 스트림을 읽고, 필터링하며, Porcupine으로 처리하는 메인 루프.
        """
        while rclpy.ok():
            try:
                # 1. 오디오 읽기 (bytes)
                pcm_bytes = self.audio_stream.read(self.porcupine.frame_length, exception_on_overflow=False)
                
                # 2. bytes -> int16 튜플로 변환
                pcm_int16_tuple = struct.unpack_from("h" * self.porcupine.frame_length, pcm_bytes)

                # --- [필터링 파이프라인 시작] ---

                # 3. int16 튜플 -> float32 Numpy 배열로 변환 (필터링을 위해 -1.0 ~ 1.0 정규화)
                audio_data_float = np.array(pcm_int16_tuple, dtype=np.float32) / 32768.0

                # 4. 고역 통과 필터 적용
                if self.high_pass_cutoff:
                    audio_data_float = self._high_pass_filter(
                        audio_data_float, 
                        self.high_pass_cutoff, 
                        fs=self.porcupine.sample_rate
                    )

                # 5. 노이즈 감소(NR) 필터 적용
                if self.nr_enabled:
                    # noisereduce는 float 타입 배열을 요구합니다.
                    audio_data_float = nr.reduce_noise(
                        y=audio_data_float, 
                        sr=self.porcupine.sample_rate,
                        stationary=True, # 배경 소음이 정적이라고 가정
                        prop_decrease=0.8 # 노이즈 감소 강도 (조정 가능)
                    )

                # 6. float32 Numpy 배열 -> int16 Numpy 배열로 변환 (Porcupine 입력을 위해)
                # 클리핑을 방지하기 위해 범위를 확인하고 다시 32767.0을 곱합니다.
                np.clip(audio_data_float, -1.0, 1.0, out=audio_data_float)
                processed_pcm_int16_array = (audio_data_float * 32767.0).astype(np.int16)

                # 7. Numpy 배열 -> int16 리스트로 변환
                pcm_for_porcupine = processed_pcm_int16_array.tolist()

                # --- [필터링 파이프라인 끝] ---

                # 8. 필터링된 오디오를 Porcupine으로 처리
                keyword_index = self.porcupine.process(pcm_for_porcupine)

                if keyword_index >= 0:
                    self._on_wake_word_detected()
                    
            except IOError as e:
                # 오디오 오버플로우는 흔히 발생할 수 있으므로 경고 대신 debug로 로깅
                if e.errno == pyaudio.paInputOverflowed:
                    self.get_logger().debug("오디오 입력 오버플로우 발생. 무시합니다.")
                else:
                    self.get_logger().error(f"오디오 IO 오류 발생: {e}")
            except Exception as e:
                self.get_logger().error(f"오디오 처리 중 오류 발생: {e}")
            
            # 원본의 time.sleep(0.01)은 불필요합니다.
            # self.audio_stream.read()는 이미 블로킹(blocking) 함수이며,
            # Porcupine 프레임 길이에 맞춰 정확한 타이밍으로 작동합니다.
            # sleep을 추가하면 오히려 오디오 프레임을 놓칠 수 있습니다.
    # --- [수정 끝] ---

    def _on_wake_word_detected(self):
        if not self.stt_active:
            self.get_logger().info("웨이크 워드 '도리야' 감지! 10초 동안 STT 활성화.")
            self.stt_active = True
            msg = Bool()
            msg.data = True
            self.pub_activate.publish(msg)

        # 기존 타이머가 있다면 취소 (새 웨이크 워드가 감지되면 타이머 리셋)
        if self.stt_active_timer and not self.stt_active_timer.cancelled():
            self.stt_active_timer.cancel()
        
        # 새 타이머 시작
        self.stt_active_timer = self.create_timer(self.stt_timeout_sec, self._on_stt_timeout)

    def _on_stt_timeout(self):
        self.get_logger().info(f"STT 활성화 시간 초과 ({self.stt_timeout_sec}초). STT 비활성화.")
        self.stt_active = False
        msg = Bool()
        msg.data = False
        self.pub_activate.publish(msg)
        
        # 타이머가 None이 아니고 활성화되어 있을 때만 취소
        if self.stt_active_timer and not self.stt_active_timer.cancelled():
            self.stt_active_timer.cancel()

    def destroy_node(self):
        self.get_logger().info("WakeWordDetector 노드 종료 중...")
        if self.audio_stream:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
        if self.py_audio:
            self.py_audio.terminate()
        if self.porcupine:
            self.porcupine.delete()
        
        if self.stt_active_timer and not self.stt_active_timer.cancelled():
            self.stt_active_timer.cancel()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # --- [추가] 필수 라이브러리 설치 확인 ---
    try:
        import noisereduce
        import scipy
        import numpy
    except ImportError as e:
        rclpy.logging.get_logger('wake_word_detector_main').error(
            f"필수 라이브러리가 설치되지 않았습니다: {e}. "
            f"pip install noisereduce scipy numpy"
        )
        return
    # --- [끝] ---

    node = WakeWordDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()