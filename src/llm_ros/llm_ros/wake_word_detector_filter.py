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

# --- [수정] 효율적인 필터링을 위한 라이브러리 임포트 ---
import numpy as np
import noisereduce as nr
# lfiltic은 필터의 초기 상태 'zi'를 계산하기 위해 필요합니다.
from scipy.signal import butter, lfilter, lfiltic
from typing import Optional
# --- [끝] ---

class WakeWordDetector(Node):
    def __init__(self):
        super().__init__('wake_word_detector')

        # 1. 파라미터 선언 및 로드
        self.declare_parameter('picovoice_access_key', '')
        self.declare_parameter('ppn_file_path', '')
        self.declare_parameter('stt_timeout_sec', 10.0)
        self.declare_parameter('pv_model_path', '') 

        self.declare_parameter('high_pass_cutoff', 90.0)
        self.declare_parameter('nr_enabled', True)

        self.picovoice_access_key = self.get_parameter('picovoice_access_key').get_parameter_value().string_value
        self.ppn_file_path = self.get_parameter('ppn_file_path').get_parameter_value().string_value
        self.stt_timeout_sec = self.get_parameter('stt_timeout_sec').get_parameter_value().double_value
        self.pv_model_path = self.get_parameter('pv_model_path').get_parameter_value().string_value

        self.high_pass_cutoff = self.get_parameter('high_pass_cutoff').get_parameter_value().double_value
        if self.high_pass_cutoff <= 0:
            self.high_pass_cutoff = None
        self.nr_enabled = self.get_parameter('nr_enabled').get_parameter_value().bool_value

        # ... (파라미터 유효성 검사 코드는 동일) ...
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
        
        # --- [추가] 최적화: 상태 저장(Stateful) 필터 초기화 ---
        self.filter_zi = None
        if self.high_pass_cutoff:
            try:
                # 필터 계수(b, a)를 한 번만 계산합니다.
                fs = self.porcupine.sample_rate
                nyq = 0.5 * fs
                norm = float(self.high_pass_cutoff) / nyq
                self.butter_b, self.butter_a = butter(5, norm, btype='high', analog=False)
                
                # 필터의 초기 상태 'zi'를 계산합니다.
                self.filter_zi = lfiltic(self.butter_b, self.butter_a, [0.0] * (max(len(self.butter_b), len(self.butter_a)) - 1))
                self.get_logger().info("고역 통과 필터 상태 초기화 완료.")
            except Exception as e:
                self.get_logger().warn(f"고역 통과 필터 초기화 실패: {e}. 필터를 비활성화합니다.")
                self.high_pass_cutoff = None
        # --- [끝] ---
        
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

        # 5. STT 비활성화 타이머 (변경 없음)
        self.stt_active_timer = None
        self.stt_active = False 

        # 6. 오디오 처리 스레드 시작 (변경 없음)
        self._audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self._audio_thread.start()
        self.get_logger().info("WakeWordDetector 노드 시작. 오디오 감지 대기 중...")

    # --- [제거] _high_pass_filter 함수 ---
    # 상태 저장 필터링을 위해 _audio_loop 내부에서 직접 처리하므로
    # 이 헬퍼 함수는 더 이상 필요하지 않습니다.
    # --- [끝] ---

    # --- [수정] 최적화된 오디오 처리 핫 루프(Hot Loop) ---
    def _audio_loop(self):
        """
        [최적화됨] 오디오 스트림을 읽고, 필터링하며, Porcupine으로 처리하는 메인 루프.
        """
        while rclpy.ok():
            try:
                # 1. 오디오 읽기 (bytes)
                pcm_bytes = self.audio_stream.read(self.porcupine.frame_length, exception_on_overflow=False)
                
                # 2. [최적화 1] bytes -> int16 Numpy 배열 (Zero-Copy)
                # struct.unpack_from + np.array(tuple) 보다 월등히 빠름
                pcm_int16_array = np.frombuffer(pcm_bytes, dtype=np.int16)

                # 3. [최적화 2] int16 array -> float32 array (필터링 준비)
                # int16_array.astype()이 np.array(tuple) 보다 빠름
                audio_data_float = pcm_int16_array.astype(np.float32) / 32768.0

                # --- [필터링 파이프라인 시작] ---

                # 4. [최적화 3] 상태 저장(Stateful) 고역 통과 필터 적용
                if self.high_pass_cutoff and self.filter_zi is not None:
                    # lfilter는 새 오디오 데이터와 *이전 상태(zi)*를 받아
                    # 필터링된 데이터와 *다음 상태(zf)*를 반환합니다.
                    audio_data_float, self.filter_zi = lfilter(
                        self.butter_b, 
                        self.butter_a, 
                        audio_data_float, 
                        zi=self.filter_zi
                    )

                # 5. 노이즈 감소(NR) 필터 적용 (이전과 동일)
                if self.nr_enabled:
                    audio_data_float = nr.reduce_noise(
                        y=audio_data_float, 
                        sr=self.porcupine.sample_rate,
                        stationary=True,
                        prop_decrease=0.8 
                    )

                # --- [필터링 파이프라인 끝] ---

                # 6. float32 -> int16 Numpy 배열 (Porcupine 입력 준비)
                # np.clip을 in-place로 수행하여 추가 메모리 할당 방지
                np.clip(audio_data_float, -1.0, 1.0, out=audio_data_float)
                processed_pcm_int16_array = (audio_data_float * 32767.0).astype(np.int16)

                # 7. [최적화 4] Porcupine 처리 (tolist() 제거)
                # Porcupine의 C 바인딩은 numpy.ndarray[int16]을 직접 받을 수 있습니다.
                # .tolist()를 제거함으로써 매번 512개 요소의 리스트를
                # 생성하고 복사하는 비용을 제거합니다.
                keyword_index = self.porcupine.process(processed_pcm_int16_array)

                if keyword_index >= 0:
                    self._on_wake_word_detected()
                    
            except IOError as e:
                if e.errno == pyaudio.paInputOverflowed:
                    self.get_logger().debug("오디오 입력 오버플로우 발생. 무시합니다.")
                else:
                    self.get_logger().error(f"오디오 IO 오류 발생: {e}")
            except Exception as e:
                self.get_logger().error(f"오디오 처리 중 오류 발생: {e}")
    # --- [수정 끝] ---

    def _on_wake_word_detected(self):
        if not self.stt_active:
            self.get_logger().info("웨이크 워드 '도리야' 감지! 10초 동안 STT 활성화.")
            self.stt_active = True
            msg = Bool()
            msg.data = True
            self.pub_activate.publish(msg)

        # [수정] 기존 타이머가 있다면 확실하게 취소
        if self.stt_active_timer and not self.stt_active_timer.cancelled():
            self.stt_active_timer.cancel()
        
        # 새 타이머 시작
        self.stt_active_timer = self.create_timer(self.stt_timeout_sec, self._on_stt_timeout)

    def _on_stt_timeout(self):
        # [수정] STT가 이미 비활성화된 경우(예: 수동 종료) 중복 실행 방지
        if not self.stt_active:
            return
            
        self.get_logger().info(f"STT 활성화 시간 초과 ({self.stt_timeout_sec}초). STT 비활성화.")
        self.stt_active = False
        msg = Bool()
        msg.data = False
        self.pub_activate.publish(msg)
        
        if self.stt_active_timer and not self.stt_active_timer.cancelled():
            self.stt_active_timer.cancel()

    def destroy_node(self):
        self.get_logger().info("WakeWordDetector 노드 종료 중...")
        
        # [수정] 스레드가 먼저 종료되도록 유도 (필수는 아님)
        if self._audio_thread and self._audio_thread.is_alive():
            self.get_logger().info("오디오 스레드 종료 대기...")
            # rclpy.ok()가 False가 되면 스레드는 자동 종료됨
        
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
    
    # --- [추가] 필수 라이브러리 설치 확인 (변경 없음) ---
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