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

class WakeWordDetector(Node):
    def __init__(self):
        super().__init__('wake_word_detector')

        # 1. 파라미터 선언 및 로드
        self.declare_parameter('picovoice_access_key', '')
        self.declare_parameter('ppn_file_path', '')
        self.declare_parameter('stt_timeout_sec', 10.0)
        
        # --- [추가] ---
        self.declare_parameter('pv_model_path', '') # .pv 모델 파일 경로 파라미터 추가
        # --- [끝] ---

        self.picovoice_access_key = self.get_parameter('picovoice_access_key').get_parameter_value().string_value
        self.ppn_file_path = self.get_parameter('ppn_file_path').get_parameter_value().string_value
        self.stt_timeout_sec = self.get_parameter('stt_timeout_sec').get_parameter_value().double_value

        # --- [추가] ---
        self.pv_model_path = self.get_parameter('pv_model_path').get_parameter_value().string_value
        # --- [끝] ---

        if not self.picovoice_access_key:
            self.get_logger().error("Picovoice Access Key가 설정되지 않았습니다. 'picovoice_access_key' 파라미터를 설정해주세요.")
            raise ValueError("Picovoice Access Key is not set.")
            
        if not self.ppn_file_path or not os.path.exists(self.ppn_file_path):
            self.get_logger().error(f"PPN 파일 경로가 유효하지 않습니다: {self.ppn_file_path}. 'ppn_file_path' 파라미터를 확인해주세요.")
            raise ValueError("PPN file path is invalid.")
        
        # --- [추가] ---
        # .pv 파일 경로 유효성 검사 추가
        if not self.pv_model_path or not os.path.exists(self.pv_model_path):
            self.get_logger().error(f"PV 모델 파일 경로가 유효하지 않습니다: {self.pv_model_path}. 'pv_model_path' 파라미터를 확인해주세요.")
            raise ValueError("PV model file path is invalid.")
        # --- [끝] ---

        # 2. Publisher 생성
        self.pub_activate = self.create_publisher(Bool, '/activate_stt', 10)

        # 3. Porcupine 초기화
        self.porcupine = None
        try:
            # --- [수정] ---
            # pvporcupine.create() 함수에 model_path 인자 추가
            self.porcupine = pvporcupine.create(
                access_key=self.picovoice_access_key,
                keyword_paths=[self.ppn_file_path],
                model_path=self.pv_model_path
            )
            # --- [끝] ---
            self.get_logger().info(f"Porcupine 초기화 성공. PPN: {self.ppn_file_path}, PV: {self.pv_model_path}")
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

    def _audio_loop(self):
        while rclpy.ok():
            try:
                pcm = self.audio_stream.read(self.porcupine.frame_length, exception_on_overflow=False)
                pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

                keyword_index = self.porcupine.process(pcm)

                if keyword_index >= 0:
                    self._on_wake_word_detected()
            except Exception as e:
                self.get_logger().error(f"오디오 처리 중 오류 발생: {e}")
            time.sleep(0.01) # CPU 사용량 조절

    def _on_wake_word_detected(self):
        if not self.stt_active:
            self.get_logger().info("웨이크 워드 '도리야' 감지! 10초 동안 STT 활성화.")
            self.stt_active = True
            msg = Bool()
            msg.data = True
            self.pub_activate.publish(msg)

        # 기존 타이머가 있다면 취소
        if self.stt_active_timer:
            self.stt_active_timer.cancel()
        
        # 새 타이머 시작
        self.stt_active_timer = self.create_timer(self.stt_timeout_sec, self._on_stt_timeout)

    def _on_stt_timeout(self):
        self.get_logger().info(f"STT 활성화 시간 초과 ({self.stt_timeout_sec}초). STT 비활성화.")
        self.stt_active = False
        msg = Bool()
        msg.data = False
        self.pub_activate.publish(msg)
        self.stt_active_timer.cancel() # 타이머 완료 후 취소

    def destroy_node(self):
        self.get_logger().info("WakeWordDetector 노드 종료 중...")
        if self.audio_stream:
            self.audio_stream.stop_stream()
            self.audio_stream.close()
        if self.py_audio:
            self.py_audio.terminate()
        if self.porcupine:
            self.porcupine.delete()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
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