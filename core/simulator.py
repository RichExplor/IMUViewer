# -*- coding: utf-8 -*-
"""
IMU 模拟数据模式 — 无需真实硬件即可调试 IMUViewer

提供两种模拟模式:
1. 正弦波模式: 传感器数据以不同频率的正弦波模拟自然运动
2. 手动模式: 用户通过 UI 滑条手动控制 Roll/Pitch/Yaw 角度
"""
import time
import math
import csv
from datetime import datetime
import numpy as np
from PyQt5 import QtCore
from core.algorithm import MahonyAHRS


class IMUSimulatorThread(QtCore.QThread):
    """模拟数据线程，以 50Hz 频率生成模拟 IMU 传感器数据"""

    data_received = QtCore.pyqtSignal(dict)
    raw_string_received = QtCore.pyqtSignal(str)
    log_received = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._running = False
        self._mode = 'sine'  # 'sine' 或 'manual'
        self._start_time = 0.0
        self._last_time = 0.0

        # 手动模式下的欧拉角 (度)
        self._manual_roll = 0.0
        self._manual_pitch = 0.0
        self._manual_yaw = 0.0

        # 姿态融合引擎 (正弦波模式下使用)
        self.fusion_engine = MahonyAHRS(kp=1.0, ki=0.0)
        self.algo_mode = "Mahony"

        # 统计
        self.packet_count = 0
        self.drop_count = 0

        # 录制
        self.is_recording = False
        self.record_file = None
        self.record_writer = None
        self.record_format = "csv"

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        self._mode = value

    def set_manual_euler(self, roll: float, pitch: float, yaw: float):
        """设置手动模式下的欧拉角 (度)"""
        self._manual_roll = roll
        self._manual_pitch = pitch
        self._manual_yaw = yaw

    def start_sim(self):
        self._running = True
        self._start_time = time.time()
        self._last_time = self._start_time
        self.fusion_engine = MahonyAHRS(kp=1.0, ki=0.0)
        self.start()

    def stop_sim(self):
        self.stop_recording()
        self._running = False
        self.wait(1000)

    # ==================== 录制功能 ====================

    def start_recording(self, file_format):
        if self.is_recording:
            return
        self.record_format = file_format.lower()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"imu_sim_record_{timestamp}.{self.record_format}"

        try:
            self.record_file = open(filename, 'w', newline='', encoding='utf-8')
            headers = ['Timestamp', 'AccX', 'AccY', 'AccZ',
                       'GyroX', 'GyroY', 'GyroZ', 'MagX', 'MagY', 'MagZ',
                       'Roll', 'Pitch', 'Yaw']

            if self.record_format == "csv":
                self.record_writer = csv.writer(self.record_file)
                self.record_writer.writerow(headers)
            else:
                header_line = "\t".join(headers) + "\n"
                self.record_file.write(header_line)

            self.is_recording = True
            self.log_received.emit(f"模拟录制已启动 -> {filename}")
        except Exception as e:
            self.log_received.emit(f"创建文件失败: {str(e)}")
            self.is_recording = False

    def stop_recording(self):
        if not self.is_recording:
            return
        self.is_recording = False
        time.sleep(0.01)
        if self.record_file:
            self.record_file.close()
            self.record_file = None
            self.record_writer = None
        self.log_received.emit("模拟数据已成功保存至本地。")

    # ==================== 线程主循环 ====================

    def run(self):
        """线程主循环，50Hz 生成模拟数据"""
        while self._running:
            now = time.time()
            dt = now - self._last_time
            self._last_time = now
            if dt <= 0 or dt > 0.1:
                dt = 0.02

            if self._mode == 'sine':
                data = self._generate_sine_data(now - self._start_time, dt)
            else:
                data = self._generate_manual_data()

            self.packet_count += 1

            # 发送原始数据字符串
            acc = data['acc']
            gyro = data['gyro']
            mag = data['mag']
            raw_str = (f"['{acc[0]:.2f},{acc[1]:.2f},{acc[2]:.2f},"
                       f"{gyro[0]:.2f},{gyro[1]:.2f},{gyro[2]:.2f},"
                       f"{mag[0]:.1f},{mag[1]:.1f},{mag[2]:.1f}']")
            self.raw_string_received.emit(raw_str)

            # 录制
            if self.is_recording and self.record_file:
                euler = data['euler']
                row_data = [
                    f"{time.time():.4f}",
                    f"{acc[0]:.4f}", f"{acc[1]:.4f}", f"{acc[2]:.4f}",
                    f"{gyro[0]:.2f}", f"{gyro[1]:.2f}", f"{gyro[2]:.2f}",
                    f"{mag[0]:.1f}", f"{mag[1]:.1f}", f"{mag[2]:.1f}",
                    f"{euler[0]:.2f}", f"{euler[1]:.2f}", f"{euler[2]:.2f}"
                ]
                try:
                    if self.record_format == "csv":
                        self.record_writer.writerow(row_data)
                    else:
                        self.record_file.write("\t".join(row_data) + "\n")
                except Exception:
                    pass

            self.data_received.emit(data)
            self.msleep(20) # 50Hz

    def _generate_sine_data(self, t: float, dt: float) -> dict:
        """
        生成正弦波模拟数据

        模拟一个在三个轴上缓慢旋转的 IMU 传感器，
        各传感器数据具有不同的频率和相位，模拟自然运动
        """
        # 加速度计 (g) — 重力分量 + 微小振动
        ax = 0.05 * math.sin(2 * math.pi * 0.7 * t + 0.0)
        ay = 0.05 * math.sin(2 * math.pi * 0.5 * t + 1.0)
        az = 1.0 + 0.03 * math.sin(2 * math.pi * 0.3 * t)  # 重力 1g + 微扰

        # 陀螺仪 (°/s) — 缓慢旋转 + 微小噪声
        gx = 15.0 * math.sin(2 * math.pi * 0.2 * t + 0.0) + 0.5 * math.sin(2 * math.pi * 3.1 * t)
        gy = 12.0 * math.sin(2 * math.pi * 0.15 * t + 0.8) + 0.4 * math.sin(2 * math.pi * 2.7 * t)
        gz = 10.0 * math.sin(2 * math.pi * 0.25 * t + 1.5) + 0.3 * math.sin(2 * math.pi * 3.5 * t)

        # 磁力计 (uT) — 地磁场 + 微扰
        mx = 30.0 + 5.0 * math.sin(2 * math.pi * 0.1 * t + 0.3)
        my = 10.0 + 3.0 * math.sin(2 * math.pi * 0.12 * t + 1.2)
        mz = 40.0 + 4.0 * math.sin(2 * math.pi * 0.08 * t + 2.1)

        # 姿态融合
        if self.algo_mode == "Raw Data Only":
            roll, pitch, yaw = 0.0, 0.0, 0.0
        else:
            self.fusion_engine.update_9dof(
                ax, ay, az,
                np.radians(gx), np.radians(gy), np.radians(gz),
                mx, my, mz, dt
            )
            roll, pitch, yaw = self.fusion_engine.get_euler()
            print("roll: %.2f, pitch: %.2f, yaw: %.2f" % (roll, pitch, yaw))

        return {
            'acc': [ax, ay, az],
            'gyro': [gx, gy, gz],
            'mag': [mx, my, mz],
            'euler': [roll, pitch, yaw]
        }

    def _generate_manual_data(self) -> dict:
        """
        根据手动设置的欧拉角反推传感器数据

        在静止状态下，根据 Roll/Pitch/Yaw 反推理想的重力分量和地磁分量
        """
        roll_rad = np.radians(self._manual_roll)
        pitch_rad = np.radians(self._manual_pitch)
        yaw_rad = np.radians(self._manual_yaw)

        # 重力在体坐标系下的投影 (静止加速度计读数)
        ax = -math.sin(pitch_rad)
        ay = math.sin(roll_rad) * math.cos(pitch_rad)
        az = math.cos(roll_rad) * math.cos(pitch_rad)

        # 陀螺仪静止为零
        gx, gy, gz = 0.0, 0.0, 0.0

        # 地磁场在体坐标系下的投影 (假设地磁水平分量 ~30uT, 垂直分量 ~40uT)
        mx = 30.0 * math.cos(pitch_rad) * math.cos(yaw_rad) + 40.0 * math.sin(pitch_rad)
        my = 30.0 * (math.sin(roll_rad) * math.sin(pitch_rad) * math.cos(yaw_rad)
                      - math.cos(roll_rad) * math.sin(yaw_rad)) \
             + 40.0 * (-math.sin(roll_rad) * math.cos(pitch_rad))
        mz = 30.0 * (math.cos(roll_rad) * math.sin(pitch_rad) * math.cos(yaw_rad)
                      + math.sin(roll_rad) * math.sin(yaw_rad)) \
             + 40.0 * math.cos(roll_rad) * math.cos(pitch_rad)

        return {
            'acc': [ax, ay, az],
            'gyro': [gx, gy, gz],
            'mag': [mx, my, mz],
            'euler': [self._manual_roll, self._manual_pitch, self._manual_yaw]
        }
