# -*- coding: utf-8 -*-
import time
import struct
import csv
from datetime import datetime
import numpy as np
from PyQt5 import QtCore
import serial
from core.algorithm import MahonyAHRS

# ==================== YS-TLV 协议常量 ====================

YS_TLV_HEADER = bytes([0x59, 0x53])

# Data ID -> (载荷字节数, 字段数, 解码格式, 缩放因子, 输出键名)
YS_TLV_DATA_IDS = {
    0x01: (2,  1, '<h',  0.01,       'temperature'),    # 温度 int16
    0x10: (12, 3, '<3i', 1e-6,       'acc'),             # 加速度 3xint32
    0x20: (12, 3, '<3i', 1e-6,       'gyro'),            # 角速度 3xint32
    0x30: (12, 3, '<3i', 1e-6,       'mag_normalized'),  # 磁场归一化 3xint32
    0x31: (12, 3, '<3i', 0.001,      'mag'),             # 磁场强度 3xint32
    0x40: (12, 3, '<3i', 1e-6,       'euler_raw'),       # 欧拉角 3xint32 (P/R/Y)
    0x41: (16, 4, '<4i', 1e-6,       'quaternion'),      # 四元数 4xint32
}


class IMUSerialThread(QtCore.QThread):
    """串口高频流解析与数据运算线程"""
    data_received = QtCore.pyqtSignal(dict)
    raw_string_received = QtCore.pyqtSignal(str)
    log_received = QtCore.pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        self.fusion_engine = MahonyAHRS(kp=1.0, ki=0.0)
        self.last_time = None
        self.algo_mode = "Mahony"

        self.GRAVITY = 9.8
        self.ACC_SCALE = 1.0 / 32768.0 * self.GRAVITY * 16.0
        self.GYRO_SCALE = 1.0 / 32768.0 * 2000.0
        self.MAG_SCALE = 1.0

        self.packet_count = 0
        self.drop_count = 0

        self.is_recording = False
        self.record_file = None
        self.record_writer = None
        self.record_format = "csv"

        # 协议模式: "Hanwei" (原有固定帧协议) 或 "YS-TLV" (新增 TLV 协议)
        self.protocol_mode = "Hanwei"

    def connect_serial(self, port, baudrate):
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.01)
            self.running = True
            self.last_time = time.time()
            self.start()
            return True
        except Exception as e:
            self.log_received.emit(f"连接失败: {str(e)}")
            return False

    def disconnect_serial(self):
        self.stop_recording()
        self.running = False
        self.wait()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def start_recording(self, file_format):
        if self.is_recording: return
        self.record_format = file_format.lower()

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"imu_record_{timestamp}.{self.record_format}"

        try:
            self.record_file = open(filename, 'w', newline='', encoding='utf-8')
            headers = ['Timestamp', 'AccX', 'AccY', 'AccZ', 'GyroX', 'GyroY', 'GyroZ', 'MagX', 'MagY', 'MagZ', 'Roll', 'Pitch', 'Yaw']

            if self.record_format == "csv":
                self.record_writer = csv.writer(self.record_file)
                self.record_writer.writerow(headers)
            else:
                header_line = "\t".join(headers) + "\n"
                self.record_file.write(header_line)

            self.is_recording = True
            self.log_received.emit(f"录制已启动 -> {filename}")
        except Exception as e:
            self.log_received.emit(f"创建文件失败: {str(e)}")
            self.is_recording = False

    def stop_recording(self):
        if not self.is_recording: return
        self.is_recording = False
        time.sleep(0.01)
        if self.record_file:
            self.record_file.close()
            self.record_file = None
            self.record_writer = None
            self.log_received.emit("数据已成功保存至本地。")

    # ==================== YS-TLV 协议解析方法 ====================

    def _verify_ys_tlv_checksum(self, frame, msg_len):
        """验证 YS-TLV 帧的双重累加校验和

        校验范围: TID (偏移2) 到 MESSAGE 结束 (偏移 4+LEN, 含 LEN 字节)

        Args:
            frame: 完整帧字节 (含帧头到CK2)
            msg_len: LEN 字段值

        Returns:
            bool: 校验是否通过
        """
        ck1_calc = 0
        ck2_calc = 0
        for i in range(2, 5 + msg_len):  # 偏移2 到 偏移4+LEN
            ck1_calc = (ck1_calc + frame[i]) & 0xFF
            ck2_calc = (ck2_calc + ck1_calc) & 0xFF

        ck1_recv = frame[5 + msg_len]
        ck2_recv = frame[5 + msg_len + 1]
        return ck1_calc == ck1_recv and ck2_calc == ck2_recv

    def _parse_ys_tlv_tlvs(self, payload):
        """解析 YS-TLV 帧中的 TLV 子包序列

        Args:
            payload: LEN 字节的 TLV 子包数据

        Returns:
            dict: 解析后的各传感器数据
        """
        result = {}
        offset = 0
        while offset < len(payload):
            data_id = payload[offset]
            offset += 1

            if data_id not in YS_TLV_DATA_IDS:
                break  # 未知 ID，无法确定长度，停止解析

            payload_len, field_count, fmt, scale, key = YS_TLV_DATA_IDS[data_id]

            if offset + payload_len > len(payload):
                break  # 数据不足，停止解析

            raw_values = struct.unpack(fmt, payload[offset:offset + payload_len])
            offset += payload_len

            if field_count == 1:
                result[key] = raw_values[0] * scale
            else:
                result[key] = [v * scale for v in raw_values]

        return result

    def _process_ys_tlv_packet(self, tlv_data):
        """将 YS-TLV 解析结果转换为统一的 processed_packet 格式

        关键转换:
        - acc: 1e-6 m/s² → m/s² (直接使用)
        - gyro: 1e-6 deg/s → deg/s (用于输出) + rad/s (用于融合算法)
        - mag: 0.001 mGauss → mGauss (直接使用, 优先 0x31, 回退 0x30)
        - euler_raw: 若存在, 直接使用 Pitch/Roll/Yaw (跳过融合算法)
        - quaternion: 若存在, 转换为欧拉角 (备用)
        - temperature: 附加到数据包

        Returns:
            dict: {'acc': [...], 'gyro': [...], 'mag': [...], 'euler': [...], ...}
        """
        processed = {
            'acc': [0.0, 0.0, 0.0],
            'gyro': [0.0, 0.0, 0.0],
            'mag': [0.0, 0.0, 0.0],
            'euler': [0.0, 0.0, 0.0],  # [roll, pitch, yaw]
        }

        # 加速度
        if 'acc' in tlv_data:
            processed['acc'] = tlv_data['acc']

        # 角速度: TLV 协议给出 deg/s, 需转换为 rad/s 进行融合
        gyro_rads = [0.0, 0.0, 0.0]
        if 'gyro' in tlv_data:
            gyro_deg = tlv_data['gyro']  # [gx, gy, gz] in deg/s
            processed['gyro'] = gyro_deg
            gyro_rads = [np.radians(g) for g in gyro_deg]

        # 磁场强度 (优先使用 0x31, 回退到 0x30)
        if 'mag' in tlv_data:
            processed['mag'] = tlv_data['mag']
        elif 'mag_normalized' in tlv_data:
            # 归一化磁场无法直接转换为 mGauss, 仅作方向参考
            processed['mag'] = tlv_data['mag_normalized']

        # 温度 (可选附加)
        if 'temperature' in tlv_data:
            processed['temperature'] = tlv_data['temperature']

        # 四元数 (可选附加)
        if 'quaternion' in tlv_data:
            processed['quaternion'] = tlv_data['quaternion']

        # 欧拉角处理策略:
        # - 若 TLV 包含 0x40 (euler_raw), 直接使用 (设备端已融合)
        # - 否则使用本地融合算法
        if 'euler_raw' in tlv_data:
            # TLV 欧拉角顺序: Pitch/Roll/Yaw → 输出顺序: Roll/Pitch/Yaw
            pitch, roll, yaw = tlv_data['euler_raw']
            processed['euler'] = [roll, pitch, yaw]
        else:
            # 使用本地 Mahony/Madgwick/EKF 融合
            ax, ay, az = processed['acc']
            gx, gy, gz = gyro_rads
            mx, my, mz = processed['mag']

            now = time.time()
            dt = now - self.last_time
            self.last_time = now
            if dt <= 0 or dt > 0.1:
                dt = 0.005

            if self.algo_mode != "Raw Data Only":
                self.fusion_engine.update_9dof(ax, ay, az, gx, gy, gz, mx, my, mz, dt)
                roll, pitch, yaw = self.fusion_engine.get_euler()
                processed['euler'] = [roll, pitch, yaw]

        return processed

    # ==================== 缓冲区处理方法 ====================

    def _process_hanwei_buffer(self, rx_buffer):
        """原有 Hanwei 固定帧协议解析 (24字节)

        帧格式:
          帧头: 0x4E 0x4A 0x13 0x01 (4B)
          数据: 9×int16 (18B)
          校验: uint16 累加和 (2B)
          总长: 24B
        """
        FRAME_LEN = 24
        while len(rx_buffer) >= FRAME_LEN:
            if rx_buffer[0] == 0x4E and rx_buffer[1] == 0x4A and rx_buffer[2] == 0x13 and rx_buffer[3] == 0x01:
                frame = rx_buffer[:FRAME_LEN]
                calc_sum = sum(frame[:22]) & 0xFFFF
                pack_sum = struct.unpack('<H', frame[22:24])[0]

                if calc_sum == pack_sum:
                    raw_data = struct.unpack('<9h', frame[4:22])
                    self.packet_count += 1

                    ax = raw_data[0] * self.ACC_SCALE
                    ay = raw_data[1] * self.ACC_SCALE
                    az = raw_data[2] * self.ACC_SCALE
                    gx = np.radians(raw_data[3] * self.GYRO_SCALE)
                    gy = np.radians(raw_data[4] * self.GYRO_SCALE)
                    gz = np.radians(raw_data[5] * self.GYRO_SCALE)
                    mx = raw_data[6] / self.MAG_SCALE
                    my = raw_data[7] / self.MAG_SCALE
                    mz = raw_data[8] / self.MAG_SCALE

                    raw_data_str = f"['{ax},{ay},{az},{np.degrees(gx)},{np.degrees(gy)},{np.degrees(gz)},{mx},{my},{mz}']"
                    self.raw_string_received.emit(raw_data_str)

                    now = time.time()
                    dt = now - self.last_time
                    self.last_time = now
                    if dt <= 0 or dt > 0.1: dt = 0.005

                    if self.algo_mode == "Raw Data Only":
                        roll, pitch, yaw = 0.0, 0.0, 0.0
                    else:
                        self.fusion_engine.update_9dof(ax, ay, az, gx, gy, gz, mx, my, mz, dt)
                        roll, pitch, yaw = self.fusion_engine.get_euler()

                    if self.is_recording and self.record_file:
                        row_data = [
                            f"{time.time():.4f}",
                            f"{ax:.4f}", f"{ay:.4f}", f"{az:.4f}",
                            f"{np.degrees(gx):.2f}", f"{np.degrees(gy):.2f}", f"{np.degrees(gz):.2f}",
                            f"{mx:.1f}", f"{my:.1f}", f"{mz:.1f}",
                            f"{roll:.2f}", f"{pitch:.2f}", f"{yaw:.2f}"
                        ]
                        try:
                            if self.record_format == "csv":
                                self.record_writer.writerow(row_data)
                            else:
                                self.record_file.write("\t".join(row_data) + "\n")
                        except Exception:
                            pass

                    processed_packet = {
                        'acc': [ax, ay, az],
                        'gyro': [np.degrees(gx), np.degrees(gy), np.degrees(gz)],
                        'mag': [mx, my, mz],
                        'euler': [roll, pitch, yaw]
                    }
                    self.data_received.emit(processed_packet)
                    del rx_buffer[:FRAME_LEN]
                else:
                    self.drop_count += 1
                    del rx_buffer[0]
            else:
                del rx_buffer[0]

    def _process_ys_tlv_buffer(self, rx_buffer):
        """YS-TLV 变长帧协议解析

        帧格式:
          帧头: 0x59 0x53 (2B)
          TID: uint16 LE (2B)
          LEN: uint8 (1B) — TLV 子包序列总字节数
          MESSAGE: LEN 字节的 TLV 子包序列
          CK1: uint8 — 双重累加校验和1
          CK2: uint8 — 双重累加校验和2
          总帧长: 5 + LEN + 2
        """
        while len(rx_buffer) >= 7:  # 最小帧: 帧头2 + TID2 + LEN1 + CK1 + CK2 = 7 (LEN=0)
            # 查找帧头
            if rx_buffer[0] != 0x59 or rx_buffer[1] != 0x53:
                del rx_buffer[0]
                continue

            if len(rx_buffer) < 5:
                break  # 数据不足, 等待更多数据

            msg_len = rx_buffer[4]
            total_frame_len = 5 + msg_len + 2  # 头5B + MESSAGE + CK1 + CK2

            if len(rx_buffer) < total_frame_len:
                break  # 数据不足, 等待更多数据

            frame = bytes(rx_buffer[:total_frame_len])

            # 校验
            if self._verify_ys_tlv_checksum(frame, msg_len):
                self.packet_count += 1

                # 解析 TLV 子包
                tlv_payload = frame[5:5 + msg_len]
                tlv_data = self._parse_ys_tlv_tlvs(tlv_payload)

                # 转换为统一数据包格式
                processed_packet = self._process_ys_tlv_packet(tlv_data)

                # 构造原始字符串显示
                raw_data_str = str(processed_packet)
                self.raw_string_received.emit(raw_data_str)

                # 录制
                if self.is_recording and self.record_file:
                    row_data = [
                        f"{time.time():.4f}",
                        f"{processed_packet['acc'][0]:.4f}", f"{processed_packet['acc'][1]:.4f}", f"{processed_packet['acc'][2]:.4f}",
                        f"{processed_packet['gyro'][0]:.2f}", f"{processed_packet['gyro'][1]:.2f}", f"{processed_packet['gyro'][2]:.2f}",
                        f"{processed_packet['mag'][0]:.1f}", f"{processed_packet['mag'][1]:.1f}", f"{processed_packet['mag'][2]:.1f}",
                        f"{processed_packet['euler'][0]:.2f}", f"{processed_packet['euler'][1]:.2f}", f"{processed_packet['euler'][2]:.2f}"
                    ]
                    try:
                        if self.record_format == "csv":
                            self.record_writer.writerow(row_data)
                        else:
                            self.record_file.write("\t".join(row_data) + "\n")
                    except Exception:
                        pass

                # 发射信号
                self.data_received.emit(processed_packet)
                del rx_buffer[:total_frame_len]
            else:
                self.drop_count += 1
                del rx_buffer[:2]  # 跳过帧头, 重新搜索

    # ==================== 主循环 ====================

    def run(self):
        rx_buffer = bytearray()

        while self.running:
            if self.serial_port and self.serial_port.in_waiting > 0:
                new_data = self.serial_port.read(self.serial_port.in_waiting)
                rx_buffer.extend(new_data)

                if self.protocol_mode == "Hanwei":
                    self._process_hanwei_buffer(rx_buffer)
                else:
                    self._process_ys_tlv_buffer(rx_buffer)
            else:
                time.sleep(0.001)
