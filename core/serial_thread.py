# -*- coding: utf-8 -*-
import time
import struct
import csv
from datetime import datetime
import numpy as np
from PyQt5 import QtCore
import serial
from core.algorithm import MahonyAHRS

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

    def run(self):
        rx_buffer = bytearray()
        FRAME_LEN = 24

        while self.running:
            if self.serial_port and self.serial_port.in_waiting > 0:
                new_data = self.serial_port.read(self.serial_port.in_waiting)
                rx_buffer.extend(new_data)
                
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
            else:
                time.sleep(0.001)