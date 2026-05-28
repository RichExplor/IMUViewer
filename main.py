# -*- coding: utf-8 -*-
import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import serial.tools.list_ports

from core.serial_thread import IMUSerialThread
from ui.main_window import Ui_IMUViewer

class IMUViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # 1. 挂载抽离出去的静态 UI
        self.ui = Ui_IMUViewer()
        self.ui.setupUi(self)

        # 2. 初始化核心后台线程与参数
        self.serial_thread = IMUSerialThread()
        self.max_points = 300
        self.plot_data = {f'{s}_{a}': [] for s in ['acc', 'gyro', 'mag'] for a in ['x', 'y', 'z']}
        self.curves = {}

        # 3. 绑定曲线显示隐藏控制信号
        colors = ['#ff5555', '#55ff55', '#5555ff']
        for sensor, plot_widget in [('acc', self.ui.plot_acc), ('gyro', self.ui.plot_gyro), ('mag', self.ui.plot_mag)]:
            for idx, axis in enumerate(['x', 'y', 'z']):
                key = f"{sensor}_{axis}"
                self.curves[key] = plot_widget.plot(pen=pg.mkPen(colors[idx], width=1.5))
                self.ui.checkboxes[key].stateChanged.connect(lambda state, k=key: self.toggle_curve_visible(k, state))

        # 4. 建立底层线程信号槽绑定
        self.serial_thread.data_received.connect(self.update_ui_data)
        self.serial_thread.raw_string_received.connect(self.update_raw_stream)
        self.serial_thread.log_received.connect(self.show_status_message)

        # 5. 绑定按钮动作
        self.ui.btn_connect.clicked.connect(self.toggle_connection)
        self.ui.btn_start_record.clicked.connect(self.start_recording_clicked)
        self.ui.btn_stop_record.clicked.connect(self.stop_recording_clicked)
        self.ui.list_algo.currentRowChanged.connect(self.change_algorithm)

        # 6. 定时器
        self.hz_timer = QtCore.QTimer()
        self.hz_timer.timeout.connect(self.calculate_fps_hz)
        self.hz_timer.start(1000)

        self.refresh_ports()
        self.build_3d_airplane()

    def toggle_curve_visible(self, key, state):
        self.curves[key].show() if state == QtCore.Qt.Checked else self.curves[key].hide()

    def start_recording_clicked(self):
        fmt = self.ui.cb_format.currentText()
        self.serial_thread.start_recording(fmt)
        if self.serial_thread.is_recording:
            self.ui.btn_start_record.setEnabled(False)
            self.ui.btn_start_record.setStyleSheet("background-color: #11535e; color: #5c6370;") 
            self.ui.btn_stop_record.setEnabled(True)
            self.ui.cb_format.setEnabled(False)

    def stop_recording_clicked(self):
        self.serial_thread.stop_recording()
        self.ui.btn_start_record.setEnabled(True)
        self.ui.btn_start_record.setStyleSheet("background-color: #17a2b8; color: white;")
        self.ui.btn_stop_record.setEnabled(False)
        self.ui.cb_format.setEnabled(True)

    def show_status_message(self, message):
        self.ui.status_bar.showMessage(message, 5000)

    def build_3d_airplane(self):
        grid = gl.GLGridItem()
        grid.setSize(18, 18, 1)
        grid.setSpacing(1, 1, 1)
        grid.setDepthValue(10)
        self.ui.gl_view.addItem(grid)

        for pos, color in [([[0,0,0],[4,0,0]], (0,1,0,1)), ([[0,0,0],[0,4,0]], (1,0,0,1)), ([[0,0,0],[0,0,4]], (0,0,1,1))]:
            self.ui.gl_view.addItem(gl.GLLinePlotItem(pos=np.array(pos), color=color, width=3))

        fuselage = gl.GLMeshItem(meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[0.3, 0.3], length=4.5), smooth=True, color=(0.7, 0.7, 0.7, 1.0), shader='shaded')
        fuselage.translate(0, 0, -2.25)
        fuselage.rotate(90, 1, 0, 0)
        
        wings = gl.GLMeshItem(vertexes=np.array([[-3.0,0,-0.05],[3.0,0,-0.05],[0,0.7,0],[0,-0.7,0]]), faces=np.array([[0,1,2],[0,3,1]]), color=(0.6, 0.6, 0.6, 1.0), shader='flat')
        wings.translate(0, 0.4, 0)

        tail = gl.GLMeshItem(vertexes=np.array([[-1.0,0,0],[1.0,0,0],[0,-0.3,0],[0,0,0.7]]), faces=np.array([[0,1,3],[0,2,1]]), color=(0.5, 0.5, 0.5, 1.0), shader='flat')
        tail.translate(0, -1.8, 0)

        self.ui.gl_view.addItem(fuselage)
        self.ui.gl_view.addItem(wings)
        self.ui.gl_view.addItem(tail)
        self.plane_parts = [fuselage, wings, tail]

    def refresh_ports(self):
        self.ui.cb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.ui.cb_port.addItem(p.device)
        if self.ui.cb_port.count() == 0: self.ui.cb_port.addItem("/dev/ttyS0")

    def toggle_connection(self):
        if not self.serial_thread.running:
            port = self.ui.cb_port.currentText()
            baud = int(self.ui.cb_baud.currentText())
            if self.serial_thread.connect_serial(port, baud):
                self.ui.btn_connect.setText("Disconnect")
                self.ui.btn_connect.setStyleSheet("background-color: #dc3545; color: white;")
                self.ui.lbl_status_indicator.setText("Connection: Active")
                self.ui.lbl_status_indicator.setStyleSheet("color: #55ff55;")
                self.ui.btn_start_record.setEnabled(True) 
        else:
            if self.serial_thread.is_recording: self.stop_recording_clicked()
            self.serial_thread.disconnect_serial()
            self.ui.btn_connect.setText("Connect")
            self.ui.btn_connect.setStyleSheet("background-color: #28a745; color: white;")
            self.ui.lbl_status_indicator.setText("Connection: Closed")
            self.ui.lbl_status_indicator.setStyleSheet("color: #ff5555;")
            self.ui.btn_start_record.setEnabled(False) 
            self.ui.btn_stop_record.setEnabled(False)

    def change_algorithm(self, row):
        self.serial_thread.algo_mode = self.ui.list_algo.item(row).text()

    def update_raw_stream(self, text):
        if self.ui.txt_raw_stream.count() > 15: self.ui.txt_raw_stream.takeItem(0)
        self.ui.txt_raw_stream.addItem(text)
        self.ui.txt_raw_stream.scrollToBottom()

    def update_ui_data(self, data):
        roll, pitch, yaw = data['euler']
        transform = QtGui.QMatrix4x4()
        transform.setToIdentity()
        transform.rotate(yaw, 0, 0, 1)     
        transform.rotate(pitch, 0, 1, 0)   
        transform.rotate(roll, 1, 0, 0)    
        
        for part in self.plane_parts: part.setTransform(transform)
        self.ui.compass.set_yaw(yaw)
        self.ui.board_roll.setText(f"{roll:.1f}°")
        self.ui.board_pitch.setText(f"{pitch:.1f}°")
        self.ui.board_yaw.setText(f"{yaw:.1f}°")

        for sensor in ['acc', 'gyro', 'mag']:
            for idx, axis in enumerate(['x', 'y', 'z']):
                key = f'{sensor}_{axis}'
                self.plot_data[key].append(data[sensor][idx])
                if len(self.plot_data[key]) > self.max_points: self.plot_data[key].pop(0)
                self.curves[key].setData(self.plot_data[key])

    def calculate_fps_hz(self):
        hz = self.serial_thread.packet_count
        self.serial_thread.packet_count = 0
        self.ui.lbl_status_hz.setText(f"Data Rate (Hz): {hz}")
        total = hz + self.serial_thread.drop_count
        drop_rate = (self.serial_thread.drop_count / total * 100) if total > 0 else 0.0
        self.ui.lbl_status_drop.setText(f"Drop Rate (%): {drop_rate:.1f}")
        self.serial_thread.drop_count = 0

    def closeEvent(self, event):
        self.serial_thread.disconnect_serial()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    viewer = IMUViewer()
    viewer.show()
    sys.exit(app.exec())