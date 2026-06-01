# -*- coding: utf-8 -*-
import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import serial.tools.list_ports

from core.serial_thread import IMUSerialThread
from core.simulator import IMUSimulatorThread
from ui.main_window import Ui_IMUViewer

class IMUViewer(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        # 1. 挂载抽离出去的静态 UI
        self.ui = Ui_IMUViewer()
        self.ui.setupUi(self)

        # 2. 初始化核心后台线程与参数
        self.serial_thread = IMUSerialThread()
        self.sim_thread = IMUSimulatorThread()
        self.is_simulator_mode = False
        self.max_points = 300
        self.plot_data = {f'{s}_{a}': [] for s in ['acc', 'gyro', 'mag'] for a in ['x', 'y', 'z']}
        self.curves = {}

        # 最新数据缓存 (线程写入, UI 定时读取)
        self._latest_data = None
        self._latest_raw = None
        self._data_mutex = QtCore.QMutex()

        # 3. 绑定曲线显示隐藏控制信号
        colors = ['#ff5555', '#55ff55', '#5555ff']
        for sensor, plot_widget in [('acc', self.ui.plot_acc), ('gyro', self.ui.plot_gyro), ('mag', self.ui.plot_mag)]:
            for idx, axis in enumerate(['x', 'y', 'z']):
                key = f"{sensor}_{axis}"
                self.curves[key] = plot_widget.plot(pen=pg.mkPen(colors[idx], width=1.5))
                self.ui.checkboxes[key].stateChanged.connect(lambda state, k=key: self.toggle_curve_visible(k, state))

        # 4. 建立底层线程信号槽绑定 — 仅缓存数据，不直接刷新 UI
        self.serial_thread.data_received.connect(self._cache_data)
        self.serial_thread.raw_string_received.connect(self._cache_raw)
        self.serial_thread.log_received.connect(self.show_status_message)

        self.sim_thread.data_received.connect(self._cache_data)
        self.sim_thread.raw_string_received.connect(self._cache_raw)
        self.sim_thread.log_received.connect(self.show_status_message)

        # 5. 绑定按钮动作
        self.ui.btn_connect.clicked.connect(self.toggle_connection)
        self.ui.btn_start_record.clicked.connect(self.start_recording_clicked)
        self.ui.btn_stop_record.clicked.connect(self.stop_recording_clicked)
        self.ui.list_algo.currentRowChanged.connect(self.change_algorithm)

        # 6. 数据源切换
        self.ui.rb_simulator.toggled.connect(self._on_simulator_radio_toggled)
        self.ui.rb_sim_manual.toggled.connect(self._on_sim_manual_radio_toggled)

        # 7. 模拟器手动滑条
        for axis_name, (slider, val_lbl) in self.ui.sim_sliders.items():
            slider.valueChanged.connect(self.on_sim_slider_changed)

        # 8. UI 刷新定时器 (30Hz，足够流畅且不卡顿)
        self.ui_refresh_timer = QtCore.QTimer()
        self.ui_refresh_timer.timeout.connect(self._refresh_ui)
        self.ui_refresh_timer.start(33)  # ~30Hz

        # 9. 统计定时器
        self.hz_timer = QtCore.QTimer()
        self.hz_timer.timeout.connect(self.calculate_fps_hz)
        self.hz_timer.start(1000)

        self.refresh_ports()
        self.build_3d_airplane()

    def toggle_curve_visible(self, key, state):
        self.curves[key].show() if state == QtCore.Qt.Checked else self.curves[key].hide()

    # ==================== 数据缓存与 UI 刷新 ====================

    def _cache_data(self, data):
        """线程信号回调：仅缓存最新数据，不直接刷新 UI"""
        self._data_mutex.lock()
        self._latest_data = data
        self._data_mutex.unlock()

    def _cache_raw(self, text):
        """线程信号回调：仅缓存最新原始字符串"""
        self._data_mutex.lock()
        self._latest_raw = text
        self._data_mutex.unlock()

    def _refresh_ui(self):
        """30Hz 定时器驱动：批量刷新所有 UI 组件"""
        self._data_mutex.lock()
        data = self._latest_data
        raw = self._latest_raw
        self._latest_data = None
        self._latest_raw = None
        self._data_mutex.unlock()

        if raw is not None:
            self.update_raw_stream(raw)

        if data is not None:
            self.update_ui_data(data)

    # ==================== 数据源切换 ====================

    def _on_simulator_radio_toggled(self, checked):
        """串口 / 模拟器 模式切换 — 由 rb_simulator.toggled 信号驱动"""
        self.is_simulator_mode = checked
        self.ui.port_box.setVisible(not checked)
        self.ui.sim_box.setVisible(checked)
        if checked:
            self.ui.btn_connect.setText("Start Sim")
        else:
            self.ui.btn_connect.setText("Connect")
        self.ui.btn_connect.setStyleSheet("background-color: #28a745; color: white; font-size: 12px;")

    def _on_sim_manual_radio_toggled(self, checked):
        """模拟器子模式切换: 正弦波 / 手动 — 由 rb_sim_manual.toggled 信号驱动"""
        self.ui.sim_manual_widget.setVisible(checked)
        self.sim_thread.mode = 'manual' if checked else 'sine'

    def on_sim_slider_changed(self):
        """手动模式滑条值变更"""
        roll = self.ui.sim_sliders['roll'][0].value()
        pitch = self.ui.sim_sliders['pitch'][0].value()
        yaw = self.ui.sim_sliders['yaw'][0].value()
        self.ui.sim_sliders['roll'][1].setText(f"{roll:.1f}°")
        self.ui.sim_sliders['pitch'][1].setText(f"{pitch:.1f}°")
        self.ui.sim_sliders['yaw'][1].setText(f"{yaw:.1f}°")
        self.sim_thread.set_manual_euler(float(roll), float(pitch), float(yaw))

    # ==================== 录制 ====================

    def start_recording_clicked(self):
        fmt = self.ui.cb_format.currentText()
        if self.is_simulator_mode:
            self.sim_thread.start_recording(fmt)
            is_rec = self.sim_thread.is_recording
        else:
            self.serial_thread.start_recording(fmt)
            is_rec = self.serial_thread.is_recording
        if is_rec:
            self.ui.btn_start_record.setEnabled(False)
            self.ui.btn_start_record.setStyleSheet("background-color: #11535e; color: #5c6370;")
            self.ui.btn_stop_record.setEnabled(True)
            self.ui.cb_format.setEnabled(False)

    def stop_recording_clicked(self):
        if self.is_simulator_mode:
            self.sim_thread.stop_recording()
        else:
            self.serial_thread.stop_recording()
        self.ui.btn_start_record.setEnabled(True)
        self.ui.btn_start_record.setStyleSheet("background-color: #17a2b8; color: white;")
        self.ui.btn_stop_record.setEnabled(False)
        self.ui.cb_format.setEnabled(True)

    def show_status_message(self, message):
        self.ui.status_bar.showMessage(message, 5000)

    def build_3d_airplane(self):
        """构建简明的 3 轴坐标架表示姿态"""
        grid = gl.GLGridItem()
        grid.setSize(18, 18, 1)
        grid.setSpacing(1, 1, 1)
        grid.setDepthValue(10)
        self.ui.gl_view.addItem(grid)

        # 参考坐标轴 (半透明, 固定不动)
        ref_colors = [(0,1,0,0.3), (1,0,0,0.3), (0,0,1,0.3)]
        for pos, color in [([[0,0,0],[4,0,0]], ref_colors[0]),
                           ([[0,0,0],[0,4,0]], ref_colors[1]),
                           ([[0,0,0],[0,0,4]], ref_colors[2])]:
            self.ui.gl_view.addItem(gl.GLLinePlotItem(pos=np.array(pos), color=color, width=1.5))

        # 姿态坐标架 — 三根粗箭头轴 + 原点球
        axis_len = 3.0
        axis_radius = 0.12
        arrow_radius = 0.3
        arrow_len = 0.6

        # X 轴 (红) — Roll
        x_shaft = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=12, radius=[axis_radius, axis_radius], length=axis_len),
            smooth=True, color=(1.0, 0.3, 0.3, 1.0), shader='shaded')
        x_shaft.translate(axis_len / 2, 0, 0)
        x_shaft.rotate(90, 0, 0, 1)

        x_arrow = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=12, radius=[arrow_radius, 0.0], length=arrow_len),
            smooth=True, color=(1.0, 0.3, 0.3, 1.0), shader='shaded')
        x_arrow.translate(axis_len, 0, 0)
        x_arrow.rotate(90, 0, 0, 1)

        # Y 轴 (绿) — Pitch
        y_shaft = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=12, radius=[axis_radius, axis_radius], length=axis_len),
            smooth=True, color=(0.3, 1.0, 0.3, 1.0), shader='shaded')
        y_shaft.translate(0, axis_len / 2, 0)

        y_arrow = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=12, radius=[arrow_radius, 0.0], length=arrow_len),
            smooth=True, color=(0.3, 1.0, 0.3, 1.0), shader='shaded')
        y_arrow.translate(0, axis_len, 0)

        # Z 轴 (蓝) — Yaw
        z_shaft = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=12, radius=[axis_radius, axis_radius], length=axis_len),
            smooth=True, color=(0.3, 0.3, 1.0, 1.0), shader='shaded')
        z_shaft.translate(0, 0, axis_len / 2)
        z_shaft.rotate(90, 1, 0, 0)

        z_arrow = gl.GLMeshItem(
            meshdata=gl.MeshData.cylinder(rows=10, cols=12, radius=[arrow_radius, 0.0], length=arrow_len),
            smooth=True, color=(0.3, 0.3, 1.0, 1.0), shader='shaded')
        z_arrow.translate(0, 0, axis_len)
        z_arrow.rotate(90, 1, 0, 0)

        # 原点球
        origin_sphere = gl.GLMeshItem(
            meshdata=gl.MeshData.sphere(rows=12, cols=12, radius=0.25),
            smooth=True, color=(0.9, 0.9, 0.9, 1.0), shader='shaded')

        for item in [x_shaft, x_arrow, y_shaft, y_arrow, z_shaft, z_arrow, origin_sphere]:
            self.ui.gl_view.addItem(item)

        self.plane_parts = [x_shaft, x_arrow, y_shaft, y_arrow, z_shaft, z_arrow, origin_sphere]

    def refresh_ports(self):
        self.ui.cb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.ui.cb_port.addItem(p.device)
        if self.ui.cb_port.count() == 0: self.ui.cb_port.addItem("/dev/ttyS0")

    def toggle_connection(self):
        if self.is_simulator_mode:
            self._toggle_sim_connection()
        else:
            self._toggle_serial_connection()

    def _toggle_serial_connection(self):
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

    def _toggle_sim_connection(self):
        if not self.sim_thread.isRunning():
            self.sim_thread.mode = 'manual' if self.ui.rb_sim_manual.isChecked() else 'sine'
            self.sim_thread.start_sim()
            self.ui.btn_connect.setText("Stop Sim")
            self.ui.btn_connect.setStyleSheet("background-color: #dc3545; color: white;")
            self.ui.lbl_status_indicator.setText("Simulator: Running")
            self.ui.lbl_status_indicator.setStyleSheet("color: #55ff55;")
            self.ui.btn_start_record.setEnabled(True)
        else:
            if self.sim_thread.is_recording: self.stop_recording_clicked()
            self.sim_thread.stop_sim()
            self.ui.btn_connect.setText("Start Sim")
            self.ui.btn_connect.setStyleSheet("background-color: #28a745; color: white;")
            self.ui.lbl_status_indicator.setText("Simulator: Stopped")
            self.ui.lbl_status_indicator.setStyleSheet("color: #ff5555;")
            self.ui.btn_start_record.setEnabled(False)
            self.ui.btn_stop_record.setEnabled(False)

    def change_algorithm(self, row):
        algo_name = self.ui.list_algo.item(row).text()
        self.serial_thread.algo_mode = algo_name
        self.sim_thread.algo_mode = algo_name

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

        # 批量更新曲线数据，减少重绘次数
        for sensor in ['acc', 'gyro', 'mag']:
            for idx, axis in enumerate(['x', 'y', 'z']):
                key = f'{sensor}_{axis}'
                self.plot_data[key].append(data[sensor][idx])
                if len(self.plot_data[key]) > self.max_points: self.plot_data[key].pop(0)

        # 一次性设置所有曲线数据
        for key, curve in self.curves.items():
            curve.setData(self.plot_data[key])

    def calculate_fps_hz(self):
        if self.is_simulator_mode:
            hz = self.sim_thread.packet_count
            self.sim_thread.packet_count = 0
            self.ui.lbl_status_hz.setText(f"Data Rate (Hz): {hz}")
            self.ui.lbl_status_drop.setText("Drop Rate (%): 0.0")
        else:
            hz = self.serial_thread.packet_count
            self.serial_thread.packet_count = 0
            self.ui.lbl_status_hz.setText(f"Data Rate (Hz): {hz}")
            total = hz + self.serial_thread.drop_count
            drop_rate = (self.serial_thread.drop_count / total * 100) if total > 0 else 0.0
            self.ui.lbl_status_drop.setText(f"Drop Rate (%): {drop_rate:.1f}")
            self.serial_thread.drop_count = 0

    def closeEvent(self, event):
        self.ui_refresh_timer.stop()
        self.hz_timer.stop()
        if self.sim_thread.isRunning():
            self.sim_thread.stop_sim()
        self.serial_thread.disconnect_serial()
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    viewer = IMUViewer()
    viewer.show()
    sys.exit(app.exec())