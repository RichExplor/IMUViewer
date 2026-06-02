# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from ui.widgets import CompassWidget

class Ui_IMUViewer(object):
    """纯粹的界面布局类，不包含任何串口、计算等业务逻辑"""
    def setupUi(self, MainWindow):
        MainWindow.setWindowTitle("IMU Hanwei Viewer v1.0")
        MainWindow.resize(1400, 800)

        MainWindow.setStyleSheet("""
            QMainWindow { background-color: #16181c; }
            QGroupBox { color: #ffffff; font-size: 12px; font-weight: bold; border: 1px solid #2d3139; border-radius: 6px; margin-top: 12px; padding-top: 12px; background-color: #1e222b; }
            QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; left: 10px; padding: 0 3px; }
            QLabel { color: #abb2bf; font-family: 'Segoe UI', Arial; }
            QPushButton { background-color: #2c313c; color: #ffffff; border: 1px solid #3e4451; border-radius: 4px; padding: 6px; min-height: 18px; font-weight: bold;}
            QPushButton:hover { background-color: #3e4451; border-color: #4b5263; }
            QPushButton:disabled { background-color: #1c1e22; color: #5c6370; border-color: #2d3139; }
            QComboBox { background-color: #181a1f; color: #ffffff; border: 1px solid #3e4451; border-radius: 3px; padding: 3px 5px; }
            QComboBox:disabled { background-color: #111317; color: #5c6370; border-color: #2d3139; }
            QListWidget { background-color: #111317; color: #a6e22e; border: 1px solid #2d3139; font-family: 'Consolas'; font-size: 11px; border-radius: 4px; }
            QStatusBar { background-color: #1e222b; color: #abb2bf; border-top: 1px solid #2d3139; }
            QCheckBox { color: #ffffff; font-weight: bold; font-family: 'Consolas'; font-size: 11px; }
            QCheckBox::indicator { width: 13px; height: 13px; }
            QRadioButton { color: #abb2bf; font-size: 11px; font-weight: bold; }
            QRadioButton::indicator { width: 13px; height: 13px; }
            QSlider::groove:horizontal { background: #2d3139; height: 6px; border-radius: 3px; }
            QSlider::handle:horizontal { background: #55ff55; width: 14px; margin: -4px 0; border-radius: 7px; }
            QSlider::sub-page:horizontal { background: #17a2b8; border-radius: 3px; }
        """)

        self.main_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal, MainWindow)
        MainWindow.setCentralWidget(self.main_splitter)

        # ================= 1. 左侧面板 =================
        self.left_widget = QtWidgets.QWidget()
        self.left_widget.setFixedWidth(270)
        self.left_layout = QtWidgets.QVBoxLayout(self.left_widget)
        self.left_layout.setContentsMargins(10, 5, 5, 10)

        # 数据源选择
        self.source_box = QtWidgets.QGroupBox("Data Source")
        self.source_vbox = QtWidgets.QVBoxLayout(self.source_box)
        self.rb_serial = QtWidgets.QRadioButton("Serial Port")
        self.rb_simulator = QtWidgets.QRadioButton("Simulator (No Hardware)")
        self.rb_serial.setChecked(True)
        # 确保两个 RadioButton 互斥 (同一父 widget 自动互斥)
        self.source_btn_group = QtWidgets.QButtonGroup(self.source_box)
        self.source_btn_group.addButton(self.rb_serial)
        self.source_btn_group.addButton(self.rb_simulator)
        self.source_vbox.addWidget(self.rb_serial)
        self.source_vbox.addWidget(self.rb_simulator)
        self.left_layout.addWidget(self.source_box)
    
        # 串口
        self.port_box = QtWidgets.QGroupBox("Port & Baud Rate")
        self.port_grid = QtWidgets.QGridLayout(self.port_box)
        self.port_grid.addWidget(QtWidgets.QLabel("Port"), 0, 0)
        self.cb_port = QtWidgets.QComboBox()
        self.port_grid.addWidget(self.cb_port, 0, 1)
        self.port_grid.addWidget(QtWidgets.QLabel("Baud Rate"), 1, 0)
        self.cb_baud = QtWidgets.QComboBox()
        self.cb_baud.addItems(["9600", "115200", "921600"])
        self.cb_baud.setCurrentText("115200")
        self.port_grid.addWidget(self.cb_baud, 1, 1)
        self.left_layout.addWidget(self.port_box)
    
        # 连接/启动按钮 (独立于 port_box 和 sim_box，始终可见)
        self.btn_connect = QtWidgets.QPushButton("Connect")
        self.btn_connect.setStyleSheet("background-color: #28a745; color: white; font-size: 12px;")
        self.left_layout.addWidget(self.btn_connect)
    
        # 模拟器控制
        self.sim_box = QtWidgets.QGroupBox("Simulator Controls")
        self.sim_vbox = QtWidgets.QVBoxLayout(self.sim_box)
        self.sim_box.setVisible(False)  # 默认隐藏，选择模拟器时才显示
    
        # 模拟模式选择
        self.sim_mode_layout = QtWidgets.QHBoxLayout()
        self.rb_sim_sine = QtWidgets.QRadioButton("Sine Wave")
        self.rb_sim_manual = QtWidgets.QRadioButton("Manual")
        self.rb_sim_sine.setChecked(True)
        self.sim_mode_btn_group = QtWidgets.QButtonGroup(self.sim_box)
        self.sim_mode_btn_group.addButton(self.rb_sim_sine)
        self.sim_mode_btn_group.addButton(self.rb_sim_manual)
        self.sim_mode_layout.addWidget(self.rb_sim_sine)
        self.sim_mode_layout.addWidget(self.rb_sim_manual)
        self.sim_vbox.addLayout(self.sim_mode_layout)
    
        # 手动控制滑条
        self.sim_manual_widget = QtWidgets.QWidget()
        self.sim_manual_layout = QtWidgets.QVBoxLayout(self.sim_manual_widget)
        self.sim_manual_widget.setVisible(False)  # 默认隐藏
    
        self.sim_sliders = {}
        for name, range_val in [("Roll", 180), ("Pitch", 90), ("Yaw", 180)]:
            row = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel(f"{name}:")
            lbl.setFixedWidth(40)
            slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            slider.setRange(-range_val, range_val)
            slider.setValue(0)
            val_lbl = QtWidgets.QLabel("0.0°")
            val_lbl.setFixedWidth(50)
            val_lbl.setStyleSheet("color: #ffffff; font-family: Consolas; font-size: 11px;")
            row.addWidget(lbl)
            row.addWidget(slider, stretch=1)
            row.addWidget(val_lbl)
            self.sim_manual_layout.addLayout(row)
            self.sim_sliders[name.lower()] = (slider, val_lbl)
    
        self.sim_vbox.addWidget(self.sim_manual_widget)
        self.left_layout.addWidget(self.sim_box)

        # 录制
        self.record_box = QtWidgets.QGroupBox("Data Recording")
        self.record_grid = QtWidgets.QGridLayout(self.record_box)
        self.record_grid.addWidget(QtWidgets.QLabel("Format"), 0, 0)
        self.cb_format = QtWidgets.QComboBox()
        self.cb_format.addItems(["CSV", "TXT"])
        self.record_grid.addWidget(self.cb_format, 0, 1)
        self.btn_start_record = QtWidgets.QPushButton("Start Record")
        self.btn_start_record.setStyleSheet("background-color: #17a2b8; color: white; font-size: 11px;")
        self.btn_start_record.setEnabled(False)
        self.record_grid.addWidget(self.btn_start_record, 1, 0)
        self.btn_stop_record = QtWidgets.QPushButton("End Record")
        self.btn_stop_record.setStyleSheet("background-color: #dc3545; color: white; font-size: 11px;")
        self.btn_stop_record.setEnabled(False)
        self.record_grid.addWidget(self.btn_stop_record, 1, 1)
        self.left_layout.addWidget(self.record_box)

        # 校准
        self.cal_box = QtWidgets.QGroupBox("Calibration Controls")
        self.cal_vbox = QtWidgets.QVBoxLayout(self.cal_box)
        for name in ["Gyro Calibrate", "Accel Calibrate", "Mag Calibrate"]:
            row = QtWidgets.QHBoxLayout()
            btn = QtWidgets.QPushButton(name)
            indicator = QtWidgets.QLabel()
            indicator.setFixedSize(12, 12)
            indicator.setStyleSheet("background-color: #28a745; border-radius: 6px;")
            row.addWidget(btn)
            row.addWidget(indicator)
            self.cal_vbox.addLayout(row)
        self.left_layout.addWidget(self.cal_box)

        # 算法
        self.algo_box = QtWidgets.QGroupBox("Algorithm")
        self.algo_vbox = QtWidgets.QVBoxLayout(self.algo_box)
        self.list_algo = QtWidgets.QListWidget()
        self.list_algo.addItems(["Mahony", "Madgwick", "EKF", "Raw Data Only"])
        self.list_algo.setCurrentRow(0)
        self.list_algo.setFixedHeight(95)
        self.algo_vbox.addWidget(self.list_algo)
        self.left_layout.addWidget(self.algo_box)

        # 流视图
        self.raw_box = QtWidgets.QGroupBox("Raw Data Stream")
        self.raw_vbox = QtWidgets.QVBoxLayout(self.raw_box)
        self.txt_raw_stream = QtWidgets.QListWidget()
        self.raw_vbox.addWidget(self.txt_raw_stream)
        self.left_layout.addWidget(self.raw_box, stretch=1)
        
        self.main_splitter.addWidget(self.left_widget)

        # ================= 2. 中间面板 =================
        self.center_widget = QtWidgets.QWidget()
        self.center_layout = QtWidgets.QVBoxLayout(self.center_widget)
        self.center_layout.setContentsMargins(5, 5, 5, 10)

        self.scene_box = QtWidgets.QGroupBox("3D Attitude (R=X P=Y Y=Z)")
        self.scene_vbox = QtWidgets.QVBoxLayout(self.scene_box)
        self.scene_vbox.setContentsMargins(8, 12, 8, 8)
    
        self.gl_view = gl.GLViewWidget()
        self.gl_view.setBackgroundColor('#1a1d24')
        self.gl_view.setCameraPosition(distance=12, elevation=25, azimuth=45)
        self.scene_vbox.addWidget(self.gl_view, stretch=4)

        self.compass = CompassWidget(self.gl_view)
        self.compass.move(15, 15)

        self.boards_layout = QtWidgets.QHBoxLayout()
        self.boards_layout.setSpacing(12)
        self.board_roll = QtWidgets.QLabel("0.0°")
        self.board_pitch = QtWidgets.QLabel("0.0°")
        self.board_yaw = QtWidgets.QLabel("0.0°")
        
        for title, val_label in [("Roll", self.board_roll), ("Pitch", self.board_pitch), ("Yaw", self.board_yaw)]:
            card_frame = QtWidgets.QFrame()
            card_frame.setStyleSheet("QFrame { background-color: #111317; border: 1px solid #2d3139; border-radius: 6px; } QLabel { border: none; }")
            card_vbox = QtWidgets.QVBoxLayout(card_frame)
            card_vbox.setContentsMargins(12, 8, 12, 8)
            
            title_lbl = QtWidgets.QLabel(title)
            title_lbl.setStyleSheet("color: #5c6370; font-size: 12px; font-weight: bold;")
            val_label.setFont(QtGui.QFont("Consolas", 26, QtGui.QFont.Weight.Bold))
            val_label.setStyleSheet("color: #ffffff; background: transparent;")
            
            card_vbox.addWidget(title_lbl)
            card_vbox.addWidget(val_label)
            self.boards_layout.addWidget(card_frame)
            
        self.scene_vbox.addLayout(self.boards_layout, stretch=1)
        self.center_layout.addWidget(self.scene_box)
        self.main_splitter.addWidget(self.center_widget)

        # ================= 3. 右侧面板 =================
        self.right_widget = QtWidgets.QWidget()
        self.right_layout = QtWidgets.QVBoxLayout(self.right_widget)
        self.right_layout.setContentsMargins(5, 5, 10, 10)

        self.wave_box = QtWidgets.QGroupBox("Real-time Data Visualization")
        self.wave_vbox = QtWidgets.QVBoxLayout(self.wave_box)
        self.wave_vbox.setSpacing(10)

        pg.setConfigOption('background', '#1a1d24')
        pg.setConfigOption('foreground', '#ffffff')

        self.plot_acc = pg.PlotWidget(title="<span style='color: #ffffff; font-weight: bold;'>Accelerometer (m/s^2)</span>")
        self.plot_gyro = pg.PlotWidget(title="<span style='color: #ffffff; font-weight: bold;'>Gyroscope (°/s)</span>")
        self.plot_mag = pg.PlotWidget(title="<span style='color: #ffffff; font-weight: bold;'>Magnetometer (mGauss)</span>")

        self.checkboxes = {}
        colors = ['#ff5555', '#55ff55', '#5555ff']
        
        for sensor, p_w in [('acc', self.plot_acc), ('gyro', self.plot_gyro), ('mag', self.plot_mag)]:
            h_layout = QtWidgets.QHBoxLayout()
            p_w.showGrid(x=True, y=True, alpha=0.15)
            p_w.getAxis('bottom').setLabel('Time (s)', color='#ffffff')
            h_layout.addWidget(p_w, stretch=6)

            cb_vbox = QtWidgets.QVBoxLayout()
            cb_vbox.setAlignment(QtCore.Qt.AlignCenter)
            title_lbl = QtWidgets.QLabel("Axes")
            title_lbl.setStyleSheet("color: #5c6370; font-size: 10px; font-weight: bold;")
            cb_vbox.addWidget(title_lbl)

            for idx, axis in enumerate(['x', 'y', 'z']):
                cb = QtWidgets.QCheckBox(axis.upper())
                cb.setChecked(True)
                cb.setStyleSheet(f"QCheckBox {{ color: {colors[idx]}; }}")
                cb_vbox.addWidget(cb)
                self.checkboxes[f"{sensor}_{axis}"] = cb
                
            h_layout.addLayout(cb_vbox, stretch=1)
            self.wave_vbox.addLayout(h_layout)

        self.right_layout.addWidget(self.wave_box)
        self.main_splitter.addWidget(self.right_widget)

        # 固定左侧面板宽度，中间和右侧设置最小宽度防止布局跳动
        self.left_widget.setFixedWidth(270)
        self.center_widget.setMinimumWidth(400)
        self.right_widget.setMinimumWidth(400)
        self.main_splitter.setStretchFactor(0, 0)  # 左侧不拉伸
        self.main_splitter.setStretchFactor(1, 1)  # 中间等比拉伸
        self.main_splitter.setStretchFactor(2, 1)  # 右侧等比拉伸
        # 设置 splitter 初始尺寸比例
        self.main_splitter.setSizes([270, 565, 565])

        # 状态栏
        self.status_bar = QtWidgets.QStatusBar(MainWindow)
        MainWindow.setStatusBar(self.status_bar)
        self.lbl_status_hz = QtWidgets.QLabel("Data Rate (Hz): 0")
        self.lbl_status_drop = QtWidgets.QLabel("Drop Rate (%): 0.0")
        self.lbl_status_indicator = QtWidgets.QLabel("Connection: Closed")
        self.status_bar.addPermanentWidget(self.lbl_status_hz, 1)
        self.status_bar.addPermanentWidget(self.lbl_status_drop, 1)
        self.status_bar.addPermanentWidget(self.lbl_status_indicator, 1)