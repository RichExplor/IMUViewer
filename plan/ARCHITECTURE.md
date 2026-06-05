# IMUViewer 架构设计文档

> **版本**: v1.0  
> **更新日期**: 2026-06-05  
> **项目定位**: 用于可视化和调试 IMU 实时数据与离线数据的跨平台上位机

---

## 目录

- [IMUViewer 架构设计文档](#imuviewer-架构设计文档)
  - [目录](#目录)
  - [1. 项目概述](#1-项目概述)
  - [2. 技术栈与依赖](#2-技术栈与依赖)
  - [3. 系统架构总览](#3-系统架构总览)
  - [4. 目录结构](#4-目录结构)
  - [5. 核心模块详解](#5-核心模块详解)
    - [5.1 应用入口 — `main.py`](#51-应用入口--mainpy)
    - [5.2 算法引擎 — `core/algorithm.py`](#52-算法引擎--corealgorithmpy)
    - [5.3 串口数据线程 — `core/serial_thread.py`](#53-串口数据线程--coreserial_threadpy)
    - [5.4 模拟器线程 — `core/simulator.py`](#54-模拟器线程--coresimulatorpy)
    - [5.5 OBJ 模型加载器 — `core/obj_loader.py`](#55-obj-模型加载器--coreobj_loaderpy)
    - [5.6 UI 布局 — `ui/main_window.py`](#56-ui-布局--uimain_windowpy)
    - [5.7 自定义组件 — `ui/widgets.py`](#57-自定义组件--uiwidgetspy)
  - [6. 数据流架构](#6-数据流架构)
    - [6.1 串口模式数据流](#61-串口模式数据流)
    - [6.2 模拟器模式数据流](#62-模拟器模式数据流)
    - [6.3 统一数据包格式](#63-统一数据包格式)
  - [7. 线程模型与并发安全](#7-线程模型与并发安全)
    - [7.1 线程架构](#71-线程架构)
    - [7.2 并发安全机制](#72-并发安全机制)
  - [8. 信号槽连接图](#8-信号槽连接图)
  - [9. UI 布局架构](#9-ui-布局架构)
  - [10. 3D 可视化架构](#10-3d-可视化架构)
    - [10.1 场景组成](#101-场景组成)
    - [10.2 姿态更新机制](#102-姿态更新机制)
  - [11. 数据录制架构](#11-数据录制架构)
  - [12. CI/CD 与打包](#12-cicd-与打包)
    - [12.1 GitHub Actions 流水线](#121-github-actions-流水线)
  - [13. 设计决策与权衡](#13-设计决策与权衡)
  - [14. 已知限制与未来扩展](#14-已知限制与未来扩展)
    - [当前限制](#当前限制)
    - [未来扩展方向](#未来扩展方向)

---

## 1. 项目概述

IMUViewer 是一款面向 IMU（惯性测量单元）传感器的实时数据可视化与调试工具，主要功能包括：

- **串口实时采集**：通过串口连接物理 IMU 传感器，实时接收并解析 9 轴传感器数据（加速度计、陀螺仪、磁力计）
- **模拟数据源**：无需物理硬件即可通过正弦波或手动模式生成模拟 IMU 数据，便于开发调试
- **姿态融合算法**：内置 Mahony 9 轴姿态融合算法，将原始传感器数据解算为 Roll/Pitch/Yaw 欧拉角
- **3D 姿态可视化**：基于 OBJ 模型实时展示 IMU 本体姿态，配合电子罗盘指示航向
- **实时曲线绘制**：三轴加速度、角速度、磁场强度的实时波形显示，支持独立显隐控制
- **数据录制**：支持 CSV/TXT 格式将传感器数据与姿态解算结果保存至本地
- **跨平台打包**：基于 GitHub Actions CI/CD 自动构建 Windows 可执行文件

---

## 2. 技术栈与依赖

| 依赖库 | 版本要求 | 用途 |
|--------|----------|------|
| **Python** | 3.13 | 运行时环境 |
| **PyQt5** | - | GUI 框架，提供窗口系统、信号槽机制、线程基础设施 |
| **pyqtgraph** | - | 高性能实时数据曲线绘制 |
| **pyqtgraph.opengl** | - | 3D OpenGL 可视化（GLViewWidget、GLMeshItem、GLGridItem） |
| **numpy** | - | 数值计算（矩阵运算、四元数归一化、数组操作） |
| **pyserial** | - | 串口通信（`serial.Serial` 及端口枚举） |

---

## 3. 系统架构总览

IMUViewer 采用 **分层架构 + 生产者-消费者模式**，整体分为三层：

```
┌─────────────────────────────────────────────────────────────┐
│                     表现层 (Presentation)                    │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────┐  │
│  │ Ui_IMUViewer │  │ CompassWidget│  │ pyqtgraph 曲线/3D │  │
│  │  (静态布局)   │  │  (电子罗盘)   │  │   (渲染组件)      │  │
│  └──────┬───────┘  └──────┬───────┘  └────────┬──────────┘  │
│         │                 │                    │             │
│  ┌──────┴─────────────────┴────────────────────┴──────────┐  │
│  │              IMUViewer (主窗口 / 控制器)                 │  │
│  │     信号槽绑定 · 数据缓存 · UI 定时刷新 · 事件分发       │  │
│  └──────────────────────┬──────────────────────────────────┘  │
├─────────────────────────┼───────────────────────────────────┤
│                   数据层 (Data Acquisition)                  │
│  ┌──────────────────────┴──────────────────────────────────┐  │
│  │              数据源抽象 (统一信号接口)                     │  │
│  │  ┌───────────────────┐  ┌────────────────────────────┐  │  │
│  │  │ IMUSerialThread   │  │ IMUSimulatorThread         │  │  │
│  │  │ (串口采集线程)     │  │ (模拟数据线程)              │  │  │
│  │  └─────────┬─────────┘  └──────────┬─────────────────┘  │  │
│  └────────────┼───────────────────────┼────────────────────┘  │
├───────────────┼───────────────────────┼──────────────────────┤
│               │    算法层 (Algorithm)  │                      │
│  ┌────────────┴───────────────────────┴────────────────────┐  │
│  │                  MahonyAHRS                              │  │
│  │         9 轴 Mahony 姿态融合算法引擎                      │  │
│  └─────────────────────────────────────────────────────────┘  │
├──────────────────────────────────────────────────────────────┤
│                    资源层 (Resources)                         │
│  ┌──────────────────┐  ┌──────────────────────────────────┐  │
│  │  OBJ 模型加载器   │  │  obj/car.obj + car_obj_cache.npz│  │
│  │  (解析/缓存/渲染) │  │  (3D 模型文件 + 解析缓存)        │  │
│  └──────────────────┘  └──────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

**核心设计原则**：

- **关注点分离**：UI 布局（`Ui_IMUViewer`）与业务逻辑（`IMUViewer`）严格分离
- **生产者-消费者**：后台线程生产数据 → 信号缓存 → UI 定时器消费刷新，避免跨线程直接操作 UI
- **数据源抽象**：串口线程与模拟器线程对外暴露统一信号接口，主窗口无需关心数据来源

---

## 4. 目录结构

```
IMUViewer/
├── main.py                    # 应用入口，IMUViewer 主窗口类定义
├── README.md                  # 项目说明与使用指南
├── requirements.txt           # Python 依赖清单
├── favicon.ico                # 应用图标（打包用）
├── logo.png                   # 项目 Logo
├── .gitignore                 # Git 忽略规则
│
├── .github/
│   └── workflows/
│       └── build.yml          # GitHub Actions CI/CD 流水线配置
│
├── core/                      # 核心业务逻辑层
│   ├── algorithm.py           # Mahony 9 轴姿态融合算法
│   ├── serial_thread.py       # 串口数据采集线程
│   ├── simulator.py           # IMU 模拟数据生成线程
│   └── obj_loader.py          # OBJ 3D 模型加载器（含缓存机制）
│
├── ui/                        # UI 表现层
│   ├── main_window.py         # 主窗口静态布局定义（Ui_IMUViewer）
│   └── widgets.py             # 自定义 Qt 组件（CompassWidget）
│
└── obj/                       # 3D 模型资源
    ├── car.obj                # 汽车 OBJ 模型文件
    └── car_obj_cache.npz      # OBJ 解析结果缓存（首次加载后自动生成）
```

---

## 5. 核心模块详解

### 5.1 应用入口 — [`main.py`](main.py)

**职责**：应用程序入口点，定义主窗口类 [`IMUViewer`](main.py:15)，协调所有模块。

[`IMUViewer`](main.py:15) 继承自 `QtWidgets.QMainWindow`，是整个应用的**控制器（Controller）**，承担以下职责：

| 职责 | 实现方式 | 代码位置 |
|------|----------|----------|
| UI 挂载 | 组合 `Ui_IMUViewer` 对象 | [`__init__`](main.py:16) |
| 后台线程管理 | 持有 `IMUSerialThread` 和 `IMUSimulatorThread` 实例 | [`__init__`](main.py:23) |
| 数据缓存 | 通过 `QMutex` 保护 `_latest_data` / `_latest_raw` | [`_cache_data`](main.py:84) / [`_cache_raw`](main.py:90) |
| UI 定时刷新 | 30Hz `QTimer` 驱动 [`_refresh_ui()`](main.py:96) | [`__init__`](main.py:67) |
| 信号槽绑定 | 连接线程信号 → 缓存方法，按钮信号 → 动作方法 | [`__init__`](main.py:43) |
| 数据源切换 | 串口/模拟器模式切换逻辑 | [`_on_simulator_radio_toggled`](main.py:113) |
| 3D 模型管理 | 加载 OBJ 模型并应用姿态变换 | [`build_3d_airplane`](main.py:168) / [`update_ui_data`](main.py:253) |
| 曲线数据管理 | 维护滑动窗口数据缓冲区 | [`update_ui_data`](main.py:253) |
| 统计信息 | 1Hz 定时器计算数据率和丢包率 | [`calculate_fps_hz`](main.py:278) |
| 生命周期管理 | 关闭事件中停止线程和定时器 | [`closeEvent`](main.py:293) |

**关键设计**：`IMUViewer` 不直接在信号回调中刷新 UI，而是通过 **缓存 + 定时器** 模式解耦：

```python
# 信号回调（后台线程上下文）→ 仅缓存
self.serial_thread.data_received.connect(self._cache_data)

# 30Hz 定时器（UI 线程上下文）→ 批量刷新
self.ui_refresh_timer.timeout.connect(self._refresh_ui)
```

---

### 5.2 算法引擎 — [`core/algorithm.py`](core/algorithm.py)

**职责**：实现 9 轴 Mahony 互补滤波姿态融合算法。

[`MahonyAHRS`](core/algorithm.py:4) 类核心接口：

| 方法 | 功能 |
|------|------|
| [`__init__(kp, ki)`](core/algorithm.py:6) | 初始化比例/积分增益，四元数初始为单位四元数 |
| [`update_9dof(ax,ay,az,gx,gy,gz,mx,my,mz,dt)`](core/algorithm.py:17) | 输入 9 轴原始数据 + 时间步长，更新内部四元数 |
| [`get_euler()`](core/algorithm.py:76) | 从四元数提取 Roll/Pitch/Yaw 欧拉角（度） |

**算法流程**：

```
原始 9 轴数据
    │
    ▼
加速度计 & 磁力计归一化
    │
    ▼
重力参考方向 (v) & 地磁参考方向 (w) 计算
    │
    ▼
叉积误差 e = (a×v) + (m×w)  ← 加速度计与磁力计互补校正
    │
    ▼
PI 控制器: 陀螺仪修正 = Kp·e + Ki·∫e·dt
    │
    ▼
中点龙格-库塔法更新四元数
    │
    ▼
四元数归一化
    │
    ▼
四元数 → 欧拉角转换 (Roll/Pitch/Yaw)
```

**关键参数**：

- `kp=0.5`（默认）/ `kp=1.0`（线程中使用）：比例增益，控制收敛速度
- `ki=0.0`：积分增益，默认关闭（纯比例控制，避免静态漂移累积）
- 陀螺仪数据采用**中点平均**（当前帧与上一帧均值），等效一阶龙格-库塔

---

### 5.3 串口数据线程 — [`core/serial_thread.py`](core/serial_thread.py)

**职责**：后台高频串口数据接收、帧解析、标度转换、姿态融合与数据发射。

[`IMUSerialThread`](core/serial_thread.py:11) 继承自 `QtCore.QThread`，核心属性：

| 属性 | 说明 |
|------|------|
| [`data_received`](core/serial_thread.py:13) | `pyqtSignal(dict)` — 处理后的数据包信号 |
| [`raw_string_received`](core/serial_thread.py:14) | `pyqtSignal(str)` — 原始数据字符串信号 |
| [`log_received`](core/serial_thread.py:15) | `pyqtSignal(str)` — 日志消息信号 |
| [`fusion_engine`](core/serial_thread.py:21) | `MahonyAHRS` 实例，在线程内执行姿态解算 |
| [`packet_count`](core/serial_thread.py:30) | 每秒数据包计数（用于 Hz 统计） |
| [`drop_count`](core/serial_thread.py:31) | 校验失败丢包计数 |

**串口协议解析**（[`run()`](core/serial_thread.py:90)）：

```
帧格式 (24 字节):
┌──────┬──────┬──────┬──────┬──────────────┬──────────┐
│ 0x4E │ 0x4A │ 0x13 │ 0x01 │ 9×int16 数据  │ 2字节校验 │
│ 帧头1 │ 帧头2 │ 帧头3 │ 帧头4 │ (18 bytes)    │ (sum)    │
└──────┴──────┴──────┴──────┴──────────────┴──────────┘
```

- **帧头**：`0x4E 0x4A 0x13 0x01`（4 字节）
- **数据**：9 个 `int16` 小端序（AccX/Y/Z, GyroX/Y/Z, MagX/Y/Z）
- **校验**：前 22 字节之和的低 16 位（`& 0xFFFF`）

**标度转换**：

| 传感器 | 量程 | 标度因子 |
|--------|------|----------|
| 加速度计 | ±16g | `1/32768 × 9.8 × 16` → m/s² |
| 陀螺仪 | ±2000°/s | `1/32768 × 2000` → °/s（再转弧度用于融合） |
| 磁力计 | 原始值 | `1.0`（无标度转换） |

**数据流**：

```
串口读取 → rx_buffer 累积 → 帧头匹配 → 校验验证
    → 标度转换 → Mahony 融合 → 构造 dict → emit data_received
    → 录制写入（如正在录制）
```

---

### 5.4 模拟器线程 — [`core/simulator.py`](core/simulator.py)

**职责**：无需物理硬件，以 50Hz 频率生成模拟 IMU 传感器数据。

[`IMUSimulatorThread`](core/simulator.py:18) 继承自 `QtCore.QThread`，支持两种模式：

| 模式 | 说明 | 数据生成方式 |
|------|------|-------------|
| **正弦波** (`'sine'`) | 模拟自然运动 | 不同频率/相位的正弦函数 + 微小噪声 |
| **手动** (`'manual'`) | 用户滑条控制 | 根据欧拉角反推理想重力/地磁分量 |

**正弦波模式**（[`_generate_sine_data()`](core/simulator.py:165)）：

- 加速度计：重力 1g + 微扰正弦（0.3~0.7Hz）
- 陀螺仪：缓慢旋转正弦（0.15~0.25Hz）+ 高频微振（2.7~3.5Hz）
- 磁力计：地磁场基值 + 微扰正弦（0.08~0.12Hz）
- 姿态解算：通过 `MahonyAHRS` 融合

**手动模式**（[`_generate_manual_data()`](core/simulator.py:206)）：

- 根据用户设定的 Roll/Pitch/Yaw 反推：
  - 加速度计 = 重力在体坐标系的投影
  - 陀螺仪 = 零（静止）
  - 磁力计 = 地磁场在体坐标系的投影（水平 ~30μT，垂直 ~40μT）
- 欧拉角直接使用手动设定值，不经融合算法

**与串口线程的统一接口**：

两个线程对外暴露完全相同的三个信号：`data_received(dict)`、`raw_string_received(str)`、`log_received(str)`，使主窗口可以统一处理。

---

### 5.5 OBJ 模型加载器 — [`core/obj_loader.py`](core/obj_loader.py)

**职责**：解析 OBJ 3D 模型文件，生成 pyqtgraph 可渲染的 `GLMeshItem` 列表。

**核心函数**：[`load_obj_as_mesh_items(obj_path, target_size, cache_dir)`](core/obj_loader.py:314)

**处理流水线**：

```
OBJ 文件路径
    │
    ▼
计算文件哈希 (MD5, 头尾各1MB)
    │
    ▼
检查 .npz 缓存 ──命中──→ 直接加载缓存数据
    │                        │
    │ 未命中                  ▼
    ▼                    居中+缩放
纯 Python 逐行解析              │
(v / vn / f / usemtl)           ▼
    │                      按材质分组构建
    ▼                      MeshData 列表
保存 .npz 缓存                   │
    │                            ▼
    └────────────────────→ 创建 GLMeshItem 列表
                                │
                                ▼
                           返回 [GLMeshItem, ...]
```

**关键设计**：

| 特性 | 实现 |
|------|------|
| **材质颜色映射** | [`MATERIAL_COLOR_MAP`](core/obj_loader.py:23) 关键词优先级匹配表，覆盖轮胎/卡钳/车灯/玻璃等 20+ 种材质 |
| **坐标系转换** | [`_center_and_scale()`](core/obj_loader.py:190) 中 Blender Y-up → pyqtgraph Z-up（交换 Y/Z 列） |
| **多边形三角化** | 扇形分割法，将 n 边形拆为 n-2 个三角形 |
| **缓存机制** | 首次解析后保存 `.npz` 压缩缓存，后续启动秒级加载；哈希校验确保缓存与源文件一致 |
| **透明度处理** | `color[3] < 1.0` 时使用 `glOptions='translucent'`，否则 `'opaque'` |

---

### 5.6 UI 布局 — [`ui/main_window.py`](ui/main_window.py)

**职责**：纯静态 UI 布局定义，**零业务逻辑**。

[`Ui_IMUViewer`](ui/main_window.py:7) 遵循 PyQt 的 `Ui_` 前缀约定，仅负责：

- 创建所有 Qt 控件实例
- 设置布局层次与尺寸约束
- 应用全局暗色主题样式表
- 暴露控件引用供 `IMUViewer` 绑定信号槽

**暴露的关键控件**：

| 控件引用 | 类型 | 用途 |
|----------|------|------|
| `rb_serial` / `rb_simulator` | `QRadioButton` | 数据源选择 |
| `cb_port` / `cb_baud` | `QComboBox` | 串口参数 |
| `btn_connect` | `QPushButton` | 连接/断开按钮 |
| `rb_sim_sine` / `rb_sim_manual` | `QRadioButton` | 模拟模式选择 |
| `sim_sliders` | `dict[str, (QSlider, QLabel)]` | 手动模式 Roll/Pitch/Yaw 滑条 |
| `cb_format` / `btn_start_record` / `btn_stop_record` | 各控件 | 录制控制 |
| `list_algo` | `QListWidget` | 算法选择列表 |
| `gl_view` | `GLViewWidget` | 3D 视图 |
| `compass` | `CompassWidget` | 电子罗盘 |
| `board_roll` / `board_pitch` / `board_yaw` | `QLabel` | 欧拉角数值显示 |
| `plot_acc` / `plot_gyro` / `plot_mag` | `PlotWidget` | 实时曲线图 |
| `checkboxes` | `dict[str, QCheckBox]` | 曲线显隐控制 |
| `txt_raw_stream` | `QListWidget` | 原始数据流视图 |
| `status_bar` / `lbl_status_hz` / `lbl_status_drop` / `lbl_status_indicator` | 各控件 | 状态栏 |

---

### 5.7 自定义组件 — [`ui/widgets.py`](ui/widgets.py)

**职责**：自定义 Qt 绘制组件。

[`CompassWidget`](ui/widgets.py:4) — 电子罗盘组件：

- 固定尺寸 90×90 像素，覆盖在 3D 视图左上角
- 绘制圆形底盘 + N/S/W/E 方位标注
- 红色三角指针指向北方，蓝色三角指向南方
- 通过 [`set_yaw(yaw)`](ui/widgets.py:11) 更新航向角，触发重绘

---

## 6. 数据流架构

### 6.1 串口模式数据流

```
┌────────────┐     ┌──────────────────────────────────────────────┐
│  IMU 传感器  │────▶│           IMUSerialThread.run()              │
│  (硬件设备)  │串口  │                                              │
└────────────┘读取  │  rx_buffer → 帧头匹配 → 校验 → 标度转换       │
                       │                    │                        │
                       │            MahonyAHRS.update_9dof()         │
                       │                    │                        │
                       │            MahonyAHRS.get_euler()           │
                       │                    │                        │
                       │     ┌──────────────┴──────────────┐        │
                       │     ▼                              ▼       │
                       │  data_received              raw_string_     │
                       │    (dict)                   received(str)   │
                       └─────┬──────────────────────────┬───────────┘
                             │                          │
                    ┌────────▼────────┐        ┌────────▼────────┐
                    │  _cache_data()  │        │  _cache_raw()   │
                    │  (QMutex 保护)   │        │  (QMutex 保护)   │
                    └────────┬────────┘        └────────┬────────┘
                             │                          │
                    ┌────────▼──────────────────────────▼────────┐
                    │           _refresh_ui() (30Hz QTimer)       │
                    │                                            │
                    │  ┌─────────────┐  ┌──────────────────────┐ │
                    │  │ 3D 模型旋转  │  │ 曲线数据追加 + 重绘   │ │
                    │  │ 罗盘更新     │  │ 欧拉角数值更新        │ │
                    │  │ 原始流显示   │  │                      │ │
                    │  └─────────────┘  └──────────────────────┘ │
                    └────────────────────────────────────────────┘
```

### 6.2 模拟器模式数据流

```
┌──────────────────────────────────────────────────┐
│           IMUSimulatorThread.run() (50Hz)         │
│                                                    │
│  ┌──────────────┐     ┌────────────────────────┐  │
│  │ 正弦波模式    │     │ 手动模式                │  │
│  │ _generate_   │     │ _generate_manual_data() │  │
│  │ sine_data()  │     │                        │  │
│  │              │     │ 滑条 → set_manual_euler │  │
│  │ Mahony 融合  │     │ 直接使用欧拉角          │  │
│  └──────┬───────┘     └───────────┬────────────┘  │
│         └──────────┬──────────────┘                │
│                    ▼                               │
│         data_received / raw_string_received        │
└────────────────────┬───────────────────────────────┘
                     │
                     ▼
            （同串口模式的缓存 → 定时刷新路径）
```

### 6.3 统一数据包格式

两个数据源发射的 `dict` 格式完全一致：

```python
{
    'acc':   [ax, ay, az],      # 加速度计 (m/s²)
    'gyro':  [gx, gy, gz],      # 陀螺仪 (°/s)
    'mag':   [mx, my, mz],      # 磁力计 (原始单位)
    'euler': [roll, pitch, yaw]  # 欧拉角 (度)
}
```

---

## 7. 线程模型与并发安全

### 7.1 线程架构

```
┌─────────────────────────────────────────────────┐
│                  Main Thread (UI)                 │
│                                                   │
│  IMUViewer (QMainWindow)                         │
│  ├─ Ui_IMUViewer (控件树)                         │
│  ├─ ui_refresh_timer (30Hz)                      │
│  ├─ hz_timer (1Hz)                               │
│  └─ QMutex (_data_mutex)                         │
│                                                   │
│  持有引用 (不直接调用 run):                        │
│  ├─ IMUSerialThread (QThread) ──┐                │
│  └─ IMUSimulatorThread (QThread)──┤               │
│                                    │               │
├────────────────────────────────────┼──────────────┤
│                    后台线程         │               │
│  ┌─────────────────────────────────┴──────────┐   │
│  │                                             │   │
│  │  IMUSerialThread.run()                      │   │
│  │  └─ 串口读取循环 (while self.running)        │   │
│  │                                             │   │
│  │  IMUSimulatorThread.run()                   │   │
│  │  └─ 模拟数据生成循环 (while self._running)   │   │
│  │                                             │   │
│  └─────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────┘
```

### 7.2 并发安全机制

| 机制 | 保护对象 | 说明 |
|------|----------|------|
| **QMutex** (`_data_mutex`) | `_latest_data` / `_latest_raw` | 后台线程写缓存 ↔ UI 定时器读缓存互斥 |
| **Qt 信号槽** (跨线程自动队列化) | `data_received` / `raw_string_received` / `log_received` | QThread 发射的信号自动通过队列传递到 UI 线程 |
| **QTimer 驱动刷新** | 所有 UI 控件更新 | 仅在 UI 线程的定时器回调中操作 UI，避免跨线程直接访问 |

**关键安全约束**：

- 后台线程**绝不**直接调用任何 UI 更新方法
- 信号回调 [`_cache_data()`](main.py:84) / [`_cache_raw()`](main.py:90) 仅操作受 mutex 保护的缓存变量
- UI 刷新 [`_refresh_ui()`](main.py:96) 在 mutex 保护下取出数据后立即释放锁，再进行 UI 操作

---

## 8. 信号槽连接图

```
┌──────────────────────┐        ┌──────────────────────────────┐
│   IMUSerialThread    │        │       IMUViewer              │
│                      │        │                              │
│ data_received ───────┼───────▶│ _cache_data()               │
│ raw_string_received ─┼───────▶│ _cache_raw()                │
│ log_received ────────┼───────▶│ show_status_message()       │
└──────────────────────┘        │                              │
                                │                              │
┌──────────────────────┐        │                              │
│  IMUSimulatorThread  │        │                              │
│                      │        │                              │
│ data_received ───────┼───────▶│ _cache_data()               │
│ raw_string_received ─┼───────▶│ _cache_raw()                │
│ log_received ────────┼───────▶│ show_status_message()       │
└──────────────────────┘        │                              │
                                │                              │
┌──────────────────────┐        │                              │
│   Ui_IMUViewer 控件   │        │                              │
│                      │        │                              │
│ btn_connect.clicked ─┼───────▶│ toggle_connection()         │
│ rb_simulator.toggled ┼───────▶│ _on_simulator_radio_toggled│
│ rb_sim_manual.toggled┼───────▶│ _on_sim_manual_radio_toggled│
│ slider.valueChanged ─┼───────▶│ on_sim_slider_changed()     │
│ btn_start_record ────┼───────▶│ start_recording_clicked()   │
│ btn_stop_record ─────┼───────▶│ stop_recording_clicked()    │
│ list_algo.currentRow ┼───────▶│ change_algorithm()          │
│ checkbox.stateChanged┼───────▶│ toggle_curve_visible()      │
└──────────────────────┘        │                              │
                                │                              │
┌──────────────────────┐        │                              │
│     QTimer           │        │                              │
│                      │        │                              │
│ ui_refresh_timer ────┼───────▶│ _refresh_ui()               │
│ hz_timer ────────────┼───────▶│ calculate_fps_hz()          │
└──────────────────────┘        └──────────────────────────────┘
```

---

## 9. UI 布局架构

主窗口采用 `QSplitter`（水平）三栏布局：

```
┌─────────────────────────────────────────────────────────────────────┐
│  IMUViewer (QMainWindow, 1400×800)                                  │
│                                                                      │
│  ┌─────────────┬──────────────────────┬──────────────────────────┐  │
│  │  左侧面板    │     中间面板          │       右侧面板           │  │
│  │  (固定270px) │   (弹性拉伸)          │     (弹性拉伸)           │  │
│  │             │                      │                          │  │
│  │ ┌─────────┐ │ ┌──────────────────┐ │ ┌──────────────────────┐ │  │
│  │ │Data     │ │ │3D Attitude       │ │ │Real-time Data        │ │  │
│  │ │Source   │ │ │                  │ │ │Visualization         │ │  │
│  │ ├─────────┤ │ │  ┌────────────┐  │ │ │                      │ │  │
│  │ │Port &   │ │ │  │ GLViewWidget│  │ │ │ ┌──────────────────┐ │ │  │
│  │ │Baud Rate│ │ │  │  (3D模型+   │  │ │ │ │Accelerometer    │ │ │  │
│  │ ├─────────┤ │ │  │   网格+轴)  │  │ │ │ │  曲线图 + XYZ   │ │ │  │
│  │ │Connect  │ │ │  └────────────┘  │ │ │ │  显隐控制        │ │ │  │
│  │ │Button   │ │ │  ┌────────────┐  │ │ │ ├──────────────────┤ │ │  │
│  │ ├─────────┤ │ │  │CompassWidget│  │ │ │ │Gyroscope        │ │ │  │
│  │ │Simulator│ │ │  └────────────┘  │ │ │ │  曲线图 + XYZ   │ │ │  │
│  │ │Controls │ │ │  ┌───┬───┬───┐   │ │ │ ├──────────────────┤ │ │  │
│  │ ├─────────┤ │ │  │Roll│Pit│Yaw│   │ │ │ │Magnetometer     │ │ │  │
│  │ │Data     │ │ │  │数值│数值│数值│  │ │ │ │  曲线图 + XYZ   │ │ │  │
│  │ │Recording│ │ │  └───┴───┴───┘   │ │ │ └──────────────────┘ │ │  │
│  │ ├─────────┤ │ └──────────────────┘ │ └──────────────────────┘ │  │
│  │ │Calibrat-│ │                      │                          │  │
│  │ │ion Ctrl │ │                      │                          │  │
│  │ ├─────────┤ │                      │                          │  │
│  │ │Algorithm│ │                      │                          │  │
│  │ ├─────────┤ │                      │                          │  │
│  │ │Raw Data │ │                      │                          │  │
│  │ │Stream   │ │                      │                          │  │
│  │ └─────────┘ │                      │                          │  │
│  └─────────────┴──────────────────────┴──────────────────────────┘  │
│                                                                      │
│  ┌────────────────────────────────────────────────────────────────┐  │
│  │ Status Bar: [Data Rate (Hz)] [Drop Rate (%)] [Connection]     │  │
│  └────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
```

**布局策略**：

- 左侧面板 `setFixedWidth(270)`，不参与弹性拉伸
- 中间/右侧面板 `setMinimumWidth(400)`，等比弹性拉伸（`stretchFactor = 1`）
- 初始尺寸比例：270 : 565 : 565

---

## 10. 3D 可视化架构

### 10.1 场景组成

```
GLViewWidget (camera: distance=12, elevation=25°, azimuth=45°)
│
├── GLGridItem (18×18, spacing=1, depthValue=10)
│
├── GLLinePlotItem × 3 (参考坐标轴, 半透明)
│   ├── X轴: 绿色 (0,1,0,0.3), 长度4
│   ├── Y轴: 红色 (1,0,0,0.3), 长度4
│   └── Z轴: 蓝色 (0,0,1,0.3), 长度4
│
├── GLMeshItem × N (OBJ 模型网格, 按材质分组)
│   ├── 轮胎 (深黑, opaque)
│   ├── 车身 (白色, opaque)
│   ├── 玻璃 (半透明, translucent)
│   └── ... (其他材质组)
│
└── CompassWidget (覆盖在左上角, 90×90px)
```

### 10.2 姿态更新机制

```python
# update_ui_data() 中:
transform = QtGui.QMatrix4x4()
transform.setToIdentity()
transform.rotate(yaw,   0, 0, 1)   # Z-Y-X 旋转顺序
transform.rotate(pitch, 0, 1, 0)
transform.rotate(roll,  1, 0, 0)

for part in self.plane_parts:
    part.setTransform(transform)     # 统一变换所有网格部件
```

- 旋转顺序：Yaw → Pitch → Roll（Z-Y-X 内旋）
- 所有 OBJ 网格部件共享同一变换矩阵，保证模型整体旋转一致性

---

## 11. 数据录制架构

两个数据源线程均内置录制功能，架构一致：

```
┌─────────────────────────────────────────────────┐
│              录制状态机                            │
│                                                   │
│  [未录制] ──start_recording()──▶ [录制中]          │
│     ▲                            │               │
│     │                            stop_recording()│
│     └────────────────────────────┘               │
│                                                   │
│  支持格式: CSV (逗号分隔) / TXT (制表符分隔)       │
│  文件命名: imu_record_YYYYMMDD_HHMMSS.{csv|txt}  │
│           imu_sim_record_YYYYMMDD_HHMMSS.{csv|txt}│
│                                                   │
│  数据列: Timestamp, AccX/Y/Z, GyroX/Y/Z,        │
│          MagX/Y/Z, Roll, Pitch, Yaw              │
└─────────────────────────────────────────────────┘
```

**录制与数据采集在同一线程内**：录制写入操作在后台线程的 `run()` 循环中执行，不阻塞 UI 线程。

---

## 12. CI/CD 与打包

### 12.1 GitHub Actions 流水线

```
触发条件: push tag 'v*' 或手动 workflow_dispatch
    │
    ▼
┌─────────────────────────────────────────┐
│  Job: build (windows-latest)            │
│                                         │
│  1. Checkout code (actions/checkout@v4) │
│  2. Setup Python 3.13                  │
│  3. pip install -r requirements.txt    │
│  4. pip install pyinstaller pyopengl   │
│  5. PyInstaller 打包:                  │
│     -F -w --hidden-import=OpenGL       │
│     -n IMUViewer --icon=favicon.ico    │
│     --add-data "obj;obj"               │
│  6. Upload artifact (dist/*.exe)       │
└─────────────────────────────────────────┘
```

**PyInstaller 打包参数**：

| 参数 | 说明 |
|------|------|
| `-F` | 单文件模式 |
| `-w` | 隐藏控制台窗口（GUI 程序） |
| `--hidden-import=OpenGL` | 显式包含 PyOpenGL（pyqtgraph.opengl 依赖） |
| `--add-data "obj;obj"` | 打包 OBJ 模型资源目录 |

---

## 13. 设计决策与权衡

| 决策 | 选择 | 理由 | 权衡 |
|------|------|------|------|
| UI 框架 | PyQt5 | 成熟稳定，信号槽机制天然支持跨线程通信 | 包体积较大，GPL 许可证限制 |
| 曲线绘制 | pyqtgraph | 基于 Qt 场景图，高性能实时绘制，与 PyQt5 无缝集成 | API 不如 matplotlib 丰富 |
| 3D 渲染 | pyqtgraph.opengl | 无需额外 GUI 集成，直接嵌入 Qt 布局 | 功能有限，不适合复杂 3D 场景 |
| 姿态融合 | Mahony 互补滤波 | 计算量小，9 轴融合效果好，适合嵌入式/上位机 | 未实现 Madgwick/EKF（UI 中已预留选项） |
| 数据缓存 | QMutex + 单值缓存 | 简单高效，30Hz 刷新足够流畅 | 丢弃中间帧（仅保留最新值），不保证每帧都显示 |
| OBJ 解析 | 纯 Python 逐行解析 | 无额外依赖，完全可控 | 首次加载大文件较慢（通过缓存缓解） |
| 模型缓存 | .npz 压缩格式 | numpy 原生支持，加载快，含哈希校验 | 缓存文件需与 OBJ 同目录管理 |
| UI/逻辑分离 | Ui_ 类 + 主窗口组合 | 遵循 PyQt 设计惯例，布局可独立修改 | 控件引用通过属性暴露，耦合度中等 |

---

## 14. 已知限制与未来扩展

### 当前限制

1. **算法引擎**：UI 中列出 Mahony/Madgwick/EKF/Raw Data Only 四个选项，但仅 Mahony 和 Raw Data Only 实际实现
2. **校准功能**：Calibration Controls 面板中的三个校准按钮未连接实际逻辑
3. **数据缓存**：单值缓存模式会丢弃高频数据中的中间帧，仅保留最新值
4. **离线数据**：当前仅支持实时数据，不支持离线数据文件回放
5. **串口协议**：硬编码 24 字节帧格式，仅支持特定 IMU 传感器协议
6. **OBJ 加载**：不支持 MTL 材质文件，颜色通过关键词映射表硬编码
7. **跨平台打包**：CI/CD 仅配置 Windows 构建，未覆盖 Linux/macOS

### 未来扩展方向

1. **算法扩展**：实现 Madgwick 和 EKF 姿态融合算法
2. **校准流程**：实现陀螺仪零偏校准、加速度计六面校准、磁力计椭球校准
3. **离线回放**：支持加载 CSV/TXT 录制文件，以原始速率回放数据
4. **协议抽象**：将串口协议解析抽象为可插拔策略，支持不同 IMU 传感器
5. **MTL 材质支持**：解析 MTL 文件获取真实材质颜色，替代硬编码映射
6. **多平台打包**：扩展 CI/CD 支持 Linux AppImage 和 macOS .dmg
7. **数据缓存优化**：引入环形缓冲区，支持全量数据保留与回溯查看
8. **网络数据源**：支持 TCP/UDP 网络流数据输入，适配远程 IMU 传感器
