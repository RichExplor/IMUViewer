# IMUViewer
**用于可视化和调试IMU实时数据和离线数据的上位机。**

## 1. 运行环境
* Python=3.13
* numpy
* PyQt5
* pyqtgraph
* pyserial


### 1.1 创建conda虚拟环境
```python
conda create -y -n imu_env python=3.13
```

### 1.2 安装第三方库
```python
conda activate imu_env
pip install -r requirements.txt
```

## 2. 运行方式
- [**Ubuntu**] `python main.py`
- [**Windows**] `点击exe可执行文件`


## 3. 跨平台打包exe可执行文件
**基于Github Actions的CI/CD流水线自动打包生成Windows可执行文件。**

### 3.1 配置CI/CD yml文件
```python
mkdir -p .github/workflows
cd .github/workflows
touch build.yml

## build.yml中详细注明流水线步骤以及依赖库，可参考本方法
```

### 3.2 推送远端
```python
# 保存修改并提交推送当前commit
git add .
git commit -m "fix: xxxx"
git push origin main

# 可配置打包标签
git tag v1.0.0
git push origin v1.0.0
```

### 3.3 构建CI/CD流水线
```python
# 1. 进入远端github仓库，点击Actions
# 2. 点击左侧Build Windows Executable
# 3. 点击右下角按钮run workflow，选择带构建分支
```

### 3.4 下载exe可执行文件
```python
# 1. 点击进入当前已构建的流水线
# 2. 找到Artifacts下的产物下载
```