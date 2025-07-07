# franka_server

使用 http 和 ws 协议访问和控制 franka 机械臂

> - 基于 https://github.com/TimSchneider42/franky 实现
> - 在 FR3 平台上测试通过

## 功能特性

- 接口简单易用，参数格式统一
- 支持异步请求
- 客户端无需配置对应环境，仅需使用网络请求即可与机械臂交互
- 支持 http 和 ws 协议，支持高频率（≈ 4000次/秒）请求和推送状态
- 支持区分不同机械臂实例启动（传参时用不同编号启动不同 IP 机械臂）

## 环境配置

- 在虚拟环境中配置 `franky`
- `pip install -r requirements.txt`

## 简单使用

```sh
conda activate franky
python main.py
```

在浏览器中打开 `http://[部署设备 IP]:29000/docs` 查看接口的使用方法

# FAQ

## franky 无法 pip 安装

```
pip install franky-control -i https://pypi.org/simple

ERROR: Could not find a version that satisfies the requirement franky-control (from versions: none)
ERROR: No matching distribution found for franky-control
```

国内源导致，推荐使用源码编译安装。

## franky 编译安装时报错

```sh
# 推荐将 libfranka 的链接路径写到 .bashrc 中
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/franka/code/libfranka/build

# 开始构建
# 需要先下载 eigen-3.4 到本地构建安装
mkdir -p build && cd build
cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH="/usr/local" \
    -DCMAKE_PREFIX_PATH=~/local/eigen-3.4 \
    -DFranka_DIR:PATH=/home/franka/code/libfranka/build \
    -DPYBIND11_INCLUDE_DIR=$(python -m pybind11 --includes | sed -e 's/-I//g') \
    -DBUILD_TESTS=off \
    ..

# 安装
make
sudo make install
cd ..

## 安装 python 版本
# pip install -e .
~/miniconda3/envs/franky/bin/pip install -e .
```

### `setup.py` 修改方法

```python
...

sys.path.append("/home/franka/miniconda3/envs/franky/lib/python3.10/site-packages")

import pybind11
from setuptools import setup, Extension, find_packages
from setuptools.command.build_ext import build_ext
pybind11_cmake_dir = pybind11.get_cmake_dir()
pybind11_include = pybind11.get_include()
print(f"pybind11_cmake_dir={pybind11_cmake_dir}")


class CMakeBuild(build_ext):
...

    def build_extension(self, ext):
        ext_dir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()

        build_type = os.environ.get("BUILD_TYPE", "Release")
        build_args = ["--config", build_type]

        # 执行
        # export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/franka/code/libfranka/build
        cmake_args = [
            f"-DPYBIND11_DIR={pybind11_cmake_dir}",
            f"-DPYBIND11_INCLUDE_DIR={pybind11_include}",
            f"-DCMAKE_PREFIX_PATH={pybind11_cmake_dir};/usr/local;{os.path.expanduser('~/local/eigen-3.4')}",
            "-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={}/".format(ext_dir),
            "-DPYTHON_EXECUTABLE={}".format(sys.executable),
            "-DCMAKE_BUILD_TYPE={}".format(build_type),
            "-DBUILD_EXAMPLES=OFF",
            "-DBUILD_PYTHON_STUBS=ON",
            "-DBUILD_TESTS=OFF",
            "-DBUILD_SHARED_LIBS=OFF",
            "-DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE",
            "-DCMAKE_INSTALL_RPATH=$ORIGIN",
            "-DCMAKE_POSITION_INDEPENDENT_CODE=ON",
            "-DFranka_DIR:PATH=/home/franka/code/libfranka/build",
            # "-Dpybind11_DIR=`pybind11-config --cmakedir`",
            # "-DCMAKE_PREFIX_PATH=/usr/local",
            # "-DCMAKE_PREFIX_PATH=~/local/eigen-3.4",
            # f"-DPYBIND11_INCLUDE_DIR={pybind11.get_include()}",
        ]

        Path(self.build_temp).mkdir(exist_ok=True, parents=True)

        subprocess.check_call(["cmake", str(Path(".").resolve())] + cmake_args, cwd=self.build_temp)
        subprocess.check_call(
            ["cmake", "--build", ".", "--target", "_franky"] + build_args + 
            ["--", "-j", str(multiprocessing.cpu_count())], cwd=self.build_temp)
...
```

## Cannot set translational velocity limit while robot is in control

```python
robot.translation_velocity_limit.set(3.0)
robot.rotation_velocity_limit.set(2.5)
```

在配置机械臂时提示无法在控制状态下修改速度，
已反馈该问题：https://github.com/TimSchneider42/franky/issues/36

修复方法：

```c
// (line 16) 添加一个 "!"
#define LIMIT_INIT(name, value_panda, value_fer) \
  name, SEL_VAL(value_panda, value_fer), control_mutex_, [this] { return !is_in_control_unsafe(); }
```
