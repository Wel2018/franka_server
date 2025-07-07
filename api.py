# import cv2
import asyncio
import json
import time
from fastapi import APIRouter
from fastapi import WebSocket
# from fastapi import File, UploadFile
# from fastapi.responses import JSONResponse, StreamingResponse
# from dataclasses import asdict, dataclass, field
from attrs import asdict, define, field
# from loguru import logger
router = APIRouter()
from config import AppConfig


from franky_wrapper import FrankaController
from franky_wrapper import FrankaControllerConfig


def create_reply(data: dict = {}, is_ok=1):
    metadata = dict(
        is_ok=is_ok, # 响应状态
        # inp=inp, # 输入参数
        # timestamp=get_time_str(),
    )
    # 合并
    metadata.update(data)
    return metadata


class Controller:
    arm: FrankaController = None # type: ignore


def create_arm():
    """创建机械臂实例
    - 使用延迟加载，前台根据 arm_id 获取机械臂设备序号
    """
    fk_cfg = FrankaControllerConfig
    print(f"cfg={AppConfig.args}")
    arm_id = AppConfig.args['arm_id']

    if arm_id == "2":
        ip = fk_cfg.ARM_IP2
    elif arm_id == "3":
        ip = fk_cfg.ARM_IP3
    else:
        raise ValueError(f"不支持设备 {AppConfig.args}")

    print(f"arm_id: {arm_id}")
    print(f"arm_ip: {ip}")
    arm = FrankaController(ip)
    Controller.arm = arm


@router.get("/recover", summary="从错误中恢复")
def recover():
    Controller.arm.recover()
    return create_reply()


@router.get("/goto_init_pos", summary="到达初始位置")
def goto_init_pos():
    Controller.arm.goto_init_pos()
    return create_reply()


@router.get("/get_curr", summary="获取当前状态")
def get_curr():
    """获取当前状态
    ```
    data = {
        "pose": self.robot.current_pose.end_effector_pose.matrix.tolist(),
        "ee": state.O_T_EE.matrix.tolist(),
        "joint": state.q.tolist(),
        # "elbow": state.elbow,
    }
    ```
    1. pose: 当前末端执行器的位姿
    2. ee: 当前末端执行器的坐标系
    3. joint: 当前关节角度
    4. elbow: 当前肘部角度
    """
    arm = Controller.arm
    state = arm.get_curr()
    return create_reply(state)


@router.websocket("/get_curr_ws")
async def get_curr_ws(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            arm = Controller.arm
            state = arm.get_curr()
            await websocket.send_text(json.dumps(state))
            # print(time.time(), state)
            await asyncio.sleep(5/1000)  # 每 5ms 发送一次
    except Exception as e:
        print("WebSocket closed:", e)


@router.post("/config", summary="配置参数（速度、加速度、碰撞）")
def config(data: dict):
    """暂未实现"""
    return create_reply()


##################################################
# 速度控制

@router.post("/cartesian_velocity_control", summary="坐标空间速度控制")
def cartesian_velocity_control(data: dict):
    """使用方法：
    ```
    req.post("ctl/cartesian_velocity_control", json={
        "x": 0.1,
        "y": 0.1,
        "z": 0.1,
        "R": 0.1,
        "P": 0.1,
        "Y": 0.1,
        "duration": 1000*60,
        "is_async": 0,
    }, timeout=3)
    ```
    """
    x = data.get("x", 0)
    y = data.get("y", 0)
    z = data.get("z", 0)
    R = data.get("R", 0)
    P = data.get("P", 0)
    Y = data.get("Y", 0)
    duration = data.get("duration", 1000*60)
    is_async = data.get("is_async", 0)

    print(f"{time.time()} cartesian_velocity_control {data}")
    arm = Controller.arm
    arm.cartesian_velocity_control(x,y,z,R,P,Y, duration, is_async)

    return create_reply()


@router.post("/joint_velocity_control", summary="关节空间速度控制")
def joint_velocity_control(data: dict):
    """使用方法：
    ```
    req.post("ctl/joint_velocity_control", json={
        "vel_lst": [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05],
        "duration": 1000*60,
        "is_async": 0,
    }, timeout=3)
    ```
    """
    vel_lst = data.get("vel_lst", [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
    duration = data.get("duration", 1000*60)
    is_async = data.get("is_async", 0)
    arm = Controller.arm
    arm.joint_velocity_control(vel_lst, duration, is_async)

    return create_reply()


@router.get("/stop_cartesian_velocity_control", summary="停止坐标空间速度控制")
def stop_cartesian_velocity_control():
    arm = Controller.arm
    arm.stop_cartesian_velocity_control()
    print(f"{time.time()} stop_cartesian_velocity_control")
    return create_reply()


@router.get("/stop_joint_velocity_control", summary="停止关节空间速度控制")
def stop_joint_velocity_control():
    arm = Controller.arm
    arm.stop_joint_velocity_control()
    return create_reply()


##################################################
# 位置控制

@router.post("/cartesian_position_control", summary="坐标空间路径规划")
def cartesian_position_control(data: dict):
    """使用方法：
    ```
    data = {
        "x": 0.1,
        "y": 0.1,
        "z": 0.1,
        "R": 0.1,
        "P": 0.1,
        "Y": 0.1,
        "mode": "relative",
        "is_async": 0,
    }
    req.post("ctl/cartesian_position_control", json=data, timeout=3)
    """
    x = data.get("x", 0)
    y = data.get("y", 0)
    z = data.get("z", 0)
    R = data.get("R", 0)
    P = data.get("P", 0)
    Y = data.get("Y", 0)
    mode = data.get("mode", "relative")
    is_async = data.get("is_async", 0)
    arm = Controller.arm
    arm.cartesian_position_control(x,y,z,R,P,Y,is_async, mode)
    return create_reply()


@router.post("/joint_position_control", summary="关节空间路径规划")
def joint_position_control(data: dict):
    """使用方法：
    ```
    data = {
        "joints_lst": [-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7],
        "mode": "relative", # absolute
        "is_async": 0,
    }
    req.post("ctl/joint_position_control", json=data, timeout=3)
    ```
    """
    joints_lst = data.get("joints_lst", [-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7])
    mode = data.get("mode", "relative")
    is_async = data.get("is_async", 0)
    arm = Controller.arm
    arm.joint_position_control(joints_lst, is_async, mode)
    return create_reply()


@router.get("/stop_cartesian_position_control", summary="停止坐标空间路径规划")
def stop_cartesian_position_control():
    arm = Controller.arm
    arm.stop_cartesian_position_control()
    return create_reply()


@router.get("/stop_joint_position_control", summary="停止关节空间路径规划")
def stop_joint_position_control():
    arm = Controller.arm
    arm.stop_joint_position_control()
    return create_reply()


##################################################
# 夹爪控制

@router.post("/gripper_control", summary="夹爪控制")
def gripper_control(data: dict):
    """传入字典字段为：
    ```
    force: 50
    speed: 0.1
    width: 0.05
    is_async: 0
    mode: move, grasp, release, wait
    ```
    """
    arm = Controller.arm
    force = data.get("force", 50)
    speed = data.get("speed", 0.1)
    width = data.get("width", 0.05)
    is_async = data.get("is_async", 0)
    mode = data.get("mode", "move")

    arm.gripper_force = force
    arm.gripper_speed = speed

    if mode == "move":
        success = arm.gripper_move(width, is_async)
    elif mode == "grasp":
        success = arm.gripper_grasp(width, is_async)
    elif mode == "release":
        arm.gripper_release(is_async)
        success = 1
    elif mode == "wait":
        arm.gripper_wait()
        success = 1
    else:
        success = 0
    return create_reply(is_ok=1)


@router.post("/join", summary="等待异步动作完成")
def join(data: dict):
    arm = Controller.arm
    timeout = data.get('timeout', 1000)
    is_ok = arm.join(timeout)
    return create_reply(is_ok=is_ok)
