import franky
import numpy as np
import time
from argparse import ArgumentParser
from franky._franky import RelativeDynamicsFactor
from franky import Robot, JointVelocityMotion, CartesianVelocityMotion, Duration, JointMotion, Twist
from franky import * # type: ignore


class FrankaControllerConfig:
    ARM_IP2 = "172.16.0.2"
    ARM_IP3 = "172.16.0.3"

    home = [0.0, -0.2, 0.0, -1.5, 0.0, 1.5, 0.0]
    # Left side reach
    left = [np.pi / 3, -0.5, 0.3, -2.0, 0.2, 2.0, np.pi / 6]
    # Right side reach
    right = [-np.pi / 3, -0.3, -0.3, -1.8, -0.2, 1.8, -np.pi / 6]
    # Forward extended position
    forward = [0.0, -0.1, 0.0, -2.5, 0.0, 2.8, 0.0]


class FrankaController:
    """基于 Franky 实现的机械臂控制器，
    支持坐标空间和关节空间的路径规划和实时透传
    """

    def __init__(self, ip="127.0.0.1") -> None:
        self.ip = ip
        self.robot = Robot(ip)
        self.robot.recover_from_errors()
        
        # 减小加速度 Reduce the acceleration and velocity dynamic
        self.robot.relative_dynamics_factor = RelativeDynamicsFactor(0.1, 0.05, 0.05)

        # 默认单位，m，ms
        self.max_time = 60*1000 # 60s
        self.goto_init_pos()

        # 夹爪控制
        self.gripper = franky.Gripper(ip)
        self.gripper_success_future = None
        self.gripper_speed = 0.1
        self.gripper_force = 50
        self.gripper_epsilon_outer = 1.0
        # self.gripper_is_open = True
        self.gripper_release()

        self.stop_async = bool(1)
        self.gripper_async = bool(0)

        # Franka Emika Panda 机械臂提供了 setCollisionBehavior 接口，
        # 用于设置关节和笛卡尔空间上的碰撞阈值，以实现碰撞检测与响应。
        # {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        # {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        # {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        # {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},

        # {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
        # {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        # {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
        # {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

        # torque_thresholds: 作用在每个关节上的力矩阈值，共7个。
        # force_thresholds: 作用在末端执行器的笛卡尔力/力矩阈值，共6个（力3+力矩3）
        # acceleration 表示运动加速过程的阈值
        # nominal 表示正常运行过程的阈值
        # 一旦超过上限，会触发碰撞检测，系统会进入 reflex mode
        # “加速/减速过程的接触力阈值”是给运动过程中“自己造成的惯性力”的容忍范围，
        # 而“正常运行过程的碰撞力阈值”是对“外界物体实际碰撞”的响应阈值。
        FORCE_LOWER = 50.0
        FORCE_UPPER = 80.0
        ACC_FORCE_LOWER = 60.0
        ACC_FORCE_UPPER = 60.0
        
        TORQUE_LOWER = 20.0
        TORQUE_UPPER = 50.0
        ACC_TORQUE_LOWER = 40.0
        ACC_TORQUE_UPPER = 60.0

        self.robot.set_collision_behavior(
            # Torque thresholds for 7 joints:
            [ACC_TORQUE_LOWER]*7,
            [ACC_TORQUE_UPPER]*7,
            [TORQUE_LOWER]*7,
            [TORQUE_UPPER]*7,
            
            # 【加速/减速过程中】在 N 中 (x,y,z,R,P,Y) 的的接触力阈值
            [ACC_FORCE_LOWER]*6,
            # ... 碰撞力阈值
            [ACC_FORCE_UPPER]*6,
            # 【正常运行过程】在 N 中 (x,y,z,R,P,Y) 的接触力阈值
            [FORCE_LOWER]*6,
            # 碰撞力阈值
            [FORCE_UPPER]*6,
        )


    def gripper_move(self, width=0.05, is_async=0):
        """控制夹爪移动到指定宽度，默认 5cm，速度为 0.1 [m/s]"""
        speed = self.gripper_speed
        if not is_async:
            success = self.gripper.move(width, speed)
            return success
        else:    
            self.gripper_success_future = self.gripper.move_async(width, speed)
            return True
    
    def gripper_wait(self):
        if self.gripper_success_future is None:
            return
        
        success_future = self.gripper_success_future
        # Wait for 1s
        if success_future.wait(1):
            print(f"Success: {success_future.get()}")
        else:
            self.gripper.stop()
            success_future.wait()
            print("Gripper motion timed out.")

    def gripper_grasp(self, width=0.0):
        """带力控抓取一个未知大小的物体，力度默认为 20 [N]"""
        speed = self.gripper_speed
        force = self.gripper_force
        epsilon_outer = self.gripper_epsilon_outer
        # success = self.gripper.move(1.00, speed)
        # if self.gripper_async:
        success = self.gripper.grasp_async(width, speed, force, epsilon_outer=epsilon_outer)
        return success

    def gripper_release(self):
        """抓取物品后释放出来，返回物体的宽度"""
        # Get the width of the grasped object
        width = self.gripper.width
        # Release the object
        self.gripper.open_async(self.gripper_speed)
        return width

    def goto_init_pos(self):
        """到达初始位置 Go to initial position"""
        # bug: 松开 G 按钮时会导致调用 stop_action，该过程中可能还没有到达初始位置
        print("等待之前的动作执行完毕")
        # franky._franky.InvalidMotionTypeException: 
        # The type of motion cannot change during runtime. 
        # Please ensure that the previous motion finished before using a new type of motion.
        self.robot.join_motion(1000.0)
        print("回归零位")
        self.robot.move(
            JointMotion([0.0, 0.0, 0.0, -2.2, 0.0, 2.2, 0.7]), 
            asynchronous=bool(0))

    def config(self):
        """配置"""
        robot = self.robot
        # Set velocity, acceleration and jerk to 5% of the maximum
        robot.relative_dynamics_factor = 0.05

        # Alternatively, you can define each constraint individually
        robot.relative_dynamics_factor = RelativeDynamicsFactor(velocity=0.1, acceleration=0.05, jerk=0.1)

        # Or, for more finegrained access, set individual limits
        robot.translation_velocity_limit.set(3.0)
        robot.rotation_velocity_limit.set(2.5)
        robot.elbow_velocity_limit.set(2.62)
        robot.translation_acceleration_limit.set(9.0)
        robot.rotation_acceleration_limit.set(17.0)
        robot.elbow_acceleration_limit.set(10.0)
        robot.translation_jerk_limit.set(4500.0)
        robot.rotation_jerk_limit.set(8500.0)
        robot.elbow_jerk_limit.set(5000.0)
        robot.joint_velocity_limit.set([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
        robot.joint_acceleration_limit.set([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0])
        robot.joint_jerk_limit.set([5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0, 5000.0])
        # By default, these limits are set to their respective maxima (the values shown here)

        # Get the max of each limit (as provided by Franka) with the max function, e.g.:
        print(robot.joint_jerk_limit.max)

    def get_curr(self):
        """获取当前状态"""
        state = self.robot.state
        data = {
            # "pose": self.robot.current_pose.end_effector_pose.matrix.tolist(),
            "ee": state.O_T_EE.matrix.tolist(),
            "joint": state.q.tolist(),
            # "elbow": state.elbow,
        }
        # print(f"get_curr: {data}")
        return data

    # 速度控制 -----------------------------------------
    def cartesian_velocity_control(self, x=0, y=0, z=0, R=0, P=0, Y=0, duration=1000*60, is_async=0):
        """坐标空间速度控制"""
        motion = CartesianVelocityMotion(
            Twist(
                linear_velocity=[x, y, z], # type: ignore
                angular_velocity=[R, P, Y]), # type: ignore
            duration=Duration(duration))
        
        def reaction_callback(robot_state: RobotState, rel_time: float, abs_time: float):
            print(f"Reaction fired at {abs_time}.")
        
        # It is important that the reaction motion uses the same control mode as the original motion. Hence, we cannot register
        # a JointMotion as a reaction motion to a CartesianMotion.
        #reaction_motion = CartesianVelocityReaction(Affine([0.0, 0.0, 0.01]), ReferenceType.Relative)
        reaction_motion = CartesianVelocityMotion(
            Twist(
                linear_velocity=[0, 0, 0.05], # type: ignore
                angular_velocity=[R, P, Y]), # type: ignore
            duration=Duration(2000))
        # Trigger reaction if the Z force is greater than 30N
        reaction = CartesianVelocityReaction(Measure.FORCE_Z < -30.0, reaction_motion)
        reaction.register_callback(reaction_callback)
        motion.add_reaction(reaction)
        self.robot.move(motion, asynchronous=bool(is_async))

    def joint_velocity_control(self, vel_lst: list, duration=1000*60, is_async=0):
        """关节空间速度控制"""
        motion = JointVelocityMotion(vel_lst, duration=Duration(duration)) # type: ignore
        self.robot.move(motion, asynchronous=bool(is_async))

    def stop_cartesian_velocity_control(self):
        """停止坐标空间速度控制"""
        # m_stop = franky.CartesianVelocityStopMotion()
        # self.robot.move(m_stop)
        self.robot.move(CartesianVelocityMotion(
            Twist(
                linear_velocity=[0.0, 0.0, 0.0], # type: ignore
                angular_velocity=[0.0, 0.0, 0.0]), # type: ignore
                # duration=Duration(1)
            ),
            self.stop_async
        )
    
    def stop_joint_velocity_control(self):
        """停止关节空间速度控制"""
        # m_stop = franky.JointVelocityStopMotion()
        # self.robot.move(m_stop)
        self.robot.move(JointVelocityMotion(
            Twist(
                linear_velocity=[0.0, 0.0, 0.0], # type: ignore
                angular_velocity=[0.0, 0.0, 0.0]), # type: ignore
            duration=Duration(1)), self.stop_async
        )

    # 位置控制 -----------------------------------------
    def cartesian_position_control(self, x=0, y=0, z=0, R=0, P=0, Y=0, is_async=0, mode="relative"):
        """坐标空间路径规划"""
        from scipy.spatial.transform import Rotation
        # Geometry
        quat = Rotation.from_euler("xyz", [R, P, Y]).as_quat()
        target = Affine([x, y, z], quat) # type: ignore
        reference_type = ReferenceType.Relative if mode == "relative" else ReferenceType.Absolute        
        motion = CartesianMotion(target, reference_type=reference_type)
        self.robot.move(motion, bool(is_async))

    def joint_position_control(self, joints_lst: list, is_async=0, mode="relative"):
        """关节空间路径规划"""
        # A point-to-point motion in the joint space
        reference_type = ReferenceType.Relative if mode == "relative" else ReferenceType.Absolute
        motion = JointMotion(joints_lst, reference_type=reference_type)
        # Trigger reaction if the Z force is greater than 30N
        reaction = JointPositionReaction(Measure.FORCE_Z < -10.0, JointStopMotion())
        motion.add_reaction(reaction)
        self.robot.move(motion, bool(is_async))

    def stop_cartesian_position_control(self):
        """停止坐标空间路径规划"""
        m_stop = franky.CartesianStopMotion()
        self.robot.move(m_stop, self.stop_async)

    def stop_joint_position_control(self):
        """停止关节空间路径规划"""
        m_stop = franky.JointStopMotion()
        self.robot.move(m_stop, self.stop_async)

    def __del__(self):
        print("释放机械臂")
        self.robot.stop()


if __name__ == "__main__":
    fc = FrankaController(FrankaControllerConfig.ARM_IP2)
    fc.gripper_speed = 0.1
    fc.gripper_grasp(0.0)
    res = fc.gripper_release()
    # res = fc.gripper_move(1.0, speed=0.2)
    # res = fc.gripper_move(0.0, speed=0.2)
    # res = fc.gripper_move(1.0, speed=0.2)
    print(res)

