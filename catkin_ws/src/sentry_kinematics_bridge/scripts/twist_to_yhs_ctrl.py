#!/usr/bin/env python
# -*- coding: utf-8 -*-

# FILENAME: ~/catkin_ws/src/sentry_kinematics_bridge/scripts/twist_to_yhs_ctrl.py

# 功能：
# 1. 订阅 /cmd_vel (geometry_msgs/Twist) 话题
# 2. 将线速度(v)和角速度(omega)通过阿克曼运动学模型转换为转向角(steering)
# 3. 发布 /ctrl_cmd (yhs_can_msgs/ctrl_cmd) 话题
# 4. 根据速度处理档位(gear)和刹车(brake)的逻辑状态

import rospy
import math
from geometry_msgs.msg import Twist
from yhs_can_msgs.msg import ctrl_cmd  # 导入自定义消息

# --- 全局变量 ---
wheelbase = 0.6  # (单位: 米) 默认值，将在启动时被ROS参数覆盖
pub = None  # 全局Publisher对象
# -----------------

# 转换线速度和角速度为转向角度（度）
def convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase):
    """ 根据线速度、角速度和轴距计算阿克曼转向角。 """
    if omega == 0 or v == 0:  # 正在直线行驶或停止
        return 0.0
    radius = v / omega
    if radius == 0:
        return 0.0
    # L / R = tan(theta) -> theta = atan(L / R)
    steering_rad = math.atan(wheelbase / radius)
    steering_deg = math.degrees(steering_rad)  # 将弧度转换为度
    return steering_deg

def cmd_callback(data):
    """ /cmd_vel 话题的回调函数，执行所有转换逻辑。 """
    global wheelbase, pub
    
    # 1. 从 /cmd_vel 获取速度和角速度
    v = data.linear.x  # 线速度 (m/s)
    omega = data.angular.z  # 角速度 (rad/s)

    # 2. 运行运动学转换
    # 计算出底盘应有的转向角 (度)
    steering = convert_trans_rot_vel_to_steering_angle(v, omega, wheelbase)

    # --- 创建并填充 yhs_can_msgs/ctrl_cmd 消息 ---
    msg = ctrl_cmd()

    # 3. 字段映射：运动学
    msg.ctrl_cmd_steering = steering  # 将转向角度填入消息

    if v > 0.05:  # 正速度，前进
        msg.ctrl_cmd_gear = 4  # 假设 '4' 是前进档
        msg.ctrl_cmd_Brake = 0  # '0' 是松开刹车
        msg.ctrl_cmd_velocity = v  # 保持原始速度
    elif v < -0.05:  # 负速度，倒车
        msg.ctrl_cmd_gear = 2  # 假设 '2' 是倒档
        msg.ctrl_cmd_Brake = 0  # '0' 是松开刹车
        msg.ctrl_cmd_velocity = -(v)  # 将负速度转换为正值（绝对值）
        msg.ctrl_cmd_steering = -(steering)
    else:  # 速度接近零，停止
        msg.ctrl_cmd_gear = 4  # 假设 '4' 是前进档（或者使用 'N' 档）
        msg.ctrl_cmd_Brake = 1  # 假设 '1' 是激活刹车
        msg.ctrl_cmd_velocity = 0.0  # 强制速度为0

    # 5. 发布转换后的消息
    if pub is not None:
        pub.publish(msg)

# --- 主程序入口 ---
if __name__ == '__main__':
    try:
        rospy.init_node('twist_to_yhs_ctrl_bridge', anonymous=True)

        # 获取话题名称参数
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/cmd_vel')
        ctrl_cmd_topic = rospy.get_param('~ctrl_cmd_topic', '/ctrl_cmd')

        # 获取关键的运动学参数：轴距 (L)
        wheelbase = rospy.get_param('~wheelbase', 0.6)
        if wheelbase == 1.0:
            rospy.logwarn("[%s] 未设置 'wheelbase' 参数，使用默认值 1.0 米。"
                          "这个值很可能是错误的，请在launch文件中设置您机器人的实际轴距!", rospy.get_name())

        # 创建订阅者，监听来自 move_base 的 /cmd_vel
        rospy.Subscriber(twist_cmd_topic, Twist, cmd_callback, queue_size=1)

        # 创建发布者，发布到底层驱动的 /ctrl_cmd
        pub = rospy.Publisher(ctrl_cmd_topic, ctrl_cmd, queue_size=1)

        rospy.loginfo("[%s] 运动学桥接节点已启动。", rospy.get_name())
        rospy.loginfo(" > 订阅话题: %s", twist_cmd_topic)
        rospy.loginfo(" > 发布话题: %s", ctrl_cmd_topic)
        rospy.loginfo(" > 使用轴距 (Wheelbase): %.3f 米", wheelbase)

        rospy.spin()  # 保持节点运行

    except rospy.ROSInterruptException:
        pass
