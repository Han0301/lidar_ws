#!/usr/bin/env python3
"""Activate planning only after the playback clock and navigation TF are ready."""

import signal
import time

import rclpy
from nav2_msgs.srv import ManageLifecycleNodes
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


# 用有效时钟、近期位姿和同一时间戳 TF 保护地图启动，避免录包时钟跳变导致超时
class NavigationStartupGate(Node):
    # 读取门控条件并建立 TF、位姿订阅、生命周期服务和墙钟检查定时器
    def __init__(self):
        super().__init__('nav2_startup_gate')
        self.require_map = self.declare_parameter('require_map', False).value
        self.map_ready = False
        self.map_subscription = self.create_subscription(OccupancyGrid, '/map', self.receive_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timeout = self.declare_parameter('input_timeout_s', 0.6).value      # 位姿到达和时间戳的新鲜度上限（s）
        self.global_frame = self.declare_parameter('global_frame', 'odom').value      # 导航地图所用的全局参考坐标
        self.base_frame = self.declare_parameter('base_frame', 'nav_base').value      # 地图需要跟踪的虚拟平面参考坐标
        topic = self.declare_parameter('odom_topic', '/nav/odom').value      # 桥接节点发布的导航位姿话题
        service = self.declare_parameter(
            'manager_service', '/lifecycle_manager_planner/manage_nodes').value      # 生命周期管理服务名称
        # 使用当前节点的 ROS 时钟缓存 TF，确保查询与位姿时间一致
        self.tf_buffer = Buffer(node=self)      # 缓存同一 ROS 时钟下的坐标变换
        self.tf_listener = TransformListener(self.tf_buffer, self)      # 监听动态和静态 TF，回调由当前执行器处理
        self.odometry = None      # 最近收到的导航位姿；None 表示尚未就绪
        self.received_at = 0.0      # 位姿最近一次到达的单调墙钟时间（s）
        self.last_report = 0.0      # 最近一次等待日志的墙钟时间（s），用于五秒节流
        self.request = None      # 生命周期启动请求的异步 Future，发出后不重复提交
        self.finished = False      # 是否已收到本次启动请求结果；失败也停止自动重复启动
        self.client = self.create_client(ManageLifecycleNodes, service)      # 调用规划生命周期管理器的客户端
        self.subscription = self.create_subscription(Odometry, topic, self.receive, 10)      # 可靠订阅桥接位姿，最多缓存十条消息
        # 使用单调墙钟检查，第一条 /clock 到达前也能运行，周期为 0.1 s
        self.timer = self.create_timer(0.1, self.check,
                                      clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))

    def receive_map(self, message):
        self.map_ready = (message.header.frame_id == self.global_frame and
                          message.info.width > 0 and message.info.height > 0 and
                          len(message.data) == message.info.width * message.info.height)

    # 保存最新位姿和墙钟到达时间，供启动条件检查
    def receive(self, message):
        self.odometry = message
        self.received_at = time.monotonic()

    # 每五秒最多报告一次未就绪原因，避免终端被等待信息刷屏
    def waiting(self, reason):
        current = time.monotonic()      # 单调墙钟，与录包时间跳变和暂停无关
        if current - self.last_report >= 5.0:
            self.get_logger().info('Waiting for navigation startup: ' + reason)
            self.last_report = current

    # 顺序检查启动条件；全部满足后只提交一次异步 STARTUP 请求
    def check(self):
        # 已经处理过启动结果时停止门控动作，持续运行仅等待进程退出
        if self.finished:
            return
        # 先处理在途请求，不在管理器配置节点期间重复提交
        if self.request is not None:
            if not self.request.done():
                return
            self.finished = True
            try:
                result = self.request.result()      # 生命周期管理器返回的成功标志
                if result is not None and result.success:
                    self.get_logger().info('Planner and costmap activated after clock, odometry and TF became ready')
                else:
                    self.get_logger().error('Planner activation failed; inspect lifecycle and costmap logs')
            except Exception as error:
                self.get_logger().error('Planner activation request failed: ' + str(error))
            return
        # 等待第一条仿真时钟，避免地图以零时间开始等待 TF
        if self.get_clock().now().nanoseconds == 0:
            self.waiting('no simulation clock')
            return
        # 同时检查墙钟到达年龄和 ROS 时间戳，排除暂停或跨通信域造成的旧输入
        message = self.odometry      # 本次检查使用的最新位姿快照
        if message is None or time.monotonic() - self.received_at > self.timeout:
            self.waiting('no recent navigation odometry')
            return
        # 确认位姿表达的是地图所需的 global_frame→base_frame
        if message.header.frame_id != self.global_frame or message.child_frame_id != self.base_frame:
            self.waiting('navigation odometry frame mismatch')
            return
        stamp = Time.from_msg(message.header.stamp)      # 位姿扫描时间，按 ROS 时间解释
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9      # 当前 ROS 时间与位姿时间的差（s），检查回放时钟是否匹配
        if abs(age) > self.timeout:
            self.waiting('odometry timestamp does not match the current clock')
            return
        # TF 必须覆盖同一扫描时刻，不能用任意最新变换代替
        if not self.tf_buffer.can_transform(self.global_frame, self.base_frame, stamp):
            self.waiting('navigation TF not available at the odometry timestamp')
            return
        if self.require_map and not self.map_ready:
            self.waiting('no complete session map in the configured frame')
            return
        # 管理器服务出现后才发请求，避免启动阶段服务尚未创建
        if not self.client.service_is_ready():
            self.waiting('lifecycle manager service unavailable')
            return
        # 有效时钟、位姿、TF 和服务都就绪，开始配置并激活规划服务器
        request = ManageLifecycleNodes.Request()      # 请求管理器配置并激活其管理的规划节点
        request.command = request.STARTUP      # STARTUP 包含生命周期配置和激活，不直接控制底盘
        self.request = self.client.call_async(request)
        self.get_logger().info('Navigation inputs ready; requesting planner startup')


# 初始化 ROS 节点并运行事件循环，处理退出和资源清理
def main():
    rclpy.init()
    node = NavigationStartupGate()      # 等待数据就绪后发出一次启动请求的 ROS 节点
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # 进程组已收到 SIGINT 后 launch 可能再次转发；清理阶段忽略重复中断
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# 作为可执行 ROS 节点启动时进入主函数
if __name__ == '__main__':
    main()
