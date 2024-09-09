import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from anolink_interfaces.srv import FlightControl  # 假设服务定义用于飞控控制
from anolink_interfaces.srv import SetFlightMode  # 假设服务定义用于模式切换
import threading
import time

class DroneMission(Node):

    def __init__(self):
        super().__init__('drone_mission')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.FC_cmd_publisher = self.create_publisher(String, '/FC_Cmd', 10)
        
        # 服务客户端，用于飞控解锁、上锁和模式切换
        self.flight_control_client = self.create_client(FlightControl, 'flight_control')
        self.set_mode_client = self.create_client(SetFlightMode, 'set_flight_mode')

        # 位置订阅者
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/drone_position',
            self.position_callback,
            10
        )

        self.current_position = None  # 存储当前无人机的位置
        self.mission_active = False
        self.waypoint_list = []  # 存储航点的列表
        self.navigation_thread = None

        self.arming_state = False  # 解锁状态

    def position_callback(self, msg):
        # 位置更新的回调函数
        self.current_position = msg.pose
        self.get_logger().info(f'Current Position: x={self.current_position.position.x}, '
                               f'y={self.current_position.position.y}, '
                               f'z={self.current_position.position.z}')

    def get_current_position(self):
        return self.current_position

    def is_close_enough(self, current_position, waypoint):
        # 检查当前位置是否足够接近目标航点
        tolerance = 0.5  # 距离阈值
        distance = ((current_position.position.x - waypoint['x']) ** 2 +
                    (current_position.position.y - waypoint['y']) ** 2 +
                    (current_position.position.z - waypoint['z']) ** 2) ** 0.5
        return distance < tolerance

    def stop_mission(self):
        # 停止任务和所有活动的线程
        self.mission_active = False
        if self.navigation_thread and self.navigation_thread.is_alive():
            self.navigation_thread.join()

    def navigation_to_waypoint(self, waypoint):
        # 向航点发送导航指令
        self.get_logger().info(f'Navigating to waypoint: {waypoint}')
        nav_msg = Twist()
        nav_msg.linear.x = waypoint['x']
        nav_msg.linear.y = waypoint['y']
        nav_msg.linear.z = waypoint['z']
        self.cmd_vel_publisher.publish(nav_msg)

    def reached_waypoint(self, waypoint):
        # 检查是否到达航点
        self.get_logger().info(f'Checking if reached waypoint: {waypoint}')
        current_position = self.get_current_position()
        if self.is_close_enough(current_position, waypoint):
            self.get_logger().info('Reached waypoint.')
            return True
        else:
            self.get_logger().info('Not yet reached waypoint.')
            return False

    def wait_for_waypoint(self, waypoint):
        # 等待无人机到达航点
        self.get_logger().info(f'Waiting to reach waypoint: {waypoint}')
        while not self.reached_waypoint(waypoint):
            rclpy.spin_once(self, timeout_sec=1)
            time.sleep(0.5)

    def add_waypoint(self, x, y, z, callback=None):
        # 添加一个航点并附带一个可选的回调函数
        waypoint = {'x': x, 'y': y, 'z': z, 'callback': callback}
        self.waypoint_list.append(waypoint)
        self.get_logger().info(f'Added waypoint: {waypoint}')

    def start_waypoint_mission(self):
        # 开始航点任务
        if not self.waypoint_list:
            self.get_logger().info('No waypoints to follow!')
            return
        
        self.mission_active = True
        self.navigation_thread = threading.Thread(target=self._navigate_waypoints)
        self.navigation_thread.start()

    def _navigate_waypoints(self):
        # 依次导航到所有航点
        for waypoint in self.waypoint_list:
            if not self.mission_active:
                break
            self.navigation_to_waypoint(waypoint)
            self.wait_for_waypoint(waypoint)

            # 执行航点任务回调（如果有的话）
            if waypoint.get('callback'):
                self.get_logger().info(f'Executing task at waypoint: {waypoint}')
                waypoint['callback'](waypoint)  # 执行回调函数

        self.get_logger().info('Waypoint mission completed!')
        self.mission_active = False

    # 飞控解锁操作
    def arm_drone(self):
        if self.flight_control_client.wait_for_service(timeout_sec=5.0):
            request = FlightControl.Request()
            request.command = "arm"
            future = self.flight_control_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.arming_state = True
                self.get_logger().info('Drone armed successfully.')
            else:
                self.get_logger().error('Failed to arm drone.')
        else:
            self.get_logger().error('Flight control service not available.')

    # 飞控上锁操作
    def disarm_drone(self):
        if self.flight_control_client.wait_for_service(timeout_sec=5.0):
            request = FlightControl.Request()
            request.command = "disarm"
            future = self.flight_control_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.arming_state = False
                self.get_logger().info('Drone disarmed successfully.')
            else:
                self.get_logger().error('Failed to disarm drone.')
        else:
            self.get_logger().error('Flight control service not available.')

    # 飞行模式切换
    def set_flight_mode(self, mode):
        if self.set_mode_client.wait_for_service(timeout_sec=5.0):
            request = SetFlightMode.Request()
            request.mode = mode
            future = self.set_mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result().success:
                self.get_logger().info(f'Flight mode set to {mode} successfully.')
            else:
                self.get_logger().error(f'Failed to set flight mode to {mode}.')
        else:
            self.get_logger().error('Set flight mode service not available.')

    # 起飞操作
    def takeoff(self, altitude=3.0):
        if not self.arming_state:
            self.get_logger().error('Cannot take off: Drone is not armed.')
            return

        self.get_logger().info(f'Taking off to {altitude} meters...')
        # 在这里发送起飞指令（可以通过cmd_vel或其他服务实现）

    # 降落操作
    def land(self):
        self.get_logger().info('Landing...')
        # 在这里发送降落指令（可以通过cmd_vel或其他服务实现）

def main(args=None):
    rclpy.init(args=args)
    drone_mission = DroneMission()

    # 飞控解锁，起飞到3米，执行航点任务
    drone_mission.arm_drone()
    drone_mission.set_flight_mode("GUIDED")
    drone_mission.takeoff(altitude=3.0)

    # 添加航点并附带任务回调
    drone_mission.add_waypoint(0.0, 1.0, 2.0)
    drone_mission.add_waypoint(1.0, 2.0, 3.0)
    drone_mission.add_waypoint(2.0, 3.0, 1.0)

    # 开始航点任务
    drone_mission.start_waypoint_mission()

    # 完成后降落并上锁
    drone_mission.land()
    drone_mission.disarm_drone()

    rclpy.spin(drone_mission)
    drone_mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
