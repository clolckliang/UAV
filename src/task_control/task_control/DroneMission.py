import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Bool
import threading
from geometry_msgs.msg import PoseStamped

class DroneMission(Node):

    def __init__(self):
        super().__init__('drone_mission')
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.FC_cmd_publisher = self.create_publisher(String,'/FC_Cmd',10)
        # 添加位置订阅者
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/drone_position',
            self.position_callback,
            10
        )

        self.detection_subscriber =self.create_subscription(
            

        )

        self.current_position = None  # 用于存储最新的位置
        self.mission_active = False
        self.keep_height_thread = None
        self.navigation_thread = None

    def detection_callback(self,msg):
        pass

    def position_callback(self, msg):
        # 回调函数处理接收到的位置消息
        self.current_position = msg.pose
        self.get_logger().info(f'Current Position: x={self.current_position.position.x}, '
                               f'y={self.current_position.position.y}, '
                               f'z={self.current_position.position.z}')

    def get_current_position(self):
        # 返回当前无人机的位置
        return self.current_position

    def is_close_enough(self, current_position, waypoint):
        # 检查当前位置是否接近目标路径点
        tolerance = 0.5  # 定义接近目标点的距离阈值
        distance = ((current_position.position.x - waypoint['x']) ** 2 +
                    (current_position.position.y - waypoint['y']) ** 2 +
                    (current_position.position.z - waypoint['z']) ** 2) ** 0.5
        return distance < tolerance

    def stop(self):
        self.mission_active = False
        if self.keep_height_thread and self.keep_height_thread.is_alive():
            self.keep_height_thread.join()
        if self.navigation_thread and self.navigation_thread.is_alive():
            self.navigation_thread.join()

        stop_msg = Twist()
        self.cmd_vel_publisher.publish(stop_msg)

        self.get_logger().info('Mission stopped and control deactivated.')

    def run(self):
        self.mission_active = True
        self.get_logger().info('Mission started.')
        
        self.pointing_takeoff()
        self.keep_height_task()
        self.navigation_task()
        self.pointing_landing()

        self.get_logger().info('Mission completed.')

    def pointing_takeoff(self):
        self.get_logger().info('Executing pointing takeoff...')
        takeoff_msg = Twist()
        takeoff_msg.linear.z = 1.0
        self.cmd_vel_publisher.publish(takeoff_msg)
        rclpy.spin_once(self, timeout_sec=5)
        takeoff_msg.linear.z = 0.0
        self.cmd_vel_publisher.publish(takeoff_msg)
        self.get_logger().info('Takeoff completed.')

    def pointing_landing(self):
        self.get_logger().info('Executing pointing landing...')
        landing_msg = Twist()
        landing_msg.linear.z = -1.0
        self.cmd_vel_publisher.publish(landing_msg)
        rclpy.spin_once(self, timeout_sec=5)
        landing_msg.linear.z = 0.0
        self.cmd_vel_publisher.publish(landing_msg)
        self.get_logger().info('Landing completed.')

    def switch_pid(self, new_pid_params):
        self.get_logger().info('Switching PID parameters...')
        self.get_logger().info('PID parameters switched.')

    def keep_height_task(self):
        self.get_logger().info('Starting keep height task...')
        if self.keep_height_thread is None or not self.keep_height_thread.is_alive():
            self.keep_height_thread = threading.Thread(target=self._keep_height)
            self.keep_height_thread.start()

    def _keep_height(self):
        while self.mission_active:
            self.get_logger().info('Maintaining height...')
            rclpy.spin_once(self, timeout_sec=1)

    def navigation_task(self):
        self.get_logger().info('Starting navigation task...')
        if self.navigation_thread is None or not self.navigation_thread.is_alive():
            self.navigation_thread = threading.Thread(target=self._navigate)
            self.navigation_thread.start()

    def _navigate(self):
        while self.mission_active:
            self.get_logger().info('Navigating to waypoint...')
            rclpy.spin_once(self, timeout_sec=1)

    def navigation_to_waypoint(self, waypoint):
        self.get_logger().info(f'Navigating to waypoint: {waypoint}')
        nav_msg = Twist()
        nav_msg.linear.x = waypoint['x']
        nav_msg.linear.y = waypoint['y']
        nav_msg.linear.z = waypoint['z']
        self.cmd_vel_publisher.publish(nav_msg)

    def set_navigation_speed(self, speed):
        self.get_logger().info(f'Setting navigation speed to: {speed}')
        nav_speed_msg = Twist()
        nav_speed_msg.linear.x = speed
        self.cmd_vel_publisher.publish(nav_speed_msg)

    def reached_waypoint(self, waypoint):
        self.get_logger().info(f'Checking if reached waypoint: {waypoint}')
        current_position = self.get_current_position()
        if self.is_close_enough(current_position, waypoint):
            self.get_logger().info('Reached waypoint.')
            return True
        else:
            self.get_logger().info('Not yet reached waypoint.')
            return False

    def wait_for_waypoint(self, waypoint):
        self.get_logger().info(f'Waiting to reach waypoint: {waypoint}')
        while not self.reached_waypoint(waypoint):
            rclpy.spin_once(self, timeout_sec=1)

    def wait_for_waypoint_with_avoidance(self, waypoint):
        self.get_logger().info(f'Waiting to reach waypoint with avoidance: {waypoint}')
        while not self.reached_waypoint(waypoint):
            self.avoid_obstacles()
            rclpy.spin_once(self, timeout_sec=1)

def main(args=None):
    rclpy.init(args=args)
    drone_mission = DroneMission()
    rclpy.spin(drone_mission)
    drone_mission.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
