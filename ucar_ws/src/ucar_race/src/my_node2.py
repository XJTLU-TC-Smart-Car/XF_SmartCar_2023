import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf

class MoveRobot:
    def __init__(self):
        # 初始化节点
        rospy.init_node('move_robot', anonymous=True)

        # 创建发布者，发布/cmd_vel主题，消息类型为Twist
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # 订阅/odom主题，获取小车的当前位置
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # 设置目标点的列表
        self.target_points = [(4.035, 3.847), (3.918, 2.238)]
        self.target_index = 0

        # 设置小车的当前位置和朝向
        self.current_x = 0
        self.current_y = 0
        self.current_theta = 0

    def odom_callback(self, msg):
        # 在这个回调函数中，我们获取小车的当前位置和朝向
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        _, _, self.current_theta = tf.transformations.euler_from_quaternion(quaternion)

    def move_robot(self):
        # 创建Twist消息对象
        vel_msg = Twist()

        # 循环发布速度指令
        rate = rospy.Rate(10)  # 10Hz
        while not rospy.is_shutdown() and self.target_index < len(self.target_points):
            # 获取当前目标点
            target_x, target_y = self.target_points[self.target_index]

            # 计算小车当前位置到目标点的距离
            distance_to_target = math.sqrt((target_x - self.current_x)**2 + (target_y - self.current_y)**2)

            # 如果小车已经到达目标点，我们让小车移动到下一个目标点
            if distance_to_target < 0.1:
                self.target_index += 1
            else:
                # 如果小车还没有到达目标点，我们让小车朝向目标点并前进
                target_theta = math.atan2(target_y - self.current_y, target_x - self.current_x)
                vel_msg.linear.x = 1.2
                vel_msg.angular.z = 0.1 * (target_theta - self.current_theta)

                # 发布速度指令
                self.pub.publish(vel_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        robot = MoveRobot()
        robot.move_robot()
    except rospy.ROSInterruptException:
        pass

