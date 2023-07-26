import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan

# 这是你的目标点列表
target_points = [
    {'x': 4.991, 'y': -0.230, 'z': 0.99999077, 'w': 0.00429631},
    {'x': 0.068, 'y': 4.075, 'z': 0.691, 'w': 0.723},
    {'x': 0.049, 'y': 0.855, 'z': -0.791, 'w': 0.611},
    {'x': 1.833, 'y': 4.260, 'z': 0.663, 'w': 0.749},
    {'x': 1.581, 'y': -0.330, 'z': 1.000, 'w': -0.005},
    {'x': 3.944, 'y': 0.113, 'z': 0.708, 'w': 0.707},
    {'x': 3.943, 'y': 1.950, 'z': 0.711, 'w': 0.703},
    {'x': 3.918, 'y': 2.238, 'z': -0.701, 'w': 0.713},
    {'x': 3.943, 'y': 1.950, 'z': -0.711, 'w': 0.703},
    {'x': 3.943, 'y': 0.113, 'z': -0.708, 'w':0.707}
    {'x': 4.991, 'y': -0.230, 'z': -0.010, 'w': 1.000} 
    # 添加更多的点...
]

def pose_callback(pose):
    tolerance = 0.1

    # 检查小车是否接近任何一个目标点
    for point in target_points:
        if abs(pose.pose.position.x - point['x']) < tolerance and abs(pose.pose.position.y - point['y']) < tolerance:
            # 如果小车接近一个目标点，发布一个消息将雷达的范围设置为0
            scan = LaserScan()
            scan.range_min = 0
            scan.range_max = 0
            scan_pub.publish(scan)
            break  # 如果已经找到一个接近的点，就不需要再检查其他的点

rospy.init_node('radar_control')
pose_sub = rospy.Subscriber('/pose', PoseStamped, pose_callback)
scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
rospy.spin()

