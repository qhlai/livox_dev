import rosbag
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

# 指定保存的 rosbag 文件路径
bag_file = "/home/lqh/ros/dataset/2023-12-13-1/2023-12-13-17-25-39.bag"
# 指定要保存的话题名称
topic_name_armpose = "/arm_pose"
topic_name_odom = "/rtabmap/odom"
# 指定保存的 txt 文件路径
txt_file_0 = "arm_pose.txt"
txt_file_1="odom.txt"
txt_file_2="arm_cam_sync.txt"

# 打开 rosbag 文件
bag = rosbag.Bag(bag_file, 'r')

# 获取 rosbag 文件中的所有话题
topics = bag.get_type_and_topic_info().topics
# 打印话题列表
print("Topics in the bag file:")
for topic in topics:
    print(topic)

file0 = open(txt_file_0, 'w')
file1 = open(txt_file_1, 'w')
file2 = open(txt_file_2, 'w')

msg_pose = None
msg_odom = None

for topic, msg, t in bag.read_messages():
    pass

    allow_t_diff=1
    if topic == topic_name_armpose:
        msg_pose = msg
        file0.write(f"{msg.header.stamp.to_sec()},{msg.pose.position.x},{msg.pose.position.y},{msg.pose.position.z},{msg.pose.orientation.w},{msg.pose.orientation.x},{msg.pose.orientation.y},{msg.pose.orientation.z}")
        file0.write("\n")
        # ax.scatter(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, c="g")
    elif topic == topic_name_odom:
        msg_odom = msg
        file1.write(f"{msg.header.stamp.to_sec()}, {msg.pose.pose.position.x},{msg.pose.pose.position.y},{msg.pose.pose.position.z}, {msg.pose.pose.orientation.w}, {msg.pose.pose.orientation.x}, {msg.pose.pose.orientation.y}, {msg.pose.pose.orientation.z}")
        file1.write("\n")
        # ax.scatter(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, c="r")
    # print("1")
    if(msg_pose != None and msg_odom != None):
        # print("1")
        duration = msg_pose.header.stamp.to_sec() - msg_odom.header.stamp.to_sec()
        print(duration, msg_pose.header.seq, msg_odom.header.seq, {msg_pose.pose.position.x}, {msg_pose.pose.position.y}, {msg_pose.pose.position.z})
        if( duration < allow_t_diff):
            print("2")
            pass

file0.close()
file1.close()
file2.close()

# 关闭 rosbag 文件
bag.close()
