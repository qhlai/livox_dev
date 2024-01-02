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


# 打开 rosbag 文件
bag = rosbag.Bag(bag_file, 'r')

# 获取 rosbag 文件中的所有话题
topics = bag.get_type_and_topic_info().topics
# 打印话题列表
print("Topics in the bag file:")
for topic in topics:
    print(topic)

txt_file_0 = "arm_pose.txt"
txt_file_1="odom.txt"
txt_file_2="arm_cam_sync.txt"

file0 = open(txt_file_0, 'r+')
file1 = open(txt_file_1, 'r+')
file2 = open(txt_file_2, 'r+')

msg_pose = None
msg_odom = None


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')


pose_v=[]
odom_v=[]
timestamp=None
line_count = 0
for line in file0.readlines():
    line_count += 1
    if line_count % 5 == 0:
        # 拆分每一行的内容
        items = line.strip().split(",")
        # 保存拆分后的位姿 XYZ 和 WXYZ
        # print(items[0])
        pose_v.append([float(items[0]), float(items[1]), float(items[2]),float(items[3]), float(items[4]), float(items[5]), float(items[6])])
line_count = 0
for line in file1.readlines():
    line_count += 1
    if line_count % 5 == 0:
        # 拆分每一行的内容
        items = line.strip().split(",")
        # 保存拆分后的位姿 XYZ 和 WXYZ
        odom_v.append([float(items[0]), float(items[1]), float(items[2]),float(items[3]), float(items[4]), float(items[5]), float(items[6])])


count_pose = 0
count_odom = 0

# print(pose_v)
for item in pose_v:
    # pass
    if count_pose%30==0:
        ax.scatter(item[1],item[2],item[3], c="g")


for item in odom_v:
    # pass
    if count_pose%9==0:
        ax.scatter(item[1],item[2],item[3], c="r")

# for topic, msg, t in bag.read_messages():
#     pass
    
#     allow_t_diff=1
#     if topic == topic_name_armpose:
#         msg_pose = msg
#         count_pose+=1
#         if count_pose%10==0:
#             ax.scatter(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, c="g")
#     elif topic == topic_name_odom:
#         msg_odom = msg
#         count_odom+=1
#         if count_odom%3==0:
#             ax.scatter(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z, c="r")
#     # print("1")
#     if(msg_pose != None and msg_odom != None):
#         # print("1")
#         duration = msg_pose.header.stamp.to_sec() - msg_odom.header.stamp.to_sec()
#         print(duration, msg_pose.header.seq, msg_odom.header.seq, {msg_pose.pose.position.x}, {msg_pose.pose.position.y}, {msg_pose.pose.position.z})
#         if( duration < allow_t_diff):
#             print("2")
#             pass

file0.close()
file1.close()
file2.close()

# 关闭 rosbag 文件
bag.close()


ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()