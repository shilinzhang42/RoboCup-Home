import actionlib
import rospy
import moveit_commander
import tf2_ros
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import math
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import euler_from_quaternion

from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import (Constraints, 
                             PositionConstraint, 
                             BoundingVolume, 
                             PlanningScene,
                             CollisionObject,
                             AttachedCollisionObject)


class NavigationToPerson():
    def __init__(self, safe_distance=0.5, target_object="person"):
        """
        :param safe_distance: 机器人与人员保持的安全距离 (单位: 米)
        :param target_object: 在 /text_markers 中要寻找的目标对象文字
        """
        self.safe_distance = safe_distance
        self.goal_object = target_object

        # 用于等待/获取 MarkerArray
        self.marker_topic = "/text_markers"
        
        # TF2 缓存
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.nav_goal_pub = rospy.Publisher('/nav_goal', PoseStamped, queue_size=10)
        self.person_pose_pub = rospy.Publisher('/person_pose', PoseStamped, queue_size=10)
        # MoveBase Action 客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

    def find_person(self):
        """
        从 /text_markers 话题中查找与 self.goal_object 匹配的 Marker
        返回：找到的 Marker 对象 (若找不到则返回 None)
        """
        target_marker = None
        try:
            marker_array_msg = rospy.wait_for_message(
                self.marker_topic, MarkerArray, timeout=5.0
            )
            rospy.loginfo(f"Found {len(marker_array_msg.markers)} markers in /text_markers.")
            for marker in marker_array_msg.markers:
                rospy.loginfo(f"Marker text: {marker.text}")
                if marker.text == self.goal_object:
                    rospy.loginfo(f"Found target object: {self.goal_object}")
                    target_marker = marker
                    break
            if not target_marker:
                rospy.logwarn("No matching person marker found in the MarkerArray.")
        except rospy.ROSException as e:
            rospy.logwarn(f"Timeout. No marker received. Error: {e}")
        
        return target_marker
    
    def get_robot_pose(self):
        """
        获取机器人在 map 坐标系下的位置和朝向
        :return: (position, orientation) 元组
        """
        try:
            # 查询从 base_link 到 map 的变换
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))

            # 提取位置和朝向
            position = transform.transform.translation
            orientation = transform.transform.rotation
            orientation = transform.transform.rotation
            quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(quaternion)  # 提取偏航角
            rospy.loginfo("Robot pose in map: x={}, y={}, z={}, w={}".format(
                position.x, position.y, orientation.z, orientation.w
            ))
            

            return position, yaw
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get robot pose: {e}")
            return None, None

    def get_nav_goal(self, marker):
        """
        根据找到的 Marker 计算导航目标点，
        并保持与 marker 所在位置的 safe_distance。
        """
        rospy.sleep(1)
        
        # 将 marker.pose (Pose) 封装成 PoseStamped 用于 tf2_geometry_msgs
        pose_in_marker_frame = PoseStamped()
        pose_in_marker_frame.header = marker.header
        pose_in_marker_frame.pose = marker.pose

        try:
            # 查询从 marker 坐标系到 map 坐标系的变换
            transform = self.tf_buffer.lookup_transform("map", marker.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

            # 进行坐标变换，得到 map 坐标系下的人员位姿
            pose_in_map = tf2_geometry_msgs.do_transform_pose(pose_in_marker_frame, transform)

            # 发布人员在 map 坐标系下的位姿
            self.person_pose_pub.publish(pose_in_map)
            rospy.loginfo("Published person pose in map frame.")
            rospy.loginfo("person pose in map: x={}, y={}, z={}, w={}".format(
                pose_in_map.pose.position.x,
                pose_in_map.pose.position.y,
                pose_in_map.pose.orientation.z,
                pose_in_map.pose.orientation.w
            ))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform person pose to map frame: {e}")

        robot_position, angle_robot = self.get_robot_pose()
        dx = pose_in_map.pose.position.x - robot_position.x
        dy = pose_in_map.pose.position.y - robot_position.y
        distance = math.sqrt(dx**2 + dy**2)
        rospy.loginfo(f"dx: {dx:.2f}, dy: {dy:.2f}")
        rospy.loginfo(f"Distance to person: {math.sqrt(dx**2 + dy**2):.2f} m")

        # 如果距离大于安全距离，则计算目标点
        if distance > self.safe_distance:
            # 计算目标点 (nx, ny) 在人员和机器人之间连线上，距离人员 safe_distance 的位置
            angle_to_person = math.atan2(dy, dx)
            nx = pose_in_map.pose.position.x - self.safe_distance * math.cos(angle_to_person)
            ny = pose_in_map.pose.position.y - self.safe_distance * math.sin(angle_to_person)
        else:
            # 如果距离小于等于安全距离，则保持当前位置
            nx = robot_position.x
            ny = robot_position.y
            angle_to_person = angle_robot
            rospy.logwarn("Person is too close. Staying at current position.")

        rospy.loginfo(f"Safe distance target: x={nx}, y={ny}")

        #  设置导航目标的姿态 (让机器人朝向 person)
        #  航向角 = angle_to_person
        qz = math.sin(angle_to_person / 2)
        qw = math.cos(angle_to_person / 2)
        # 构造 move_base 的导航目标
        nav_goal = MoveBaseGoal()
        nav_goal.target_pose.header.frame_id = "map"
        nav_goal.target_pose.header.stamp = rospy.Time.now()
        nav_goal.target_pose.pose.position.x = nx
        nav_goal.target_pose.pose.position.y = ny
        nav_goal.target_pose.pose.orientation.z = qz
        nav_goal.target_pose.pose.orientation.w = qw
        

        # 输出变换后的位姿
        rospy.loginfo("Pose in map: x={}, y={}, z={}, w={}".format(
            nav_goal.target_pose.pose.position.x,
            nav_goal.target_pose.pose.position.y,
            nav_goal.target_pose.pose.orientation.z,
            nav_goal.target_pose.pose.orientation.w
        ))
        
        nav_goal_msg = PoseStamped()
        nav_goal_msg.header = nav_goal.target_pose.header
        nav_goal_msg.pose = nav_goal.target_pose.pose
        self.nav_goal_pub.publish(nav_goal_msg)
        rospy.loginfo("Published navigation goal.")

        return nav_goal
    
    def get_rotation_goal(self, yaw_angle):
        """
        生成一个原地旋转的导航目标
        :param yaw_angle: 要旋转的角度（弧度制）
        :return: MoveBaseGoal
        """
        # 获取机器人当前位置
        robot_position, current_yaw = self.get_robot_pose()
        if robot_position is None:
            rospy.logerr("Failed to get robot pose for rotation.")
            return None

        # 计算新的朝向角度（原地旋转）
        target_yaw = current_yaw + yaw_angle
        target_yaw = math.atan2(math.sin(target_yaw), math.cos(target_yaw))  # 将角度归一化到 [-pi, pi]

        # 生成四元数表示的目标朝向
        qz = math.sin(target_yaw / 2)
        qw = math.cos(target_yaw / 2)

        # 构造旋转目标（保持当前位置，只改变朝向）
        rotation_goal = MoveBaseGoal()
        rotation_goal.target_pose.header.frame_id = "map"
        rotation_goal.target_pose.header.stamp = rospy.Time.now()
        rotation_goal.target_pose.pose.position.x = robot_position.x
        rotation_goal.target_pose.pose.position.y = robot_position.y
        rotation_goal.target_pose.pose.orientation.z = qz
        rotation_goal.target_pose.pose.orientation.w = qw

        rospy.loginfo(f"Generated rotation goal: yaw={math.degrees(target_yaw):.2f} degrees")
        return rotation_goal
    
    def get_detour_goal(self, base_goal, angle_offset):
        """
        生成绕行目标，基于原始导航目标偏移一定角度
        :param base_goal: 原始导航目标 (MoveBaseGoal)
        :param angle_offset: 绕行角度偏移量（弧度制）
        :return: 绕行后的导航目标 (MoveBaseGoal)
        """
        # 获取原始目标的位置和朝向
        goal_position = base_goal.target_pose.pose.position
        goal_orientation = base_goal.target_pose.pose.orientation
        _, _, base_yaw = euler_from_quaternion([goal_orientation.x, goal_orientation.y, goal_orientation.z, goal_orientation.w])

        # 计算绕行后的朝向角度
        detour_yaw = base_yaw + angle_offset
        detour_yaw = math.atan2(math.sin(detour_yaw), math.cos(detour_yaw))  # 归一化到 [-pi, pi]

        # 生成绕行后的四元数
        qz = math.sin(detour_yaw / 2)
        qw = math.cos(detour_yaw / 2)

        # 构造绕行目标（保持位置不变，改变朝向）
        detour_goal = MoveBaseGoal()
        detour_goal.target_pose.header.frame_id = "map"
        detour_goal.target_pose.header.stamp = rospy.Time.now()
        detour_goal.target_pose.pose.position.x = goal_position.x
        detour_goal.target_pose.pose.position.y = goal_position.y
        detour_goal.target_pose.pose.orientation.z = qz
        detour_goal.target_pose.pose.orientation.w = qw

        rospy.loginfo(f"Generated detour goal with yaw offset: {math.degrees(angle_offset):.2f} degrees")
        return detour_goal



    def navigate_to_person(self):
        """
        主流程：持续追踪 person，导航到目标后再重新获取目标
        """
        rate = rospy.Rate(1)  # 设置循环频率
        rotation_goals = [math.radians(angle) for angle in [15, 30, 45, 60, -60, -15, -30, -45, -60, 60]]
        detour_angles = [math.radians(15), math.radians(-15), math.radians(30), math.radians(-30)]
        while not rospy.is_shutdown():
            # 查找人员位置
            person_marker = self.find_person()
            if person_marker is None:
                rospy.logwarn("No person marker found. Starting rotation search...")
                for angle in rotation_goals:
                    # 计算旋转目标角度（原地旋转）
                    rotation_goal = self.get_rotation_goal(angle)
                    self.move_base_client.send_goal(rotation_goal)
                    finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(10.0))

                    if finished_within_time and self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                        rospy.loginfo("Rotation completed. Retrying to find person...")
                        person_marker = self.find_person()
                        if person_marker:
                            rospy.loginfo("Person found after rotation.")
                            break
                    else:
                        rospy.logwarn("Rotation failed or timed out. Retrying...")
                if person_marker is None:
                    rospy.logerr("Person not found after rotation search. Exiting.")
                    break

            # 获取导航目标
            goal = self.get_nav_goal(person_marker)
            if goal is None:
                rospy.logwarn("Could not compute navigation goal. Retrying...")
                rate.sleep()
                continue

            rospy.loginfo("Sending goal to move_base...")
            self.move_base_client.send_goal(goal)

            finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(4.0))
            if finished_within_time:
                state = self.move_base_client.get_state()
                if state == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo("Navigation to person succeeded!")
                    rospy.sleep(2.0)  # 等待 2 秒后重新导航
                    continue
                else:
                    rospy.logwarn(f"Navigation failed with state: {state}. Trying detour...")
            else:
                rospy.logwarn("Navigation timed out. Trying detour...")

            # 导航失败后尝试绕行
            for i, angle in enumerate(detour_angles):
                detour_goal = self.get_detour_goal(goal, angle)
                self.move_base_client.send_goal(detour_goal)
                finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(15.0))

                if finished_within_time and self.move_base_client.get_state() == actionlib.GoalStatus.SUCCEEDED:
                    rospy.loginfo(f"Detour attempt {i + 1} succeeded!")
                    break
                else:
                    rospy.logwarn(f"Detour attempt {i + 1} failed.")

            # 导航完成或超时后，重新获取目标
            rospy.sleep(0.1)  # 等待0.1秒后重新导航


if __name__ == "__main__":
    rospy.init_node("navigation_to_person_node")
    nav_person = NavigationToPerson(safe_distance=1.0, target_object="person")
    try:
        nav_person.navigate_to_person()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation interrupted.")
