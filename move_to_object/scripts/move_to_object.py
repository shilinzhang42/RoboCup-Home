import actionlib
import rospy
import moveit_commander
import tf2_ros
import numpy as np
import math
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf.transformations import euler_from_quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from sensor_msgs.msg import PointCloud2
from moveit_msgs.msg import (Constraints, 
                             PositionConstraint, 
                             BoundingVolume, 
                             PlanningScene,
                             CollisionObject,
                             AttachedCollisionObject)


class MoveToObject():
    def __init__(self, safe_distance=0.5, target_object="bottle", final_orientation=[0.0, 0.0]):
        """
        :param safe_distance: 机器人与人员保持的安全距离 (单位: 米)
        :param target_object: 在 /text_markers 中要寻找的目标对象文字
        """
        self.safe_distance = safe_distance
        self.goal_object = target_object
        self.final_orientation = final_orientation

        # 用于等待/获取 MarkerArray
        self.marker_topic = "/text_markers"
        
        # TF2 缓存
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 发布人员位姿和导航目标
        self.nav_goal_pub = rospy.Publisher('/nav_goal', PoseStamped, queue_size=10)
        self.object_pose_pub = rospy.Publisher('/object_pose', PoseStamped, queue_size=10)
        # 头部控制器发布器
        self.head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=10)

        # MoveBase Action 客户端
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base server")

        self.tracking_timer = rospy.Timer(
            rospy.Duration(0.2),  # 每隔 0.2s 执行一次 (5Hz)
            self.head_tracking_cb
        )

    def find_object(self):
        """
        从 /text_markers 话题中查找与 self.goal_object 匹配的 Marker
        返回：找到的 Marker 对象 (若找不到则返回 None)
        """
        target_marker = None
        try:
            marker_array_msg = rospy.wait_for_message(
                self.marker_topic, MarkerArray, timeout=2.0
            )
            # rospy.loginfo(f"Found {len(marker_array_msg.markers)} markers in /text_markers.")
            for marker in marker_array_msg.markers:
                # rospy.loginfo(f"Marker text: {marker.text}")
                if marker.text == self.goal_object:
                    rospy.loginfo(f"Found target object: {self.goal_object}")
                    target_marker = marker
                    break
            if not target_marker:
                rospy.logwarn("No matching object marker found in the MarkerArray.")
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
            self.object_pose_pub.publish(pose_in_map)
            rospy.loginfo("Published object pose in map frame.")
            rospy.loginfo("object pose in map: x={}, y={}, z={}, w={}".format(
                pose_in_map.pose.position.x,
                pose_in_map.pose.position.y,
                pose_in_map.pose.orientation.z,
                pose_in_map.pose.orientation.w
            ))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to transform object pose to map frame: {e}")

        robot_position, angle_robot = self.get_robot_pose()
        dx = pose_in_map.pose.position.x - robot_position.x
        dy = pose_in_map.pose.position.y - robot_position.y
        distance = math.sqrt(dx**2 + dy**2)
        rospy.loginfo(f"dx: {dx:.2f}, dy: {dy:.2f}")
        rospy.loginfo(f"Distance to object: {math.sqrt(dx**2 + dy**2):.2f} m")

        x, y = final_orientation
        final_angle = math.atan2(y, x)

        # 如果距离大于安全距离，则计算目标点
        if distance > self.safe_distance:
            # 计算目标点 (nx, ny) 在人员和机器人之间连线上，距离人员 safe_distance 的位置
            nx = pose_in_map.pose.position.x - self.safe_distance * math.cos(final_angle)
            ny = pose_in_map.pose.position.y - self.safe_distance * math.sin(final_angle)
        else:
            # 如果距离小于等于安全距离，则保持当前位置
            nx = robot_position.x
            ny = robot_position.y
            final_angle = angle_robot
            rospy.logwarn("Person is too close. Staying at current position.")

        rospy.loginfo(f"Safe distance target: x={nx}, y={ny}")

        #  设置导航目标的姿态 (让机器人朝向 object)
        #  航向角 = final_angle
        qz = math.sin(final_angle / 2)
        qw = math.cos(final_angle / 2)
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

    def _look_direction(self, positions):
        head_joint_traj = JointTrajectory()
        head_joint_traj.joint_names = ["head_1_joint", "head_2_joint"]
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(0.15)
        head_joint_traj.points.append(point)
        self.head_pub.publish(head_joint_traj)
    
    def track_object_with_head(self, marker):
        marker_position = marker.pose.position
        px, py = marker_position.x, marker_position.y
        angle = math.atan2(py, px)
        self._look_direction([angle, 0.0])

    def head_tracking_cb(self, event):
            """
            定时器回调函数，用于持续追踪目标。
            1. 调用 find_object() 查找目标 Marker
            2. 如果找到，则调用 track_object_with_head() 调整头部
            """
            obj_marker = self.find_object()
            if obj_marker is not None:
                self.track_object_with_head(obj_marker)

    def navigate_to_object(self):
        """
        主流程：持续追踪 object，导航到目标后再重新获取目标
        """
        # 查找人员位置
        # 查找人员位置
        obj_marker = self.find_object()

        if obj_marker is None:
                rospy.logerr("Object not found.")
                
        # 获取导航目标
        goal = self.get_nav_goal(obj_marker)
        if goal is None:
            rospy.logwarn("Could not compute navigation goal. Retrying...")

        rospy.loginfo("Sending goal to move_base...")
        self.move_base_client.send_goal(goal)

        finished_within_time = self.move_base_client.wait_for_result(rospy.Duration(10.0))
        if finished_within_time:
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Navigation to object succeeded!")
                rospy.sleep(0.3)  # 等待机器人稳定
            else:
                rospy.logwarn("Navigation to object failed.")
                return False
        
        return True

if __name__ == "__main__":
    # get parameters
    safe_distance = rospy.get_param("~safe_distance", 0.5)
    target_object = rospy.get_param("~target_object", "bottle")
    final_orientation = rospy.get_param("~orientation", [0.0, 0.0])
    rospy.init_node("move_to_object")
    move_to_object = MoveToObject(safe_distance, target_object, final_orientation)


    if move_to_object.navigate_to_object():
        rospy.loginfo("Finished navigation to object.")
    pass