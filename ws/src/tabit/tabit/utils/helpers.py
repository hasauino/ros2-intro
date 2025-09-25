from geometry_msgs.msg import PoseStamped, TransformStamped


def transform_stamped_to_pose_stamped(transform: TransformStamped) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header = transform.header
    pose_stamped.pose.position.x = transform.transform.translation.x
    pose_stamped.pose.position.y = transform.transform.translation.y
    pose_stamped.pose.position.z = transform.transform.translation.z
    pose_stamped.pose.orientation.x = transform.transform.rotation.x
    pose_stamped.pose.orientation.y = transform.transform.rotation.y
    pose_stamped.pose.orientation.z = transform.transform.rotation.z
    pose_stamped.pose.orientation.w = transform.transform.rotation.w
    return pose_stamped


def odom_to_pose_stamped(odom) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header = odom.header
    pose_stamped.pose = odom.pose.pose
    return pose_stamped
