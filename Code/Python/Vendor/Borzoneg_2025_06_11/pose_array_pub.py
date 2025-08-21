from geometry_msgs.msg import PoseArray, Pose
from rclpy.node import Node
from std_msgs.msg import Header
import spatialmath as sm

class PoseArrayPublisher(Node):
    def __init__(self, topic_name: str):
        super().__init__('PoseArrayPublisher')
        self.publisher = self.create_publisher(PoseArray, topic_name, 10)

    def send_poses(self, Ts: list[sm.SE3]):
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'  # Set the frame ID as needed
        for T in Ts:
            pose = Pose()
            pose.position.x = float(T.t[0])
            pose.position.y = float(T.t[1])
            pose.position.z = float(T.t[2])
            pose.orientation.x = float(sm.SO3(T.R).UnitQuaternion().vec_xyzs[0])
            pose.orientation.y = float(sm.SO3(T.R).UnitQuaternion().vec_xyzs[1])
            pose.orientation.z = float(sm.SO3(T.R).UnitQuaternion().vec_xyzs[2])
            pose.orientation.w = float(sm.SO3(T.R).UnitQuaternion().vec_xyzs[3])
            msg.poses.append(pose)
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)