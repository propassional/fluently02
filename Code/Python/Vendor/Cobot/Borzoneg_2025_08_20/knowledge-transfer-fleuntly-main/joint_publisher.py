import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointPublisher(Node):
    def __init__(self, topic_name: str, joint_names=["J1", "J2", "J3", "J4", "J5", "J6"]):
        super().__init__('JointPublisher')
        self.publisher = self.create_publisher(JointState, topic_name, 10)
        self.joint_names = joint_names

    def send_joint(self, q: list, names: list = None, qd: list = None, qdd: list = None):
        if len(q) != 6:
            raise ValueError
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names if names is not None else self.joint_names
        # msg.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] # for ur5e
        msg.position = [float(x) for x in q]
        if qd is not None:
            msg.velocity = [float(xd) for xd in qd]
        if qdd is not None:
            msg.effort = [float(xdd) for xdd in qdd]
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg)
    
    def send_traj(self, qs, time_step=0.01, names: list = None, qds: list = None, qdds: list = None):
        for q in qs:
            self.send_joint(q, names=names)
            time.sleep(time_step)