#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import JointState
from open_manipulator_msgs.msg import KinematicsPose


class OpenManipulatorStateSubscriber:

    def __init__(self):

        # Subscribe to joint states
        self.joint_subscriber = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_callback,
            queue_size=10
        )

        # Subscribe to end-effector pose
        self.pose_subscriber = rospy.Subscriber(
            '/gripper/kinematics_pose',
            KinematicsPose,
            self.pose_callback,
            queue_size=10
        )

        rospy.loginfo(
            "OpenMANIPULATOR-X state subscriber started"
        )

    def joint_callback(self, msg):

        rospy.loginfo("----- Joint State -----")

        for i, name in enumerate(msg.name):

            position = msg.position[i]

            if i < len(msg.velocity):
                velocity = msg.velocity[i]
            else:
                velocity = 0.0

            if i < len(msg.effort):
                effort = msg.effort[i]
            else:
                effort = 0.0

            rospy.loginfo(
                "%s: position = %.4f rad, "
                "velocity = %.4f rad/s, "
                "effort = %.4f",
                name,
                position,
                velocity,
                effort
            )

    def pose_callback(self, msg):

        rospy.loginfo("----- End-Effector Pose -----")

        p = msg.pose.position
        q = msg.pose.orientation

        rospy.loginfo(
            "Position: x = %.4f, y = %.4f, z = %.4f",
            p.x,
            p.y,
            p.z
        )

        rospy.loginfo(
            "Orientation: qx = %.4f, qy = %.4f, "
            "qz = %.4f, qw = %.4f",
            q.x,
            q.y,
            q.z,
            q.w
        )


def main():

    rospy.init_node(
        'open_manipulator_state_subscriber'
    )

    OpenManipulatorStateSubscriber()

    rospy.spin()


if __name__ == '__main__':
    main()
