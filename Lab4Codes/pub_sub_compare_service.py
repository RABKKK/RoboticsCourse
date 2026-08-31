import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import JointState
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest
from open_manipulator_msgs.msg import JointPosition

class ServiceCycloidalController:
    def __init__(self):
        rospy.init_node('cycloidal_service_actuator')

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']
        self.service_name = 'goal_joint_space_path'

        # Wait for the service server provided by open_manipulator_controller
        rospy.loginfo("Waiting for Service: %s...", self.service_name)
        rospy.wait_for_service(self.service_name)
        self.set_joint_position_srv = rospy.ServiceProxy(self.service_name, SetJointPosition)
        rospy.loginfo("Service connected successfully.")

        # Subscribe to joint states for actual feedback
        rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)

        # Trajectory setup parameters
        self.t_total = 5.0    # Duration of trajectory in seconds
        self.freq = 20.0      # Service command rate (Hz)
        self.dt = 1.0 / self.freq

        # Target start and end positions in radians
        #self.q_start = np.array([0.0, -0.5, 0.2, 0.3])
        self.q_start = np.array([0.0, 0.0, 0.0, 0.0])
        
        #self.q_end   = np.array([0.8,  0.2, -0.4, 0.5])
        self.q_end   = np.array([np.pi/2,-np.pi/4,np.pi/4,-np.pi/8])           
        
        # State storage
        self.actual_positions = []
        self.actual_timestamps = []
        self.start_time = None
        self.recording = False

    def generate_cycloidal_point(self, t):
        """Calculates current position along the cycloidal curve."""
        tau = np.clip(t / self.t_total, 0.0, 1.0)
        s = tau - (1.0 / (2.0 * np.pi)) * np.sin(2.0 * np.pi * tau)
        return self.q_start + (self.q_end - self.q_start) * s

    def joint_state_callback(self, msg):
        """Records state feedback."""
        if not self.recording:
            return

        curr_time = rospy.get_time() - self.start_time
        positions = []

        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])

        if len(positions) == len(self.joint_names):
            self.actual_timestamps.append(curr_time)
            self.actual_positions.append(positions)

    def send_joint_service_goal(self, target_positions, path_time):
        """Builds and sends SetJointPosition request."""
        req = SetJointPositionRequest()
        req.planning_group = ""  # Default planning group
        
        jp = JointPosition()
        jp.joint_name = self.joint_names
        jp.position = target_positions.tolist()
        
        req.joint_position = jp
        req.path_time = path_time

        try:
            res = self.set_joint_position_srv(req)
            return res.is_planned
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return False

    def run_trajectory(self):
        # 1. First move arm safely to starting position
        rospy.loginfo("Moving arm to initial state...")
        self.send_joint_service_goal(self.q_start, path_time=2.0)
        rospy.sleep(2.5)

        # 2. Execute incremental cycloidal trajectory via service
        num_points = int(self.t_total * self.freq)
        time_vector = np.linspace(0, self.t_total, num_points)
        commanded_positions = []

        rospy.loginfo("Executing Cycloidal Path via Service Calls...")
        self.start_time = rospy.get_time()
        self.recording = True

        rate = rospy.Rate(self.freq)
        for t in time_vector:
            q_target = self.generate_cycloidal_point(t)
            commanded_positions.append(q_target)

            # Request step move with small path_time allocation
            self.send_joint_service_goal(q_target, path_time=self.dt)
            rate.sleep()

        self.recording = False
        rospy.loginfo("Execution complete.")
        return time_vector, np.array(commanded_positions)


def plot_comparison(time_cmd, q_cmd, time_act, q_act, joint_names):
    """Plots commanded targets vs real response."""
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    axes = axes.flatten()

    for i in range(len(joint_names)):
        axes[i].plot(time_cmd, q_cmd[:, i], 'r--', label='Commanded Target (Service)', linewidth=2)
        if len(q_act) > 0:
            axes[i].plot(time_act, q_act[:, i], 'b-', label='Actual Feedback', alpha=0.8)

        axes[i].set_title(f'Joint Performance: {joint_names[i]}')
        axes[i].set_xlabel('Time (s)')
        axes[i].set_ylabel('Position (rad)')
        axes[i].grid(True)
        axes[i].legend()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    try:
        controller = ServiceCycloidalController()
        t_cmd, q_cmd = controller.run_trajectory()

        t_act = np.array(controller.actual_timestamps)
        q_act = np.array(controller.actual_positions)

        plot_comparison(t_cmd, q_cmd, t_act, q_act, ['Joint 1', 'Joint 2', 'Joint 3', 'Joint 4'])

    except rospy.ROSInterruptException:
        pass
