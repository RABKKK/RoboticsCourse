import rospy
from open_manipulator_msgs.srv import SetJointPosition, SetJointPositionRequest

def move_robot(jval,pathtime):
    rospy.init_node('open_manipulator_service_client')
    rospy.loginfo('Waiting for /goal_joint_space_path service...')
    rospy.wait_for_service('/goal_joint_space_path')
    try:
        move_joint = rospy.ServiceProxy('/goal_joint_space_path',SetJointPosition)
        req=SetJointPositionRequest()
        req.planning_group='arm'
        req.joint_position.joint_name = ['joint1','joint2', 'joint3', 'joint4']
        req.joint_position.position = jval
        req.joint_position.max_accelerations_scaling_factor=0.2
        req.joint_position.max_velocity_scaling_factor=0.3
        req.path_time=pathtime
        response = move_joint(req)
        rospy.loginfo('Trajectory planned: %s',response.is_planned)
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s',e)

if __name__ == '__main__':
    move_robot([0.0,0.0,0.0,0.0],3.0)
    rospy.sleep(3.0)	
    move_robot([1.0,0.0,0.0,0.0],3.0)	
