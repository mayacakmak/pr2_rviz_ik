import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('kinematics_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('control_msgs')

import threading
import rospy
from numpy import array, sign, pi
from kinematics_msgs.srv import GetKinematicSolverInfo, GetPositionIK
from kinematics_msgs.srv import GetPositionIKRequest
from actionlib import SimpleActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryGoal
from control_msgs.msg import JointTrajectoryAction


class IK:

    r_joint_names = ['r_shoulder_pan_joint',
                      'r_shoulder_lift_joint',
                      'r_upper_arm_roll_joint',
                      'r_elbow_flex_joint',
                      'r_forearm_roll_joint',
                      'r_wrist_flex_joint',
                      'r_wrist_roll_joint']

    l_joint_names = ['l_shoulder_pan_joint',
                     'l_shoulder_lift_joint',
                     'l_upper_arm_roll_joint',
                     'l_elbow_flex_joint',
                     'l_forearm_roll_joint',
                     'l_wrist_flex_joint',
                     'l_wrist_roll_joint']
                     

    def __init__(self, side_prefix):
        self.side_prefix = side_prefix

        # Set up Inversse Kinematics services
        ik_info_srv_name = ('pr2_' + self._side() + '_arm_kinematics_simple/get_ik_solver_info')
        ik_srv_name = 'pr2_' + self._side() + '_arm_kinematics_simple/get_ik'

        rospy.loginfo('Waiting for IK info service to respond.')
        rospy.wait_for_service(ik_info_srv_name)
        ik_info_srv = rospy.ServiceProxy(ik_info_srv_name, GetKinematicSolverInfo)
        solver_info = ik_info_srv()
        self.ik_joints = solver_info.kinematic_solver_info.joint_names
        self.ik_limits = solver_info.kinematic_solver_info.limits

        rospy.loginfo('Waiting for IK service to respond.')
        rospy.wait_for_service(ik_srv_name)
        self.ik_srv = rospy.ServiceProxy(ik_srv_name, GetPositionIK, persistent=True)

        # Set up common parts of an IK request
        self.ik_request = GetPositionIKRequest()
        self.ik_request.timeout = rospy.Duration(4.0)
        self.ik_request.ik_request.ik_link_name = solver_info.kinematic_solver_info.link_names[0]
        self.ik_request.ik_request.pose_stamped.header.frame_id = 'base_link'
        self.ik_request.ik_request.ik_seed_state.joint_state.name = self.ik_joints
        self.ik_request.ik_request.ik_seed_state.joint_state.position = [0] * len(self.ik_joints)
    
        motion_request_name = '/' + side_prefix + '_arm_motion_request/joint_trajectory_action'
        self.traj_action_client = SimpleActionClient(motion_request_name, JointTrajectoryAction)
        rospy.loginfo('Waiting for a response from the trajectory motion request server on arm: ' + side_prefix)
        self.traj_action_client.wait_for_server()
        rospy.loginfo('IK ready for arm: ' + side_prefix)

    def _side(self):
        if (self.side_prefix == 'r'):
            return 'right'
        else:
            return 'left'

    def move_to_joints(self, positions, time_to_joint):
        velocities = [0] * len(positions)
        traj_goal = JointTrajectoryGoal()
    	traj_goal.trajectory.header.stamp = (rospy.Time.now() + rospy.Duration(0.1))
    	traj_goal.trajectory.points.append(JointTrajectoryPoint(positions=positions,
                velocities=velocities, time_from_start=rospy.Duration(time_to_joint)))
        if (self.side_prefix == 'r'):
            traj_goal.trajectory.joint_names = IK.r_joint_names
        else:
            traj_goal.trajectory.joint_names = IK.l_joint_names
        self.traj_action_client.send_goal(traj_goal)

    def get_ik_for_ee(self, ee_pose, seed=None):
        ''' Finds the IK solution for given end effector pose'''
        
        self.ik_request.ik_request.pose_stamped.pose = ee_pose
        
        if seed == None:
            seed = []
            for i in range(0, len(self.ik_joints)):
                seed.append((self.ik_limits[i].min_position +
                             self.ik_limits[i].max_position) / 2.0)

        self.ik_request.ik_request.ik_seed_state.joint_state.position = seed
        
        try:
            rospy.loginfo('Requesting IK.')
            response = self.ik_srv(self.ik_request)
            if(response.error_code.val == response.error_code.SUCCESS):
                joints = response.solution.joint_state.position
            else:
                rospy.logwarn('Could not find IK solution.')
                return None
        except rospy.ServiceException:
            rospy.logerr('Exception while getting the IK solution.')
            return None

        rollover = array((array(joints) - array(seed)) / pi, int)
        joints -= ((rollover + (sign(rollover) + 1) / 2) / 2) * 2 * pi

        return joints


