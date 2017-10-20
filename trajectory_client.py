import roslib
roslib.load_manifest('snake_dynamixel')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from snake_msgs.msg import SnakeJointData, SnakeJointCommand
from dynamixel_msgs.msg import JointState, MotorState, MotorStateList

JointNumber = 29
joint_position = [0 for _ in range(JointNumber)]

# class Joint:
#         def __init__(self, motor_name):
# 		self.name = motor_name           
#     		self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
# 		rospy.loginfo('Waiting for joint trajectory action')
# 	    	self.jta.wait_for_server()
# 		rospy.loginfo('Found joint trajectory action!')
#
# 										            
# 	def SetJointPosition(self, joint_command.joint_index, joint_command.target_all, joint_command.target_position, joint_command.target_velocity)
#
# 		goal = FollowJointTrajectoryGoal()
# 		for value in range(1, JointNumber):
# 			if value < 10:
# 				goal.trajectory.joint_names[value] = 'Joint0' + str(value)
# 			else:
# 				goal.trajectory.joint_names[value] = 'Joint' + str(value)
#     	    	point = JointTrajectoryPoint()
# 		point.positions = angles
#     		point.time_from_start = rospy.Duration(3)
# 		goal.trajectory.points.append(point)
# 		self.jta.send_goal_and_wait(goal)

def ReadJointPosition(MotorStateList):
	for MotorState in MotorStateList.motor_states:
		joint_position[MotorState.id - 10] = MotorState.position

def ShowPosition(joint_position):
	i = 0
	print u'Joint Position'
	for degree in joint_position:
		print 'motor id: %d MotorPos: %d' %(i, degree)
		i = i + 1

def ShowFlags(data):
	# print "Get requests"
	i = 100

def ShowTargetPosition(data):
	print "Joint Number: %s Target Pos: %s" %(str(data.joint_index), str(data.value))

def main():
	r = rospy.Rate(10)
	#joints = Joint('snake_joint')
	rospy.Subscriber('motor_states/dxl_manager', MotorStateList, ReadJointPosition)
	rospy.Subscriber('joint_command', SnakeJointCommand, ShowFlags)
	rospy.Subscriber('joint_target_position', SnakeJointData, ShowTargetPosition)

	while not rospy.is_shutdown():
		print u'Hit a key to save/see the motor posiitons'
		key = raw_input('>>')
		ShowPosition(joint_position)
		r.sleep()


if __name__ == '__main__':
	rospy.init_node('SnakeJointController')
	main()
