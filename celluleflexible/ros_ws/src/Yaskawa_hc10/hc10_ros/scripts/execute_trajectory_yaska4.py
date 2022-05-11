#!/usr/bin/env python3
from http import client
from ntpath import join
from os import wait
from pydoc import ErrorDuringImport
import sys
import copy
import rospy
import actionlib
import moveit_commander
import actionlib_tutorials.msg
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg
from moveit_msgs.msg import MoveGroupSequenceActionResult, MotionSequenceRequest, MotionSequenceItem, Constraints, JointConstraint, MoveGroupSequenceActionGoal, MoveGroupSequenceAction
from sensor_msgs.msg import JointState

mymsgYaska4 = FinDeplacerPiece_Msg()
trajectoryState = 3 # SUCCEEDED = 3 in status
trajectoryErrorCode = 1 #Succes = 1 in error_code

def TrajectoryResultCallback(errormsg):
	global trajectoryState
	global trajectoryErrorCode
	trajectoryState = errormsg.status.status
	trajectoryErrorCode = errormsg.result.error_code.val

def ErrorTrajectoryExecution():
	choice = 0
	while ((choice != 1) & (choice != 2)):
		print("Error during execution of trajectory :")
		print("""What would you like to do? 
		1. Finish trajectory 
		2. Abort trajectory """)
		try:
			choice = int(input('Choice:'))
		except ValueError:
			print("Not a number")
		print("Invalid choice, try again")
	return choice

def BuildSequenceRequest(pub_yaska4):
	motionPlanItemInitialize = MotionSequenceItem()
	motionSequenceRequest = MotionSequenceRequest()
	constraints_ = Constraints()
	if pub_yaska4.data == 1:
		group = moveit_commander.MoveGroupCommander("DN1P", robot_description="/yaska4/robot_description", ns="/yaska4")
		motionPlanItemInitialize.req.group_name = "DN1P"
	elif pub_yaska4.data == 2:
		group = moveit_commander.MoveGroupCommander("DN2P", robot_description="/yaska4/robot_description", ns="/yaska4")
		motionPlanItemInitialize.req.group_name = "DN2P"
	elif pub_yaska4.data == 3:
		group = moveit_commander.MoveGroupCommander("DPN1", robot_description="/yaska4/robot_description", ns="/yaska4")
		motionPlanItemInitialize.req.group_name = "DPN1"
	elif pub_yaska4.data == 4:
		group = moveit_commander.MoveGroupCommander("DPN2", robot_description="/yaska4/robot_description", ns="/yaska4")
		motionPlanItemInitialize.req.group_name = "DPN2"
	else :
		print("Error callback control yakuza \n")
		return
	print ("Value of deplacement %d" %pub_yaska4.data)
	motionPlanItemInitialize.req.pipeline_id = "pilz_industrial_motion_planner"
	motionPlanItemInitialize.req.planner_id = "PTP"
	motionPlanItemInitialize.req.max_velocity_scaling_factor = 1
	motionPlanItemInitialize.req.max_acceleration_scaling_factor = 1
	motionPlanItemInitialize.blend_radius = 0
	#initialise la trajectoire à la position dans laquelle se trouve le robot
	jointName_ = group.get_active_joints()
	startState_ = group.get_current_joint_values()
	motionPlanItemInitialize.req.start_state.joint_state = JointState(name=jointName_,position=startState_)
	#on récupère le premier goal
	#récupère le nom des différentes positions définie dans le groupe du fichier srdf
	group_state = group.get_named_targets()
	#on ajoute notre premier item a la sequence request
	dict_ = group.get_named_target_values(group_state[0])

	for i in range(0,len(jointName_)):
		goalJointTemp_ = JointConstraint()
		goalJointTemp_.joint_name = jointName_[i]
		goalJointTemp_.position = dict_.get(jointName_[i])
		constraints_.joint_constraints.append(goalJointTemp_)

	motionPlanItemInitialize.req.goal_constraints.append(constraints_)
	motionSequenceRequest.items.append(motionPlanItemInitialize)


	for n in range(1,len(group_state)):
		#on construit les autres items
		motionPlanItem = MotionSequenceItem()
		constraints2_ = Constraints()
		motionPlanItem.req.pipeline_id = "pilz_industrial_motion_planner"
		motionPlanItem.req.planner_id = "PTP"
		motionPlanItem.req.max_velocity_scaling_factor = 1
		motionPlanItem.req.max_acceleration_scaling_factor = 1
		motionPlanItem.blend_radius = 0
		motionPlanItem.req.group_name = motionPlanItemInitialize.req.group_name
		dict_ = group.get_named_target_values(group_state[n])
		for i in range(0,len(jointName_)):
			goalJointTemp_ = JointConstraint()
			goalJointTemp_.joint_name = jointName_[i]
			goalJointTemp_.position = dict_.get(jointName_[i])
			constraints2_.joint_constraints.append(goalJointTemp_)
		motionPlanItem.req.goal_constraints.append(constraints2_)
		motionSequenceRequest.items.append(copy.deepcopy(motionPlanItem))
	# motionSequenceRequest._check_types()
	return motionSequenceRequest

def ControlCallback(pub_yaska4):
	client_ = actionlib.SimpleActionClient('/yaska4/sequence_move_group', MoveGroupSequenceAction)
	client_.wait_for_server()
	goal_ = MoveGroupSequenceActionGoal()
	myPlan_ = BuildSequenceRequest(pub_yaska4)
	goal_.goal.request = myPlan_
	# pub_motionSequenceRequest.publish(goal_)
	print("Executing trajectory :")
	client_.send_goal(goal_.goal)
	client_.wait_for_result()
	# rospy.loginfo(client_.action_client.ActionResult)
	# print("Results: %s" %client_.action_client.ActionResult)
	mymsgYaska4.FinDeplacerR4 = 1
	rospy.loginfo(mymsgYaska4)
	pub_fintache.publish(mymsgYaska4)
	rospy.sleep(1)
	mymsgYaska4.FinDeplacerR3 = 0
	pub_fintache.publish(mymsgYaska4)

if __name__ == "__main__":
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('execute_trajectory_yaska4',anonymous=True)
	rospy.Subscriber('/control_robot_yaska4',Int32, ControlCallback)
	#rospy.Subscriber('/sequence_move_group/result', MoveGroupSequenceActionResult, TrajectoryResultCallback)
	pub_fintache = rospy.Publisher("/commande/Simulation/finTache", FinDeplacerPiece_Msg,  queue_size=1)
	# pub_motionSequenceRequest = rospy.Publisher( "/sequence_move_group/goal", MoveGroupSequenceActionGoal, queue_size=1)
	rospy.spin()	
	moveit_commander.roscpp_shutdown()


