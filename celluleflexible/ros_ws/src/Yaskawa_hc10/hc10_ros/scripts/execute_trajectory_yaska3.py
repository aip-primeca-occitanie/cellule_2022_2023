#!/usr/bin/env python3
from ntpath import join
from os import wait
from pydoc import ErrorDuringImport
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg
from moveit_msgs.msg import MoveGroupSequenceActionResult, MotionSequenceRequest, MotionSequenceItem, Constraints, JointConstraint, MoveGroupSequenceActionGoal
from sensor_msgs.msg import JointState

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('execute_trajectory_yaska3',anonymous=True)

#Misc variables
robotyaska3 = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
mymsgYaska3 = FinDeplacerPiece_Msg()
trajectoryState = 3 # SUCCEEDED = 3 in status
trajectoryErrorCode = 1 #Succes = 1 in error_code

# goalJointTrajectory_ = [JointConstraint()]
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

def BuildSequenceRequest(pub_yaska3):
	motionPlanItemInitialize = MotionSequenceItem()
	motionSequenceRequest = MotionSequenceRequest()
	constraints_ = Constraints()
	if pub_yaska3.data == 1:
		group = moveit_commander.MoveGroupCommander("yaska3_DN1P")
		motionPlanItemInitialize.req.group_name = "yaska3_DN1P"
	elif pub_yaska3.data == 2:
		group = moveit_commander.MoveGroupCommander("yaska3_DN2P")
		motionPlanItemInitialize.req.group_name = "yaska3_DN2P"
	elif pub_yaska3.data == 3:
		group = moveit_commander.MoveGroupCommander("yaska3_DPN1")
		motionPlanItemInitialize.req.group_name = "yaska3_DPN1"
	elif pub_yaska3.data == 4:
		group = moveit_commander.MoveGroupCommander("yaska3_DPN2")
		motionPlanItemInitialize.req.group_name = "yaska3_DPN2"
	else :
		print("Error callback control yakuza \n")
		return
	print ("Value of deplacement %d" %pub_yaska3.data)
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
	#motionPlanItemInitialize.req.goal_constraints = group.get_named_target_values(group_state[1])
	#goalJointValue_ = []
	dict_ = group.get_named_target_values(group_state[1])
	for i in range(0,len(jointName_)):
		goalJointTemp_ = JointConstraint()
		# goalJointValue_.append(dict_[jointName_[i]])
		goalJointTemp_.joint_name = jointName_[i]
		goalJointTemp_.position = dict_.get(jointName_[i])
		constraints_.joint_constraints.append(goalJointTemp_)
	# constraints_ = Constraints(joint_constraints=goalJoint_)
	motionPlanItemInitialize.req.goal_constraints.append(constraints_)
	motionSequenceRequest.items.append(motionPlanItemInitialize)


	for n in range(2,len(group_state)):
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
			# goalJointValue_.append(dict_[jointName_[i]])
			goalJointTemp_.joint_name = jointName_[i]
			goalJointTemp_.position = dict_.get(jointName_[i])
			constraints2_.joint_constraints.append(goalJointTemp_)
		motionPlanItem.req.goal_constraints.append(constraints2_)
		motionSequenceRequest.items.append(copy.deepcopy(motionPlanItem))
	# print("motionSequenceRequest: %s" %motionSequenceRequest)
	# motionSequenceRequest._check_types()
	return motionSequenceRequest

def ControlCallback(pub_yaska3):
	goal_ = MoveGroupSequenceActionGoal()
	myPlan_ = BuildSequenceRequest(pub_yaska3)
	goal_.goal.request = myPlan_
	print("Goal: %s" %goal_)
	pub_motionSequenceRequest.publish(goal_)
	rospy.sleep(60)
	mymsgYaska3.FinDeplacerR3 = 1
	rospy.loginfo(mymsgYaska3)
	pub_fintache.publish(mymsgYaska3)
	rospy.sleep(1)
	mymsgYaska3.FinDeplacerR3 = 0
	pub_fintache.publish(mymsgYaska3)

if __name__ == "__main__":
	rospy.Subscriber('/control_robot_yaska3',Int32, ControlCallback)
	#rospy.Subscriber('/sequence_move_group/result', MoveGroupSequenceActionResult, TrajectoryResultCallback)
	# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                            moveit_msgs.msg.DisplayTrajectory,
    #                                            queue_size=20)
	pub_fintache = rospy.Publisher("/commande/Simulation/finTache", FinDeplacerPiece_Msg,  queue_size=1)
	pub_motionSequenceRequest = rospy.Publisher( "/sequence_move_group/goal", MoveGroupSequenceActionGoal, queue_size=1)
	rospy.spin()
	
moveit_commander.roscpp_shutdown()

	#Choose your planning pipeline ("ompl", "pilz_industrial_motion_planner")
	# group.set_planning_pipeline_id("pilz_industrial_motion_planner")
	# #Set id planner ("Point To Point")
	# group.set_planner_id("PTP")
	# #Uncomment the line below if you want to set the velocity of the robot manually (range 0 - 1 )
	# group.set_max_velocity_scaling_factor(1)
	# group.set_max_acceleration_scaling_factor(1)
	#Group states taken from the srdf file
	# group_state = group.get_named_targets()
	#Planning and executing with set_joint_value_target
	# print("Number of group states in srdf file: %i \n" % len(group_state))
	# n = 1
	# while n < len(group_state): #Home configuration (i.e 0 position) is a singularity 
	# 		print("group state %i: %s \n" %(n,group.get_active_joints()))
	# 		print("Joint Values %s \n" % group.get_current_joint_values())
	# 		print("Joint Values %s \n\n" % group.get_named_target_values(group_state[n]))
	# 		trajectory_.append(group.get_named_target_values(group_state[n]))
	# 		n += 1
	# motionPlan.pipeline_id = "pilz_industrial_motion_planner"
	# motionPlan.planner_id = "PTP"
	# motionPlan.max_velocity_scaling_factor = 1
	# motionPlan.max_acceleration_scaling_factor = 1
	# motionPlan.group_name = "yaska3_DN1P"
	# client = SimpleActionClient("/plan_sequence_path")
	# sequence.goal.planning_options.planning_scene_diff.robot_state.multi_dof_joint_state = BuildTrajectory(group)
	# group.plan()

	# if not rospy.is_shutdown():

	# 	print("New target has been set")
	# 	rospy.sleep(1)
	# 	# If you want to move the group to the specified targets uncomment the lines below
	# 	print("Plannig done, now executing \n")
	# 	group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
	# 	#group.get_goal_tolerance() # Return a tuple of goal tolerances: joint, position and orientation
	# 	group.stop()
	# 	rospy.sleep(1)

