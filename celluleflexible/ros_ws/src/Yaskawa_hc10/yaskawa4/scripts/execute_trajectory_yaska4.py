#!/usr/bin/env python3
from ntpath import join
from os import wait
import sys
import copy
import rospy
import actionlib
import moveit_commander
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg
from moveit_msgs.msg import MoveGroupSequenceActionResult, MotionSequenceRequest, MotionSequenceItem, Constraints, JointConstraint, MoveGroupSequenceActionGoal, MoveGroupSequenceAction
from sensor_msgs.msg import JointState

trajectoryErrorCode4 = 1 #Succes = 1 in error_code

def TrajectoryResultCallback(errormsg):
	global trajectoryErrorCode4
	trajectoryErrorCode4 = errormsg.result.response.error_code.val

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
		if((choice!=1) & (choice !=2)):
			print("Invalid choice, try again")
	return choice

# cette fonction permet de construire un séquence de points dont pilz plannifiera la trajectoire
def BuildSequenceRequest(pub_yaska4):
	motionPlanItemInitialize = MotionSequenceItem()
	motionSequenceRequest = MotionSequenceRequest()
	constraints_ = Constraints()
	#on récupère la trajectoire a réaliser définie dans les groupe du fichier config/.srdf
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
	#le premier planning a besoin d'un start_state, les autres démarrent automatiquement au goal précédent
	motionPlanItemInitialize.req.pipeline_id = "pilz_industrial_motion_planner"
	motionPlanItemInitialize.req.planner_id = "PTP"
	motionPlanItemInitialize.req.max_velocity_scaling_factor = 1
	motionPlanItemInitialize.req.max_acceleration_scaling_factor = 1
	motionPlanItemInitialize.blend_radius = 0
	#initialise start_state à la position dans laquelle se trouve le robot
	jointName_ = group.get_active_joints()
	jointPosition_ = group.get_current_joint_values()
	motionPlanItemInitialize.req.start_state.joint_state = JointState(name=jointName_,position=jointPosition_)
	#on récupère le premier goal
	#récupère le nom des différentes positions définie dans le groupe du fichier srdf
	group_name = group.get_named_targets()
	#on récupère les noms et valeurs des joints de la première postion du groupe sous forme de dictionnaire
	dict_ = group.get_named_target_values(group_name[0])
	#on ajoute notre premier item a la sequence request
	for i in range(0,len(jointName_)):
		goalJointTemp_ = JointConstraint()
		goalJointTemp_.joint_name = jointName_[i]
		goalJointTemp_.position = dict_.get(jointName_[i])
		constraints_.joint_constraints.append(goalJointTemp_)

	motionPlanItemInitialize.req.goal_constraints.append(constraints_)
	motionSequenceRequest.items.append(motionPlanItemInitialize)

	#on ajoute les autres items a la sequence
	for n in range(1,len(group_name)):
		motionPlanItem = MotionSequenceItem()
		constraints2_ = Constraints()
		motionPlanItem.req.pipeline_id = "pilz_industrial_motion_planner"
		motionPlanItem.req.planner_id = "PTP"
		motionPlanItem.req.max_velocity_scaling_factor = 1
		motionPlanItem.req.max_acceleration_scaling_factor = 1
		motionPlanItem.blend_radius = 0
		motionPlanItem.req.group_name = motionPlanItemInitialize.req.group_name
		dict_ = group.get_named_target_values(group_name[n])
		for i in range(0,len(jointName_)):
			goalJointTemp_ = JointConstraint()
			goalJointTemp_.joint_name = jointName_[i]
			goalJointTemp_.position = dict_.get(jointName_[i])
			constraints2_.joint_constraints.append(goalJointTemp_)
		motionPlanItem.req.goal_constraints.append(constraints2_)
		motionSequenceRequest.items.append(copy.deepcopy(motionPlanItem))
	# la ligne ci-dessous permet de tester si les types de message que l'on ajoute à la séquence sont bons
	# motionSequenceRequest._check_types()
	return motionSequenceRequest

def ControlCallback(pub_yaska4):
	#création d'un client actionlib
	client_ = actionlib.SimpleActionClient('/yaska4/sequence_move_group', MoveGroupSequenceAction)
	#attente de connexion
	client_.wait_for_server()
	#construction du message a envoyer
	goal_ = MoveGroupSequenceActionGoal()
	myPlan_ = BuildSequenceRequest(pub_yaska4)
	goal_.goal.request = myPlan_
	# pub_motionSequenceRequest.publish(goal_)
	while(True):
		print("Executing trajectory :")
		client_.send_goal(goal_.goal)
		finished_before_timeout = client_.wait_for_result(rospy.Duration(30))
		rospy.sleep(1)
		if(finished_before_timeout & (trajectoryErrorCode4 == 1)):
			rospy.loginfo(client_.get_goal_status_text())
			# print("Results: %s" %client_.action_client.ActionResult)
			#si tout s'est bien passé, on averti coppeliaSim de la fin d'execution du movement du robot
			mymsgYaska4.FinDeplacerR4 = 1
			rospy.loginfo(mymsgYaska4)
			pub_fintache.publish(mymsgYaska4)
			rospy.sleep(1)
			mymsgYaska4.FinDeplacerR3 = 0
			pub_fintache.publish(mymsgYaska4)
			break
		else: 
			choice = ErrorTrajectoryExecution()
			if(choice == 2): 
				print("Aborting trajectory")
				mymsgYaska4.FinDeplacerR4 = 1
				rospy.loginfo(mymsgYaska4)
				pub_fintache.publish(mymsgYaska4)
				rospy.sleep(1)
				mymsgYaska4.FinDeplacerR3 = 0
				pub_fintache.publish(mymsgYaska4)
				break
		
		

if __name__ == "__main__":
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('execute_trajectory_yaska4',anonymous=True)
	mymsgYaska4 = FinDeplacerPiece_Msg()
	rospy.Subscriber('/control_robot_yaska4',Int32, ControlCallback)
	rospy.Subscriber('/yaska4/sequence_move_group/result', MoveGroupSequenceActionResult, TrajectoryResultCallback)
	pub_fintache = rospy.Publisher("/commande/Simulation/finTache", FinDeplacerPiece_Msg,  queue_size=1)
	# pub_motionSequenceRequest = rospy.Publisher( "/sequence_move_group/goal", MoveGroupSequenceActionGoal, queue_size=1)
	rospy.spin()	
	moveit_commander.roscpp_shutdown()


