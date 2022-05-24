#!/usr/bin/env python3
import sys
import copy
import rospy
import actionlib
import moveit_commander
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg
from moveit_msgs.msg import MoveGroupSequenceActionResult, MoveGroupSequenceActionGoal, MoveGroupSequenceAction, MoveGroupActionResult
from moveit_msgs.msg import MotionSequenceRequest, MotionSequenceItem, Constraints, JointConstraint
from sensor_msgs.msg import JointState

trajectoryErrorCode4 = 1 #Succes = 1 in error_code

#Callback de fin de la trajectoire de pick and place
def TrajectoryResultCallback(errormsg):
	global trajectoryErrorCode4
	trajectoryErrorCode4 = errormsg.result.response.error_code.val

#Callback de fin de trajectoire d'initialisation
def InitializationCallback(errormsg):
	global trajectoryErrorCode4
	trajectoryErrorCode4 = errormsg.result.error_code.val

#Interface de gestion de l'erreur par l'utilisateur
def ErrorTrajectoryExecution():
	print("""What would you like to do? 
	1. Finish trajectory 
	2. Abort trajectory """)
	while(True):
		choice = input('Choice:')
		if(choice.isdigit() and int(choice) in range(1,3)):
			return int(choice)
		print("Invalid choice, try again")

#Cette fonction permet de faire revenir le robot en position home peut importe sa position actuelle en utilisant le planner ompl
def Initialize(group):
	print("Initiating ...")
	group.set_planning_pipeline_id("ompl")
	group.set_start_state_to_current_state()
	group.set_joint_value_target(group.get_named_target_values("home"))
	group.plan()
	rospy.sleep(1)
	group.go(wait=True)
	group.stop()
	rospy.sleep(1)

# cette fonction permet de construire un séquence de points dont le plannifieur pilz calculera la trajectoire
def BuildSequenceRequest(group):
	motionPlanItemInitialize = MotionSequenceItem()
	motionSequenceRequest = MotionSequenceRequest()
	constraints1_ = Constraints()	
	#le premier planning a besoin d'un start_state, les autres démarrent automatiquement au goal précédent
	motionPlanItemInitialize.req.pipeline_id = "pilz_industrial_motion_planner"
	# PTP, CIR, LIN
	motionPlanItemInitialize.req.planner_id = "PTP"
	motionPlanItemInitialize.req.max_velocity_scaling_factor = 1.0
	motionPlanItemInitialize.req.max_acceleration_scaling_factor = 1.0
	motionPlanItemInitialize.blend_radius = 0.0
	motionPlanItemInitialize.req.allowed_planning_time = 5.0
	motionPlanItemInitialize.req.group_name = group.get_name()
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
		constraints1_.joint_constraints.append(goalJointTemp_)
	motionPlanItemInitialize.req.goal_constraints.append(constraints1_)
	motionSequenceRequest.items.append(motionPlanItemInitialize)
	#on ajoute les autres items a la sequence
	for n in range(1,len(group_name)):
		motionPlanItem = MotionSequenceItem()
		constraints2_ = Constraints()
		motionPlanItem.req.pipeline_id = "pilz_industrial_motion_planner"
		motionPlanItem.req.planner_id = "PTP"
		motionPlanItem.req.max_velocity_scaling_factor = 1.0
		motionPlanItem.req.max_acceleration_scaling_factor = 1.0
		motionPlanItem.blend_radius = 0.0
		motionPlanItem.req.allowed_planning_time = 5.0
		motionPlanItem.req.group_name = group.get_name()
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

#Callback de DeplacerPiece si mode rviz ou atelier
def ControlCallback(pub_yaska4):
	#création d'un client actionlib
	clientSequence_ = actionlib.SimpleActionClient('/yaska4/sequence_move_group', MoveGroupSequenceAction)
	#attente de connexion
	clientSequence_.wait_for_server()
	# pub_motionSequenceRequest.publish(goal_)
	#on récupère la trajectoire a réaliser définie dans les groupe du fichier config/.srdf
	if pub_yaska4.data == 1:
		group = moveit_commander.MoveGroupCommander("DN1P", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 2:
		group = moveit_commander.MoveGroupCommander("DN2P", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 3:
		group = moveit_commander.MoveGroupCommander("DPN1", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 4:
		group = moveit_commander.MoveGroupCommander("DPN2", robot_description="/yaska4/robot_description", ns="/yaska4")
	else :
		print("Error callback control yaska4 \n")
		return
	print(f"Deplacement is {group.get_name()}")
	while(True):
		# Initialise la position du robot à "home"
		Initialize(group)
		#Si l'initialisation s'est bien passé, on commence le pick and place
		if((trajectoryErrorCode4 == 1)):
			#construction de la séquence de point à envoyer
			group.set_start_state_to_current_state()
			goal_ = MoveGroupSequenceActionGoal()
			goal_.goal.request = BuildSequenceRequest(group)
			clientSequence_.send_goal(goal_.goal)
			finished_before_timeout = clientSequence_.wait_for_result(rospy.Duration(30))
			rospy.sleep(1)
			if(finished_before_timeout & (trajectoryErrorCode4 == 1)):
				rospy.loginfo(clientSequence_.get_goal_status_text())
				# print("Results: %s" %client_.action_client.ActionResult)
				#si tout s'est bien passé, on averti coppeliaSim de la fin d'execution du movement du robot
				mymsgYaska4.FinDeplacerR4 = 1
				rospy.loginfo(mymsgYaska4)
				pub_fintache.publish(mymsgYaska4)
				rospy.sleep(1)
				mymsgYaska4.FinDeplacerR4 = 0
				pub_fintache.publish(mymsgYaska4)
				break
			else:
				print("Error during trajectory execution")
				choice = ErrorTrajectoryExecution()
				if(choice == 2):
					print("Aborting trajectory")
					mymsgYaska4.FinDeplacerR4 = 1
					rospy.loginfo(mymsgYaska4)
					pub_fintache.publish(mymsgYaska4)
					rospy.sleep(1)
					mymsgYaska4.FinDeplacerR4 = 0
					pub_fintache.publish(mymsgYaska4)
					break
		else:
			print("Error during initialization")
			choice = ErrorTrajectoryExecution()
			if(choice == 2):
				print("Aborting trajectory")
				mymsgYaska4.FinDeplacerR4 = 1
				rospy.loginfo(mymsgYaska4)
				pub_fintache.publish(mymsgYaska4)
				rospy.sleep(1)
				mymsgYaska4.FinDeplacerR4 = 0
				pub_fintache.publish(mymsgYaska4)
				break

if __name__ == "__main__":
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('execute_trajectory_yaska4',anonymous=True)
	mymsgYaska4 = FinDeplacerPiece_Msg()
	rospy.Subscriber('/control_robot_yaska4',Int32, ControlCallback)
	rospy.Subscriber('/yaska4/sequence_move_group/result', MoveGroupSequenceActionResult, TrajectoryResultCallback)
	rospy.Subscriber('/yaska4/move_group/result', MoveGroupActionResult, InitializationCallback)
	pub_fintache = rospy.Publisher("/commande/Simulation/finTache", FinDeplacerPiece_Msg,  queue_size=1)
	# pub_motionSequenceRequest = rospy.Publisher( "/sequence_move_group/goal", MoveGroupSequenceActionGoal, queue_size=1)
	rospy.spin()
	moveit_commander.roscpp_shutdown()
