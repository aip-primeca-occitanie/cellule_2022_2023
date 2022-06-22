#!/usr/bin/env python3
import sys
import rospy
import actionlib
import moveit_commander
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg
from moveit_msgs.msg import MoveGroupSequenceActionResult, MoveGroupSequenceActionGoal, MoveGroupSequenceAction, DisplayTrajectory
from moveit_msgs.msg import MotionSequenceRequest, MotionSequenceItem, Constraints, JointConstraint, PositionConstraint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


_POS_TOLERANCE_ = 1e-3

def DisplayTrajectoryCallback(traj):
	accel = []
	vel = []
	pos = []
	for points in traj.trajectory[0].joint_trajectory.points:
		if len(points.accelerations)==6:
			pos.append(points.positions)
			accel.append(points.accelerations)
			vel.append(points.velocities)
	fichierpos = open("./src/Yaskawa_hc10/yaskawa4/scripts/pos.txt","a")
	fichiervel = open("./src/Yaskawa_hc10/yaskawa4/scripts/vel.txt","a")
	fichieracc = open("./src/Yaskawa_hc10/yaskawa4/scripts/acc.txt","a")
	for i in range(0,len(vel)):
		fichieracc.write(f"{accel[i][0]} {accel[i][1]} {accel[i][2]} {accel[i][3]} {accel[i][4]} {accel[i][5]}\n")
		fichiervel.write(f"{vel[i][0]} {vel[i][1]} {vel[i][2]} {vel[i][3]} {vel[i][4]} {vel[i][5]}\n")
		fichierpos.write(f"{pos[i][0]} {pos[i][1]} {pos[i][2]} {pos[i][3]} {pos[i][4]} {pos[i][5]}\n")
	fichierpos.close()
	fichiervel.close()
	fichieracc.close()

trajectoryErrorCode4 = 1 #Succes = 1 in error_code

#Callback de fin de la trajectoire de pick and place
def TrajectoryResultCallback(errormsg):
	global trajectoryErrorCode4
	trajectoryErrorCode4 = errormsg.result.response.error_code.val

#Callback de fin de trajectoire d'initialisation
# def InitializationCallback(errormsg):
# 	global trajectoryErrorCode4
# 	trajectoryErrorCode4 = errormsg.result.error_code.val

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
# cette fonction permet de construire un séquence de points dont le plannifieur pilz calculera la trajectoire
def BuildSequenceRequest(armgroup):
	motionPlanItemInitialize_ = MotionSequenceItem()
	motionSequenceRequest_ = MotionSequenceRequest()
	constraints1_ = Constraints()	
	#le premier planning a besoin d'un start_state, les autres démarrent automatiquement au goal précédent
	motionPlanItemInitialize_.req.pipeline_id = "pilz_industrial_motion_planner"
	motionPlanItemInitialize_.req.planner_id = "LIN"
	motionPlanItemInitialize_.req.max_velocity_scaling_factor = 1.0
	motionPlanItemInitialize_.req.max_acceleration_scaling_factor = 0.2
	motionPlanItemInitialize_.blend_radius = 0.0
	motionPlanItemInitialize_.req.allowed_planning_time = 5.0
	motionPlanItemInitialize_.req.group_name = armgroup.get_name()
	#initialise start_state à la position dans laquelle se trouve le robot
	# jointName_ = armgroup.get_active_joints()
	# jointPosition_ = armgroup.get_current_joint_values()
	# motionPlanItemInitialize_.req.start_state.joint_state = JointState(name=jointName_,position=jointPosition_)
	#on récupère le premier goal
	#récupère le nom des différentes positions définie dans le groupe du fichier srdf
	#on récupère les noms et valeurs des joints de la première postion du groupe sous forme de dictionnaire
	#on ajoute notre premier item a la sequence request
	# goalJointTemp_ = OrientationConstraint()
	goalJointTemp_ = Pose()
	goalRegionTemp_ = SolidPrimitive()
	goalConstraintTemp_ = PositionConstraint()
	goalJointTemp_.orientation.x = 0.0009
	goalJointTemp_.orientation.y = -0.999828
	goalJointTemp_.orientation.z = 0.0172717
	goalJointTemp_.orientation.w = 0.00658933
	# goalJointTemp_.header.frame_id = "world"
	# goalJointTemp_.link_name = "cylinder_link"
	goalJointTemp_.position.x = -0.718883
	goalJointTemp_.position.y = 0.110829
	goalJointTemp_.position.z = 1.742208
	# goalConstraintTemp_.target_point_offset.x = -0.718883
	# goalConstraintTemp_.target_point_offset.y = 0.110829
	# goalConstraintTemp_.target_point_offset.z = 1.742208
	goalConstraintTemp_.header.frame_id = "world"
	goalConstraintTemp_.link_name = "cylinder_link"
	goalConstraintTemp_.weight = 1.0
	goalRegionTemp_.type = SolidPrimitive.SPHERE
	goalRegionTemp_.dimensions.append(_POS_TOLERANCE_)
	goalConstraintTemp_.constraint_region.primitives.append(goalRegionTemp_)
	goalConstraintTemp_.constraint_region.primitive_poses.append(goalJointTemp_)
	constraints1_.position_constraints.append(goalConstraintTemp_)
	# constraints1_.orientation_constraints.append(goalJointTemp_)
	motionPlanItemInitialize_.req.goal_constraints.append(constraints1_)
	motionSequenceRequest_.items.append(motionPlanItemInitialize_)
	# #on ajoute les autres items a la sequence	
	motionPlanItem_ = MotionSequenceItem()
	constraints2_ = Constraints()
	motionPlanItem_.req.pipeline_id = "pilz_industrial_motion_planner"
	motionPlanItem_.req.planner_id = "LIN"
	motionPlanItem_.req.max_velocity_scaling_factor = 1.0
	motionPlanItem_.req.max_acceleration_scaling_factor = 0.2
	motionPlanItem_.req.allowed_planning_time = 5.0
	motionPlanItem_.blend_radius = 0.0
	motionPlanItem_.req.group_name = armgroup.get_name()
	#on récupère le goal
	# goalJointTemp1_ = OrientationConstraint()
	goalJointTemp1_ = Pose()
	goalRegionTemp1_ = SolidPrimitive()
	goalConstraintTemp1_ = PositionConstraint()
	goalJointTemp1_.orientation.x = 0.008156
	goalJointTemp1_.orientation.y = 0.999663
	goalJointTemp1_.orientation.z = -0.022593
	goalJointTemp1_.orientation.w = 0.009843
	# goalJointTemp1_.header.frame_id = "world"
	# goalJointTemp1_.link_name = "cylinder_link"
	goalJointTemp1_.position.x = -0.7221004
	goalJointTemp1_.position.y = 0.112325
	goalJointTemp1_.position.z = 1.462857
	# goalConstraintTemp1_.target_point_offset = Vector3(x= -0.7221004, y = 0.112325, z = 1.462857)
	goalConstraintTemp1_.header.frame_id = "world"
	goalConstraintTemp1_.link_name = "cylinder_link"
	goalConstraintTemp1_.weight = 1.0
	goalRegionTemp1_.type = SolidPrimitive.SPHERE
	goalRegionTemp1_.dimensions.append(_POS_TOLERANCE_)
	goalConstraintTemp1_.constraint_region.primitives.append(goalRegionTemp1_)
	goalConstraintTemp1_.constraint_region.primitive_poses.append(goalJointTemp1_)
	constraints2_.position_constraints.append(goalConstraintTemp1_)
	# constraints2_.orientation_constraints.append(goalJointTemp1_)
	#on ajoute notre item a la sequence request
	motionPlanItem_.req.goal_constraints.append(constraints2_)
	motionSequenceRequest_.items.append(motionPlanItem_)
	# la ligne ci-dessous permet de tester si les types de message que l'on ajoute à la séquence sont bons
	return motionSequenceRequest_

#Callback de DeplacerPiece si mode rviz ou atelier
def ControlCallback(pub_yaska4):
	#création d'un client actionlib
	clientSequence_ = actionlib.SimpleActionClient('/yaska4/sequence_move_group', MoveGroupSequenceAction)
	#attente de connexion
	clientSequence_.wait_for_server()
	# pub_motionSequenceRequest.publish(goal_)
	#on récupère la trajectoire a réaliser définie dans les groupes du fichier config/.srdf
	if pub_yaska4.data == 1:
		armgroup = moveit_commander.MoveGroupCommander("DN1P", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 2:
		armgroup = moveit_commander.MoveGroupCommander("DN2P", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 3:
		armgroup = moveit_commander.MoveGroupCommander("DPN1", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 4:
		armgroup = moveit_commander.MoveGroupCommander("DPN2", robot_description="/yaska4/robot_description", ns="/yaska4")
	else :
		print("Error callback control yaska4 \n")
		return
	# on récupère le groupe associé à la pince du robot
	handgroup = moveit_commander.MoveGroupCommander("yaskawa4_hand", robot_description="/yaska4/robot_description", ns="/yaska4")
	print(f"Deplacement is {armgroup.get_name()}")
	while(True):
		# Initialise la position du robot à "home"
		initialize_ = MoveGroupSequenceActionGoal()
		initialize_.goal.request = BuildSequenceRequest(armgroup)
		print("Initiating ...")
		clientSequence_.send_goal(initialize_.goal)
		finished_before_timeout = clientSequence_.wait_for_result(rospy.Duration(1))
		#Si l'initialisation s'est bien passé, on commence le pick and place
		if(finished_before_timeout & (trajectoryErrorCode4 == 1)):
			print("Initialization succeeded !")
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
	pub_fintache = rospy.Publisher("/commande/Simulation/finTache", FinDeplacerPiece_Msg,  queue_size=1)
	# rospy.Subscriber('/yaska4/move_group/result', MoveGroupActionResult, InitializationCallback)
	# pub_motionSequenceRequest = rospy.Publisher( "/sequence_move_group/goal", MoveGroupSequenceActionGoal, queue_size=1)
	rospy.Subscriber('yaska4/move_group/display_planned_path', DisplayTrajectory, DisplayTrajectoryCallback)
	# group = moveit_commander.MoveGroupCommander("DN1P", robot_description="/yaska4/robot_description", ns="/yaska4")
	# while(True):
	# 	print(group.get_current_pose())
	rospy.spin()
	moveit_commander.roscpp_shutdown()
