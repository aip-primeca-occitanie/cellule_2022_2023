#!/usr/bin/env python3
import sys
import rospy
import actionlib
import moveit_commander
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg
from moveit_msgs.msg import MoveGroupSequenceActionResult, MoveGroupSequenceActionGoal, MoveGroupSequenceAction, DisplayTrajectory
from moveit_msgs.msg import MotionSequenceRequest, MotionSequenceItem, Constraints, JointConstraint, MotionSequenceResponse
from sensor_msgs.msg import JointState

trajectoryErrorCode4 = 1 #Succes = 1 in error_code

#Callback de fin de trajectoire 
#permet de vérifier que la trajectoire s'est bien effectuée 
#permet de créer des fichiers txt pour visualiser les trajectoires des joints en utilisant le script plot.py
def TrajectoryResultCallback(resultmsg):
	accel = []
	vel = []
	pos = []
	sec = []
	nsec = []
	global trajectoryErrorCode4
	trajectoryErrorCode4 = resultmsg.result.response.error_code.val
	for traj in resultmsg.result.response.planned_trajectories:
		for points in traj.joint_trajectory.points:
			if len(points.accelerations)==6:
				pos.append(points.positions)
				accel.append(points.accelerations)
				vel.append(points.velocities)
				sec.append(points.time_from_start.secs)
				nsec.append(points.time_from_start.nsecs)
	fichierpos = open("./src/Yaskawa_hc10/yaskawa4/scripts/pos.txt","a")
	fichiervel = open("./src/Yaskawa_hc10/yaskawa4/scripts/vel.txt","a")
	fichieracc = open("./src/Yaskawa_hc10/yaskawa4/scripts/acc.txt","a")
	fichiertime = open("./src/Yaskawa_hc10/yaskawa4/scripts/time.txt","a")
	for i in range(0,len(vel)):
		fichieracc.write(f"{accel[i][0]} {accel[i][1]} {accel[i][2]} {accel[i][3]} {accel[i][4]} {accel[i][5]}\n")
		fichiervel.write(f"{vel[i][0]} {vel[i][1]} {vel[i][2]} {vel[i][3]} {vel[i][4]} {vel[i][5]}\n")
		fichierpos.write(f"{pos[i][0]} {pos[i][1]} {pos[i][2]} {pos[i][3]} {pos[i][4]} {pos[i][5]}\n")
		fichiertime.write(f"{sec[i]} {nsec[i]}\n")
	fichierpos.close()
	fichiervel.close()
	fichieracc.close()
	fichiertime.close()

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
def Initialize(armgroup,handgroup):
	motionSequenceRequest_ = MotionSequenceRequest()
	motionPlanItemInitialize_ = MotionSequenceItem()
	constraints_ = Constraints()	
	#le premier planning a besoin d'un start_state, les autres démarrent automatiquement au goal précédent
	motionPlanItemInitialize_.req.pipeline_id = "ompl"
	motionPlanItemInitialize_.req.max_velocity_scaling_factor = 1.0
	motionPlanItemInitialize_.blend_radius = 0.0
	motionPlanItemInitialize_.req.allowed_planning_time = 5.0
	motionPlanItemInitialize_.req.group_name = armgroup.get_name()
	#initialise start_state à la position dans laquelle se trouve le robot
	jointName_ = armgroup.get_active_joints()
	jointPosition_ = armgroup.get_current_joint_values()
	motionPlanItemInitialize_.req.start_state.joint_state = JointState(name=jointName_,position=jointPosition_)
	#on récupère le goal
	dict_ = armgroup.get_named_target_values("home")
	for i in range(0,len(jointName_)):
		goalJointTemp_ = JointConstraint()
		goalJointTemp_.joint_name = jointName_[i]
		goalJointTemp_.position = dict_.get(jointName_[i])
		goalJointTemp_.tolerance_above = 1e-6
		goalJointTemp_.tolerance_below = 1e-6
		goalJointTemp_.weight = 1.0
		constraints_.joint_constraints.append(goalJointTemp_)
	#on ajoute notre premier item a la sequence request
	motionPlanItemInitialize_.req.goal_constraints.append(constraints_)
	motionSequenceRequest_.items.append(motionPlanItemInitialize_)
	motionPlanItemInitializeEof_ = MotionSequenceItem()
	constraints2_ = Constraints()
	motionPlanItemInitializeEof_.req.pipeline_id = "ompl"
	motionPlanItemInitializeEof_.req.max_velocity_scaling_factor = 1.0
	motionPlanItemInitializeEof_.blend_radius = 0.0
	motionPlanItemInitializeEof_.req.allowed_planning_time = 5.0
	motionPlanItemInitializeEof_.req.group_name = handgroup.get_name()
	#initialise start_state à la position dans laquelle se trouve la pince
	jointName_ = handgroup.get_active_joints()
	jointPosition_ = handgroup.get_current_joint_values()
	motionPlanItemInitializeEof_.req.start_state.joint_state = JointState(name=jointName_,position=jointPosition_)
	#on récupère le goal
	dict_ = handgroup.get_named_target_values("Open")
	for i in range(0,len(jointName_)):
		goalJointTemp_ = JointConstraint()
		goalJointTemp_.joint_name = jointName_[i]
		goalJointTemp_.position = dict_.get(jointName_[i])
		goalJointTemp_.tolerance_above = 1e-6
		goalJointTemp_.tolerance_below = 1e-6
		goalJointTemp_.weight = 1.0
		constraints2_.joint_constraints.append(goalJointTemp_)
	#on ajoute notre second item a la sequence request
	motionPlanItemInitializeEof_.req.goal_constraints.append(constraints2_)
	motionSequenceRequest_.items.append(motionPlanItemInitializeEof_)
	return motionSequenceRequest_


# cette fonction permet de construire un séquence de points dont le plannifieur pilz calculera la trajectoire
def BuildSequenceRequest(armgroup,handgroup):
	motionSequenceRequest_ = MotionSequenceRequest()
	armgroup_name = armgroup.get_named_targets()
	isEofClosed = True
	iter = 0
	while(iter<len(armgroup_name)):
		motionPlanItem_ = MotionSequenceItem()
		constraints1_ = Constraints()
		# motionPlanItem_.req.pipeline_id = "ompl"
		motionPlanItem_.req.pipeline_id = "pilz_industrial_motion_planner"
		motionPlanItem_.req.planner_id = "PTP"
		motionPlanItem_.req.max_velocity_scaling_factor = 1.0
		motionPlanItem_.req.max_acceleration_scaling_factor = 0.2
		motionPlanItem_.req.allowed_planning_time = 5.0
		#blend radius doit être égal à 0 pour le dernier point ainsi que lorsqu'on change de planning group
		if(iter == 0 or iter == 2 or iter == 3 or iter == 4 or iter == 6):
			motionPlanItem_.blend_radius = 0.15
		else:
			motionPlanItem_.blend_radius = 0.0
		motionPlanItem_.req.group_name = armgroup.get_name()
		jointName_ = armgroup.get_active_joints()
		#on récupère le goal
		dict_ = armgroup.get_named_target_values(armgroup_name[iter])
		for i in range(0,len(jointName_)):
			goalJointTemp_ = JointConstraint()
			goalJointTemp_.joint_name = jointName_[i]
			goalJointTemp_.position = dict_.get(jointName_[i])
			goalJointTemp_.tolerance_above = 1e-6
			goalJointTemp_.tolerance_below = 1e-6
			goalJointTemp_.weight = 1.0
			constraints1_.joint_constraints.append(goalJointTemp_)
		#on ajoute notre item du robot a la sequence request
		motionPlanItem_.req.goal_constraints.append(constraints1_)
		motionSequenceRequest_.items.append(motionPlanItem_)
		#on est au dessus d'une navette ou d'un poste de travail on récupère une instuction pince
		if((armgroup_name[iter][:2] == 'On')):
			motionPlanItemEof_ = MotionSequenceItem()
			constraints2_ = Constraints()
			# motionPlanItem_.req.pipeline_id = "ompl"
			motionPlanItemEof_.req.pipeline_id = "pilz_industrial_motion_planner"
			motionPlanItemEof_.req.planner_id = "PTP"
			motionPlanItemEof_.req.max_velocity_scaling_factor = 1.0
			motionPlanItemEof_.req.max_acceleration_scaling_factor = 0.2
			motionPlanItemEof_.req.allowed_planning_time = 5.0
			jointName_ = handgroup.get_active_joints()
			# jointPosition_ = handgroup.get_current_joint_values()
			if isEofClosed:
				dict_ = handgroup.get_named_target_values("Close")
			else:
				dict_ = handgroup.get_named_target_values("Open")
			motionPlanItemEof_.blend_radius = 0.0
			motionPlanItemEof_.req.group_name = handgroup.get_name()
			# motionPlanItem_.req.start_state.joint_state = JointState(name=jointName_,position=jointPosition_)
			#on récupère le goal
			for i in range(0,len(jointName_)):
				goalJointTemp_ = JointConstraint()
				goalJointTemp_.joint_name = jointName_[i]
				goalJointTemp_.position = dict_.get(jointName_[i])
				goalJointTemp_.tolerance_above = 1e-6
				goalJointTemp_.tolerance_below = 1e-6
				goalJointTemp_.weight = 1.0
				constraints2_.joint_constraints.append(goalJointTemp_)
			#on ajoute notre item de pince a la sequence request
			motionPlanItemEof_.req.goal_constraints.append(constraints2_)
			motionSequenceRequest_.items.append(motionPlanItemEof_)
			isEofClosed = not isEofClosed
		iter = iter+1
	# la ligne ci-dessous permet de tester si les types de message que l'on ajoute à la séquence sont bons
	# motionSequenceRequest._check_types()
	return motionSequenceRequest_

#Callback de DeplacerPiece si mode rviz ou atelier
def ControlCallback(pub_yaska4):
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
		#Initialise la position du robot à "home" avec ompl
		initialize_ = MoveGroupSequenceActionGoal()
		initialize_.goal.request = Initialize(armgroup,handgroup)
		print("Initiating ...")
		clientSequence_.send_goal(initialize_.goal)
		finished_before_timeout = clientSequence_.wait_for_result(rospy.Duration(30))
		#Si l'initialisation s'est bien passé, on commence le pick and place
		if(finished_before_timeout & (trajectoryErrorCode4 == 1)):
			print("Initialization succeeded !")
			#construction de la séquence de point à envoyer
			# armgroup.set_start_state_to_current_state()
			goal_ = MoveGroupSequenceActionGoal()
			goal_.goal.request = BuildSequenceRequest(armgroup,handgroup)
			print("Executing Trajectory ...")
			clientSequence_.send_goal(goal_.goal)
			finished_before_timeout = clientSequence_.wait_for_result(rospy.Duration(1))
			rospy.sleep(1)
			if(finished_before_timeout & (trajectoryErrorCode4 == 1)):
				print("Trajectory completed !")
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
		#Si l'initialisation a échouée
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
	#création d'un client actionlib
	clientSequence_ = actionlib.SimpleActionClient('/yaska4/sequence_move_group', MoveGroupSequenceAction)
	#attente de connexion
	clientSequence_.wait_for_server()
	rospy.spin()
	moveit_commander.roscpp_shutdown()
