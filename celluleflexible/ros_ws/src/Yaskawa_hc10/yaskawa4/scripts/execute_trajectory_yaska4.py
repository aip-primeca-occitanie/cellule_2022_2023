#!/usr/bin/env python3
import sys
import rospy
import actionlib
import moveit_commander
import pyassimp
import re
import numpy as np
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg
from moveit_msgs.msg import MoveGroupSequenceActionResult, MoveGroupSequenceActionGoal, MoveGroupSequenceAction
from moveit_msgs.msg import MotionSequenceRequest, MotionSequenceItem, Constraints, JointConstraint, CollisionObject, AttachedCollisionObject
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from shape_msgs.msg import Mesh, MeshTriangle

#Variable globale
trajectoryErrorCode4 = 1 #Succes = 1 in error_code

#Pour orienter la navette dans l'environnement
def get_quaternion_from_euler(roll, pitch, yaw):
#   """
#   Convert an Euler angle to a quaternion.
   
#   Input
#     :param roll: The roll (rotation around x-axis) angle in radians.
#     :param pitch: The pitch (rotation around y-axis) angle in radians.
#     :param yaw: The yaw (rotation around z-axis) angle in radians.
 
#   Output
#     :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
#   """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]

# Fonction reprise de la fonction statique make_mesh() de planning_scene_interface.py
# Elle permet de charger un fichier stl sous forme de CollisionObject
def MakeCollisionObjectFromMeshFile(name, filename, scale=(1,1,1)):
	co = CollisionObject()
	if pyassimp is False:
		rospy.loginfo("Pyassimp needs patch https://launchpadlibrarian.net/319496602/patchPyassim.txt")
	scene = pyassimp.load(filename)
	if not scene.meshes or len(scene.meshes) == 0:
		rospy.loginfo("There are no meshes in the file")
	if len(scene.meshes[0].faces) == 0:
		rospy.loginfo("There are no faces in the mesh")
	co.operation = CollisionObject.ADD
	co.id = name
	mesh = Mesh()
	first_face = scene.meshes[0].faces[0]
	if hasattr(first_face, "__len__"):
		for face in scene.meshes[0].faces:
			if len(face) == 3:
				triangle = MeshTriangle()
				triangle.vertex_indices = [face[0], face[1], face[2]]
				mesh.triangles.append(triangle)
	elif hasattr(first_face, "indices"):
		for face in scene.meshes[0].faces:
			if len(face.indices) == 3:
				triangle = MeshTriangle()
				triangle.vertex_indices = [
					face.indices[0],
					face.indices[1],
					face.indices[2],
				]
				mesh.triangles.append(triangle)
	else:
		rospy.loginfo("Unable to build triangles from mesh due to mesh object structure")
	for vertex in scene.meshes[0].vertices:
		point = Point()
		point.x = vertex[0] * scale[0]
		point.y = vertex[1] * scale[1]
		point.z = vertex[2] * scale[2]
		mesh.vertices.append(point)
	co.meshes = [mesh]
	pyassimp.release(scene)
	return co

# Cette fonction définit la postion de la navette sur le rail en fonction du mouvement de pick and place effectué,
# et ajoute la navette dans l'environnement RViz
def AddShuttleToWorkspace(armgroup,ps,nav):
	aco = AttachedCollisionObject()
	pose = PoseStamped()
	pose.header.frame_id = "rail_milieu_link"
	if(re.findall('2',str(armgroup.get_name()))):
		quaternion = get_quaternion_from_euler(1.57079, 0. ,-1.57079)
		pose.pose.position = Point(x = 0.12 , y = 0.001 , z = 0.1)
		pose.pose.orientation = Quaternion(x= quaternion[0] , y= quaternion[1], z= quaternion[2], w= quaternion[3])		
	else:
		quaternion = get_quaternion_from_euler(1.57079, 0. ,-1.57079)
		pose.pose.position = Point(x = -0.29 , y = 0.001 , z = 0.1)
		pose.pose.orientation = Quaternion(x= quaternion[0] , y= quaternion[1], z= quaternion[2], w= quaternion[3])		
	nav.header = pose.header
	nav.mesh_poses = [pose.pose]
	rospy.loginfo("Adding shuttle to workspace ...")
	aco.object = nav
	aco.link_name = "rail_milieu_link"
	aco.touch_links = ["rail_milieu_link"]
	ps.attach_object(aco)
	rospy.loginfo("Shuttle successfully added !")

# Cette fonction permet d'enlever la navette de l'environnement RViz, est appelé à la fin de l'exécution d'un mouvement.
def RemoveShuttleFromWorkspace(ps):
	ps.remove_attached_object(link = "rail_milieu_link", name = "Navette")
	rospy.sleep(0.1)
	ps.remove_world_object(name = "Navette")
	rospy.loginfo("Removed shuttle from workspace !")

#Callback de fin de trajectoire 
#permet de vérifier que la trajectoire s'est bien effectuée 
#permet de créer des fichiers txt pour visualiser les trajectoires des joints en utilisant le script plot.py
def TrajectoryResultCallback(resultmsg):

	global trajectoryErrorCode4
	trajectoryErrorCode4 = resultmsg.result.response.error_code.val
	# accel = []
	# vel = []
	# pos = []
	# sec = []
	# nsec = []
	# for traj in resultmsg.result.response.planned_trajectories:
	# 	for points in traj.joint_trajectory.points:
	# 		if len(points.accelerations)==6:
	# 			pos.append(points.positions)
	# 			accel.append(points.accelerations)
	# 			vel.append(points.velocities)
	# 			sec.append(points.time_from_start.secs)
	# 			nsec.append(points.time_from_start.nsecs)
	# fichierpos = open("./src/Yaskawa_hc10/yaskawa4/scripts/pos.txt","a")
	# fichiervel = open("./src/Yaskawa_hc10/yaskawa4/scripts/vel.txt","a")
	# fichieracc = open("./src/Yaskawa_hc10/yaskawa4/scripts/acc.txt","a")
	# fichiertime = open("./src/Yaskawa_hc10/yaskawa4/scripts/time.txt","a")
	# for i in range(0,len(vel)):
	# 	fichieracc.write(f"{accel[i][0]} {accel[i][1]} {accel[i][2]} {accel[i][3]} {accel[i][4]} {accel[i][5]}\n")
	# 	fichiervel.write(f"{vel[i][0]} {vel[i][1]} {vel[i][2]} {vel[i][3]} {vel[i][4]} {vel[i][5]}\n")
	# 	fichierpos.write(f"{pos[i][0]} {pos[i][1]} {pos[i][2]} {pos[i][3]} {pos[i][4]} {pos[i][5]}\n")
	# 	fichiertime.write(f"{sec[i]} {nsec[i]}\n")
	# fichierpos.close()
	# fichiervel.close()
	# fichieracc.close()
	# fichiertime.close()

#Interface de gestion de l'erreur par l'utilisateur
def ErrorTrajectoryExecution():
	print("""What would you like to do? 
	1. New trajectory execution atempt
	2. Abort trajectory execution""")
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
		armgroup = moveit_commander.MoveGroupCommander("DN2P1", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 2:
		armgroup = moveit_commander.MoveGroupCommander("DN2P4", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 3:
		armgroup = moveit_commander.MoveGroupCommander("DN3P1", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 4:
		armgroup = moveit_commander.MoveGroupCommander("DN3P4", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 5:
		armgroup = moveit_commander.MoveGroupCommander("DP1N2", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 6:
		armgroup = moveit_commander.MoveGroupCommander("DP4N2", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 7:
		armgroup = moveit_commander.MoveGroupCommander("DP1N3", robot_description="/yaska4/robot_description", ns="/yaska4")
	elif pub_yaska4.data == 8:
		armgroup = moveit_commander.MoveGroupCommander("DP4N3", robot_description="/yaska4/robot_description", ns="/yaska4")
	else :
		rospy.loginfo("Error callback control yaska4 \n")
		return
	# on récupère le groupe associé à la pince du robot
	handgroup = moveit_commander.MoveGroupCommander("yaskawa4_hand", robot_description="/yaska4/robot_description", ns="/yaska4")
	rospy.loginfo(f"Deplacement is {armgroup.get_name()}")
	# Ajout de la navette dans l'environnement
	AddShuttleToWorkspace(armgroup,planingScene,navette)
	while(True):
		#Initialise la position du robot à "home" avec ompl
		initialize_ = MoveGroupSequenceActionGoal()
		initialize_.goal.request = Initialize(armgroup,handgroup)
		rospy.loginfo("Initiating ...")
		clientSequence_.send_goal(initialize_.goal)
		finished_before_timeout = clientSequence_.wait_for_result(rospy.Duration(30))
		#Si l'initialisation s'est bien passé, on commence le pick and place
		if(finished_before_timeout & (trajectoryErrorCode4 == 1)):
			rospy.loginfo("Initialization succeeded !")
			#construction de la séquence de point à envoyer
			# armgroup.set_start_state_to_current_state()
			goal_ = MoveGroupSequenceActionGoal()
			goal_.goal.request = BuildSequenceRequest(armgroup,handgroup)
			rospy.loginfo("Executing Trajectory ...")
			clientSequence_.send_goal(goal_.goal)
			finished_before_timeout = clientSequence_.wait_for_result(rospy.Duration(60))
			rospy.sleep(1)
			if(finished_before_timeout & (trajectoryErrorCode4 == 1)):
				rospy.loginfo("Trajectory completed !")
				rospy.loginfo(clientSequence_.get_goal_status_text())
				# print("Results: %s" %clientSequence_.action_client.ActionResult)
				#si tout s'est bien passé, on averti coppeliaSim de la fin d'execution du movement du robot
				mymsgYaska4.FinDeplacerR4 = 1
				rospy.loginfo(mymsgYaska4)
				pub_fintache.publish(mymsgYaska4)
				rospy.sleep(1)
				mymsgYaska4.FinDeplacerR4 = 0
				pub_fintache.publish(mymsgYaska4)
				RemoveShuttleFromWorkspace(planingScene)
				break
			else:
				rospy.loginfo("Error during trajectory execution")
				choice = ErrorTrajectoryExecution()
				if(choice == 2):
					rospy.loginfo("Aborting trajectory")
					mymsgYaska4.FinDeplacerR4 = 1
					rospy.loginfo(mymsgYaska4)
					pub_fintache.publish(mymsgYaska4)
					rospy.sleep(1)
					mymsgYaska4.FinDeplacerR4 = 0
					pub_fintache.publish(mymsgYaska4)
					RemoveShuttleFromWorkspace(planingScene)
					break
		#Si l'initialisation a échouée
		else:
			rospy.loginfo("Error during initialization")
			choice = ErrorTrajectoryExecution()
			if(choice == 2):
				rospy.loginfo("Aborting trajectory")
				mymsgYaska4.FinDeplacerR4 = 1
				rospy.loginfo(mymsgYaska4)
				pub_fintache.publish(mymsgYaska4)
				rospy.sleep(1)
				mymsgYaska4.FinDeplacerR4 = 0
				pub_fintache.publish(mymsgYaska4)
				RemoveShuttleFromWorkspace(planingScene)
				break

if __name__ == "__main__":
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('execute_trajectory_yaska4',anonymous=True)
	mymsgYaska4 = FinDeplacerPiece_Msg()
	rospy.Subscriber('/control_robot_yaska4',Int32, ControlCallback)
	# Pour ploter les trajectoires du robot
	rospy.Subscriber('/yaska4/sequence_move_group/result', MoveGroupSequenceActionResult, TrajectoryResultCallback)
	# Pour avertir Coppelia d'une fin de mouvement
	pub_fintache = rospy.Publisher("/commande/Simulation/finTache", FinDeplacerPiece_Msg,  queue_size=1)
	#création d'un client actionlib
	clientSequence_ = actionlib.SimpleActionClient('/yaska4/sequence_move_group', MoveGroupSequenceAction)
	#attente de connexion
	rospy.loginfo("En attente de connexion du client actionlib ...")
	clientSequence_.wait_for_server()
	rospy.loginfo("Client connecté !")
	#Scene de l'environnement Rviz
	rospy.loginfo("Initialisation de la scène ...")
	planingScene = moveit_commander.PlanningSceneInterface(ns= "/yaska4")
	#Création de la navette
	rospy.loginfo("Import de la navette ...")
	name = "Navette"
	scale = (0.1,0.1,0.1)
	filename = "src/Yaskawa_hc10/yaskawa4/meshes/env/Navette.stl"
	navette = MakeCollisionObjectFromMeshFile(name, filename, scale)
	rospy.loginfo("Navette chargée !")
	rospy.spin()
	moveit_commander.roscpp_shutdown()
