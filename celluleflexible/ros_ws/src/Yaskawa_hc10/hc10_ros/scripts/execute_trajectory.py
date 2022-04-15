#!/usr/bin/env python3
from os import wait
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Int32
from robots.msg import FinDeplacerPiece_Msg


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('execute_trajectory',anonymous=True)
#Misc variables
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
mymsg = FinDeplacerPiece_Msg()
def callback(pub_yaku):
	if pub_yaku.data == 1:
		group = moveit_commander.MoveGroupCommander("DN1P")
	elif pub_yaku.data == 2:
		group = moveit_commander.MoveGroupCommander("DN2P")
	elif pub_yaku.data == 3:
		group = moveit_commander.MoveGroupCommander("DPN1")
	elif pub_yaku.data == 4:
		group = moveit_commander.MoveGroupCommander("DPN2")
	else :
		print("Error callback control yakuza \n")
		return
	print ("Value of deplacement %d" %pub_yaku.data)
	#Choose your planning pipeline
	group.set_planning_pipeline_id("pilz_industrial_motion_planner")
	#Set id planner
	group.set_planner_id("PTP")
	#Uncomment the line below if you want to set the velocity of robot manually (range 0 - 1 )
	#group.set_max_velocity_scaling_factor(0.7)
	#Group states taken from the srdf file
	group_state = group.get_named_targets()
	#Planning and executing with set_joint_value_target
	print("Number of group states in srdf file: %i \n" % len(group_state))
	for n in range(0,len(group_state)): #Home configuration (i.e 0 position) is a singularity 
		if not rospy.is_shutdown():
			print("group state %i: %s" %(n,group_state[n]))
			print("Joint Values %s" %group.get_named_target_values(group_state[n]))
			group.set_joint_value_target(group.get_named_target_values(group_state[n]))
			print("New target has been set")
			plan2 = group.plan()
			rospy.sleep(1)
			# If you want to move the group to the specified targets uncomment the lines below
			print("Plannig done, now executing \n")
			group.go(wait=True) #Blocking call, same as "group.move()" for roscpp
			group.stop()
			rospy.sleep(1)
	mymsg.FinDeplacerR4 = 1
	rospy.loginfo(mymsg)
	pub_fintache.publish(mymsg)
	rospy.sleep(1)
	mymsg.FinDeplacerR4 = 0
	pub_fintache.publish(mymsg)

if __name__ == "__main__":
	rospy.Subscriber('/control_yakuza',Int32, callback)
	# display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
    #                                            moveit_msgs.msg.DisplayTrajectory,
    #                                            queue_size=20)
	pub_fintache = rospy.Publisher("/commande/Simulation/finTache", FinDeplacerPiece_Msg,  queue_size=1)
	rospy.spin()
	

moveit_commander.roscpp_shutdown()



