#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import roslib; roslib.load_manifest('motoman_driver')

class MoveGroupPythonIntefaceTutorial(object):
  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('display_interferences')

    # Misc variables
    self.group = moveit_commander.MoveGroupCommander('DN1P')
    self.box_name = ''
    self.robot = moveit_commander.RobotCommander()
    self.scene = moveit_commander.PlanningSceneInterface()

  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    scene = self.scene
    box_name = self.box_name
    start = rospy.get_time()
    seconds = rospy.get_time()
    
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False

  def add_interferences(self, timeout=4):
    rospy.sleep(1)
    group = self.group
    scene = self.scene
    robot = self.robot
    # xBloc = [[-350, -200], [-990, 1150], [-990, 1150], [200, 1150], [-990, -200], [-990, 1150], [900, 1150], [1150, 1160], [-1000, -990], [-990, 1150]]
    # yBloc = [[-150, 150], [-860, 1530], [450, 1530], [-860, 1530], [-860, 1530], [-860, -200], [50, 270], [-860, 1530], [-860, 1530], [-870, -860]]
    # zBloc = [[-500, -150], [-500, 1530], [-500, 320], [-500, -260], [-500, -260], [-500, -260], [-500, 1350], [-500, 1530],[-500, 1530], [-500, 1530]]
    
    # xBloc = [[-350, -200], [-1530, 860], [-920, -320], [-320, 900], [-320, 900], [200, 900], [-270, -50], [-920, 900], [-920, 900], [900, 910]]
    # yBloc = [[-150, 150], [-990, 1150], [-900, 900], [200, 900], [-900, -200], [-200, 200], [900, 1150], [900, 910], [-910, -900], [-900, 900]]
    # zBloc = [[-500, -150], [-500, 1530], [-500, 360], [-500, -260], [-500, -260], [-500, -260], [-500, 1350], [-500, 2300],[-500, 2300], [-500, 2300]]
    # name = ["cables", "zone_travail", "chariot", "plan_part1", "plan_part3", "plan_part2", "poteau", "mur_x_pos", "mur_x_neg", "mur_y_neg"]
    
    xBloc = [[-295, 895], [-1050, -500],   [-1050, 1110],  [1060, 1110],    [-195, -105]]
    yBloc = [[-295, 895], [-1214.5, 1275], [1275, 1325],   [-1214.5, 1325], [978, 1275]]
    zBloc = [[0, -637.5], [600, -637.5],   [2000, -637.5], [2000, -637.5],  [2000, -637.5]]
    name  = [  'table',      'rails',        'wall1',         'wall2',         'pillar']
    
    dilatation = 25 # mm
    dilatations = [1 for n in name]
    dilatations = [d * dilatation for d in dilatations]
    
    # add tool interference
    tool_size = 0.1 # m
    tool_diameter = 0.05 # m
    cylinder_pose = geometry_msgs.msg.PoseStamped()
    cylinder_pose.header.frame_id = group.get_end_effector_link() # "tool0"
    cylinder_pose.pose.position.x = 0
    cylinder_pose.pose.position.y = 0
    cylinder_pose.pose.position.z = (0.04+tool_size)/2
    cylinder_pose.pose.orientation.w = 1
    cylinder_name = "tool"
    scene.add_cylinder(cylinder_name, cylinder_pose, 0.04+tool_size, tool_diameter)
    self.box_name = cylinder_name
    self.wait_for_state_update(box_is_known=True)
    
    touch_links = robot.get_link_names(group='DN1P')
    scene.attach_box(group.get_end_effector_link(), cylinder_name, touch_links=touch_links)

    # add scene interferences
    for i in range(len(xBloc)):
      # if i!=1: # skip workspace as it's not a collision box

      # Define sizes of the box
      xL = (abs(xBloc[i][1]-xBloc[i][0]) + 2*dilatations[i])*1e-3
      yL = (abs(yBloc[i][1]-yBloc[i][0]) + 2*dilatations[i])*1e-3
      zL = (abs(zBloc[i][1]-zBloc[i][0]) + 2*dilatations[i])*1e-3
      
      # Define origin/center of the box
      xC = (xBloc[i][0]+xBloc[i][1])/2*1e-3
      yC = (yBloc[i][0]+yBloc[i][1])/2*1e-3
      zC = (zBloc[i][0]+zBloc[i][1])/2*1e-3
      
      # apply position of the box based on its origin/center
      box_pose = geometry_msgs.msg.PoseStamped()
      # rospy.logwarn(box_pose.header.frame_id)
      box_pose.header.frame_id = "base"
      box_pose.pose.position.x = round(xC, 2)
      box_pose.pose.position.y = round(yC, 2)
      box_pose.pose.position.z = round(zC, 2)
      box_pose.pose.orientation.w = 1

      box_name = name[i]
      scene.add_box(box_name, box_pose, size=(xL, yL, zL))
      rospy.logwarn('Added object {} to the scene, origin: x:{} y:{} z:{} | size: x:{} y:{} z:{}'.format(
        box_name,
        box_pose.pose.position.x, box_pose.pose.position.y, box_pose.pose.position.z,
        xL, yL, zL
      ))
        
      self.box_name = box_name
      self.wait_for_state_update(box_is_known=True)

def main():
  try:
    hc10 = MoveGroupPythonIntefaceTutorial()
    hc10.add_interferences()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()