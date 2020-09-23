#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupActionResult
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
import std_msgs.msg
from moveit_commander.conversions import pose_to_list, list_to_pose

class PickNPlaceTutorial():
    """PickNPlaceTutorial"""
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_n_place_tutorial', anonymous=True)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander('indy7')
        self.hand_group = moveit_commander.MoveGroupCommander('hand')

        self.plan_result_pub = rospy.Publisher("/move_group/result", MoveGroupActionResult, queue_size=1)

        # Misc variables
        self.box_name = ''

    def hold_hand(self, target_name):
        touch_links = ['left_inner_finger', 'right_inner_finger']
        self.hand_group.attach_object(target_name, 'robotiq_85_base_link', touch_links=touch_links)

    def release_hand(self, target_name):
        self.hand_group.detach_object(target_name)

    def jmove_to_pose_goal(self, pose_goal):
        # self.robot_group.set_pose_target(pose_goal)
        plan = self.robot_group.plan(pose_goal)
        self.robot_group.execute(plan, wait=True)
        # self.robot_group.go(wait=True)

    def jmove_to_joint_goal(self, joint_goal):
        # self.robot_group.go(joint_goal, wait=True)
        plan = self.robot_group.plan(joint_goal)
        self.robot_group.execute(plan, wait=True)
        
    def tmove_to_pose_goal(self, pose_goal):
        waypoints = []
        waypoints.append(self.robot_group.get_current_pose().pose)
        waypoints.append(pose_goal)

        plan, _ = self.robot_group.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold

        msg = MoveGroupActionResult()
        msg.result.planned_trajectory = plan
        self.plan_result_pub.publish(msg)
        self.robot_group.execute(plan)
    
    def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Note that attaching the box will remove it from known_objects
            is_known = box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()
        return False

    def add_box(self, name, pose_stamp, size=(0.05, 0.05, 0.05)):
        self.scene.add_box(name, pose_stamp, size=size)
        self.wait_for_state_update(box_name=name, box_is_known=True)

    def show_current_joint(self):
        joint_values = self.robot_group.get_current_joint_values()
        print joint_values
    def show_current_pos(self):
        pos_values = self.robot_group.get_current_pose()
        print pos_values
def main():
    try:
        pnp = PickNPlaceTutorial()
        while True:
            pnp.show_current_joint()
            pnp.show_current_pos()
            raw_input()
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("============ Python pick place demo complete!")
if __name__ == '__main__':
  main()
