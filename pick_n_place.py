#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import MoveGroupActionResult, RobotState, Constraints, OrientationConstraint
import geometry_msgs.msg
import shape_msgs.msg
from math import pi
from std_msgs.msg import String
import std_msgs.msg
from moveit_commander.conversions import pose_to_list, list_to_pose
import time
#from std_msgs.msg import String,Int32,Int32MultiArray,MultiArrayLayout,MultiArrayDimension
from std_msgs.msg import Int32MultiArray

class PickNPlaceTutorial():
    """PickNPlaceTutorial"""
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pick_n_place_tutorial', anonymous=True)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot_group = moveit_commander.MoveGroupCommander('indy7')
        self.hand_group = moveit_commander.MoveGroupCommander('hand')

        self.plan_result_pub = rospy.Publisher("/move_group/result", MoveGroupActionResult, queue_size=1)

        self.gripper_pub = rospy.Publisher("/robotis/pos",Int32MultiArray, queue_size = 1)

        self.robot_group.set_planner_id("RRTConnectkConfigDefault")
        self.robot_group.set_num_planning_attempts(5)
        self.robot_group.set_planning_time(5)
        #self.robot_group.allow_replanning(True)
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
    def go_cup_ready(self):

        joint_goal = [1.4966198335851, -1.326799297366056,
         -2.3646593370644577, -0.04153883619746379, 
         2.091894065899511, 1.56272327949214]
        self.jmove_to_pose_goal(joint_goal)

    def go_cup_pick(self):
        joint_goal = [1.5204237004310464, -1.2234713661491383, -2.060448448441854, -0.012849023353027792, 1.6853771921882825, 1.581778165977047]
        self.jmove_to_pose_goal(joint_goal)
    def go_cup_pick_up(self):
        joint_goal = [1.5213124031006353, -1.0696250320846978, -2.13191968131102, -0.010384709049366059, 1.601077789316958, 1.5834105228422943]
        self.jmove_to_pose_goal(joint_goal)
    def go_cup_pick_up_back(self):
        joint_goal = [1.485013394059338, -1.2205087459196038, -2.558914482811422, -0.05925392810520602, 2.176512843699474, 1.5491935569119]
        self.jmove_to_pose_goal(joint_goal)              
    
    def go_cup_juice_plate_1(self):
        joint_goal = [1.1585109273853935, -1.1412947340268882, -1.9651534712829661, 1.174693853054754, 1.5802267648312642, 1.5647516568196982]
        self.jmove_to_pose_goal(joint_goal)              
    
    def go_cup_juice_plate_2(self):
        joint_goal = [0.92240650967898, -1.172250392101962, -1.778228708394378, 0.9456193887305044, 1.4795156069155557, 1.4431953051824347]
        self.jmove_to_pose_goal(joint_goal)                         
    
    def go_gripper_pos(self,pos):
        cup_hold = Int32MultiArray()
        cup_hold.data = [pos]
        self.gripper_pub.publish(cup_hold)

    def go_cup_hold(self):
        cup_hold = Int32MultiArray()
        cup_hold.data = [100]
        self.gripper_pub.publish(cup_hold)
    def go_cup_release(self):
        cup_hold = Int32MultiArray()
        cup_hold.data = [50]
        self.gripper_pub.publish(cup_hold)
    def go_testing_pos_move(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.0174523581312
        pose_goal.orientation.y = -0.999828809961
        pose_goal.orientation.z = 0.00613744673102
        pose_goal.orientation.w = 0.000312593759324

        pose_goal.position.x = 0.18
        pose_goal.position.y = 0.47
        pose_goal.position.z = 0.80

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_orange_to_juice_pot2(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.620015838475
        pose_goal.orientation.y = -0.373841701158
        pose_goal.orientation.z = -0.332026067318
        pose_goal.orientation.w = 0.604633304687

        pose_goal.position.x = -0.561062699341
        pose_goal.position.y = -0.133818775573
        pose_goal.position.z = 1.62573164543

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_orange_pick_pos(self):
        joint_goal = [-1.525941365018604, -0.14172073526193607, -0.9297368925373561, 0.0006108652381979773, -2.083661327493378, 1.5812857556003057]
        self.jmove_to_pose_goal(joint_goal)              
    
    def go_orange_to_juice_pot1(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.620121216554
        pose_goal.orientation.y = -0.374091217336
        pose_goal.orientation.z = -0.331611714704
        pose_goal.orientation.w = 0.604598303473

        pose_goal.position.x = -0.448826582349
        pose_goal.position.y = 0.0282399813568
        pose_goal.position.z = 1.68930662465

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_juicer_pick_cup_ok(self):
        joint_goal = [1.1844676968659216, -1.2657127735462559, -1.8955148341283938, 1.187522023056912, 1.601077789316958, 1.4844723419921242]
        self.jmove_to_pose_goal(joint_goal)   

    def test_constraints(self):


        constraint = Constraints()
        constraint.name = "tilt constraint"
        ocm = OrientationConstraint()
        # 'base_link' is equal to the world link
        ocm.header.frame_id = "world"
        # The link that must be oriented upwards
        ocm.link_name = "tcp"
        # ocm.orientation = Quaternion(0.5, 0.5, 0.5, 0.5)
        ocm.orientation.x =  0.683951325248
        ocm.orientation.y =  -0.209429903965
        ocm.orientation.z =  -0.202265206016
        ocm.orientation.w =  0.668908429049

        # Allow rotation of 45 degrees around the x and y axis
        ocm.absolute_x_axis_tolerance = 0.5 #Allow max rotation of 45 degrees
        ocm.absolute_y_axis_tolerance = 1.5 #Allow max rotation of 360 degrees
        ocm.absolute_z_axis_tolerance = 0.5 #Allow max rotation of 45 degrees
        # The tilt constraint is the only constraint
        ocm.weight = 1
        constraint.orientation_constraints = [ocm]

        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.461741097024
        pose_goal.position.y = -0.433851093412
        pose_goal.position.z = 1.26149858128
        move_group.set_pose_target(pose_goal)

        move_group.set_path_constraints(constraint)
        move_group.plan()
       

    def test_constraints2(self):
        move_group = self.robot_group
        move_group.clear_path_constraints()
        self.go_juicer_pick_cup_ok()
        self.test_constraints()
        # self.go_sealing_step1()
    def go_juicer_constraint_test(self):

        
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.461741097024
        pose_goal.position.y = -0.433851093412
        pose_goal.position.z = 1.26149858128
        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_sealing_step1(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.461741097024
        pose_goal.position.y = -0.433851093412
        pose_goal.position.z = 1.26149858128

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_sealing_step2(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.545566440256
        pose_goal.position.y = -0.515964899805
        pose_goal.position.z = 1.26149858128

        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_sealing_goal(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.683951325248
        pose_goal.orientation.y = -0.209429903965
        pose_goal.orientation.z = -0.202265206016
        pose_goal.orientation.w = 0.668908429049

        pose_goal.position.x = -0.545566440256
        pose_goal.position.y = -0.515964899805
        pose_goal.position.z = 1.05706463542

        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_juicer_push_ready(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = -0.489606100026
        pose_goal.orientation.y = -0.500430269326
        pose_goal.orientation.z = 0.49569733542
        pose_goal.orientation.w = 0.513945098252

        pose_goal.position.x = -0.398840350759
        pose_goal.position.y = -0.201061446823
        pose_goal.position.z = 1.04976587286

        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_juicer_pop(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = -0.489606100026
        pose_goal.orientation.y = -0.500430269326
        pose_goal.orientation.z = 0.49569733542
        pose_goal.orientation.w = 0.513945098252

        pose_goal.position.x = -0.418840350759
        pose_goal.position.y = -0.201061446823
        pose_goal.position.z = 1.04976587286

        move_group.set_pose_target(pose_goal)
        move_group.go()
    def go_juicer_push(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = -0.489606100026
        pose_goal.orientation.y = -0.500430269326
        pose_goal.orientation.z = 0.49569733542
        pose_goal.orientation.w = 0.513945098252

        pose_goal.position.x = -0.418840350759
        pose_goal.position.y = -0.201061446823
        pose_goal.position.z = 1.02876587286

        move_group.set_pose_target(pose_goal)
        move_group.go()        
def main():
    try:
        pnp = PickNPlaceTutorial()
        pnp.show_current_joint()
        pnp.show_current_pos()
        while True:            
            input = raw_input()
            if( input == '1'):
                pnp.go_cup_ready()
            elif( input == '2'):
                pnp.go_cup_pick()
                pnp.go_gripper_pos(130)
                pnp.go_cup_pick_up()
                pnp.go_cup_pick_up_back()
            elif( input == '3'):
                pnp.go_cup_juice_plate_1()
                pnp.go_cup_juice_plate_2()
                pnp.go_gripper_pos(0)
                pnp.go_cup_juice_plate_1()
            elif( input == '3.1'):
                pnp.go_orange_pick_pos()
            elif( input == '4'):
                pnp.go_orange_to_juice_pot1()                
                pnp.go_orange_to_juice_pot2()
                pnp.go_cup_release()
                time.sleep(0.5)
                pnp.go_orange_to_juice_pot1()             
            elif( input == '4.1'):
                pnp.go_juicer_pick_cup_ok()
            elif( input == '5'):
                pnp.go_sealing_step1()
            elif( input == '6'):
                pnp.go_sealing_step2()                
            elif( input == '7'):
                pnp.go_sealing_goal()
            elif( input == '8'):
                pnp.go_juicer_push_ready()
            elif( input == '9'):
                pnp.go_juicer_pop()

                #pnp.go_juicer_pop()
                pnp.go_juicer_push()
                time.sleep(4)
                pnp.go_juicer_pop()
                #pnp.go_juicer_push()
                #pnp.go_juicer_pop()
                #pnp.go_juicer_push()

                pnp.go_juicer_pop()

            elif( input == '10'):
                pnp.go_juicer_pop()
            elif( input == 'h'):
                pnp.go_gripper_pos(80)
            elif( input == 'a'):
                pnp.go_gripper_pos(255)
            elif( input == 'r'):
                pnp.go_gripper_pos(0)       
            elif( input == 't'):
                pnp.go_testing_pos_move()
            elif( input == 'q'):
                break
            elif( input == 'a1'):
                pnp.test_constraints()
            elif( input == 'a2'):
                pnp.test_constraints2()
            else:
                pnp.show_current_joint()
                pnp.show_current_pos()
            
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    else:
        print("============ Python pick place demo complete!")
if __name__ == '__main__':
  main()

