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
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.697164469105
        pose_goal.orientation.x = 0.716618448965
        pose_goal.orientation.y = -0.0185623033006
        pose_goal.orientation.z = -0.00866847830944

        pose_goal.position.x = -0.213759464511
        pose_goal.position.y = -0.473728179994
        pose_goal.position.z = 0.899694597225

        move_group.set_pose_target(pose_goal)
        move_group.go()    
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.697164469105
        pose_goal.orientation.x = 0.716618448965
        pose_goal.orientation.y = -0.0185623033006
        pose_goal.orientation.z = -0.00866847830944

        pose_goal.position.x = -0.213759464511
        pose_goal.position.y = -0.473728179994
        pose_goal.position.z = 0.899694597225

        move_group.set_pose_target(pose_goal)
        move_group.go()

    def go_cup_pick(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.697164469105
        pose_goal.orientation.x = 0.716618448965
        pose_goal.orientation.y = -0.0185623033006
        pose_goal.orientation.z = -0.00866847830944

        pose_goal.position.x = -0.213759464511
        pose_goal.position.y = -0.593728179994
        pose_goal.position.z = 0.899694597225

        move_group.set_pose_target(pose_goal)
        move_group.go()    
    def go_cup_pick_up(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 0.697164469105
        pose_goal.orientation.x = 0.716618448965
        pose_goal.orientation.y = -0.0185623033006
        pose_goal.orientation.z = -0.00866847830944

        pose_goal.position.x = -0.213759464511
        pose_goal.position.y = -0.593728179994
        pose_goal.position.z = 0.959694597225

        move_group.set_pose_target(pose_goal)
        move_group.go()            
    def go_cup_pick_up_back(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.w = 0.696290026114
        pose_goal.orientation.x = 0.717493706632
        pose_goal.orientation.y = -0.018006197871
        pose_goal.orientation.z = -0.00766533208544
        

        pose_goal.position.x = -0.213033016781
        pose_goal.position.y = -0.433996235451
        pose_goal.position.z = 0.9593263894

        move_group.set_pose_target(pose_goal)
        move_group.go()                    
    
    def go_cup_juice_plate_1(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.4832957098
        pose_goal.orientation.y = -0.504994165185
        pose_goal.orientation.z = -0.502877654077
        pose_goal.orientation.w = 0.508448832282

        pose_goal.position.x = -0.397201939325
        pose_goal.position.y = -0.198214268682
        pose_goal.position.z = 0.936391917232

        move_group.set_pose_target(pose_goal)
        move_group.go()  
    def go_cup_juice_plate_2(self):
        move_group = self.robot_group
        pose_goal = geometry_msgs.msg.Pose()
        
        pose_goal.orientation.x = 0.483213559137
        pose_goal.orientation.y = -0.505158316969
        pose_goal.orientation.z = -0.502849044308
        pose_goal.orientation.w = 0.508392141659

        pose_goal.position.x = -0.518667834037
        pose_goal.position.y = -0.196999708132
        pose_goal.position.z = 0.907416595201

        move_group.set_pose_target(pose_goal)
        move_group.go()                    
    
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
                pnp.go_cup_hold()
                pnp.go_cup_pick_up()
                pnp.go_cup_pick_up_back()
            elif( input == '3'):
                pnp.go_cup_juice_plate_1()
                pnp.go_cup_juice_plate_2()
                pnp.go_gripper_pos(0)
                pnp.go_cup_juice_plate_1()
            elif( input == '4'):
                pnp.go_orange_to_juice_pot1()                
                pnp.go_orange_to_juice_pot2()
                pnp.go_cup_release()
                time.sleep(0.5)
                pnp.go_orange_to_juice_pot1()                
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

