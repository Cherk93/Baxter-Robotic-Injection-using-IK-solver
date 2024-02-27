#!/usr/bin/env python3
import numpy as np
import baxter_interface
import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import baxter_interface
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
import argparse
import struct
import sys
from std_msgs.msg import Header


# rospy.init_node('Hello_baxter')
# limb = baxter_interface.Limb('right')

# angles = limb.joint_angles()

# print("Initial Position")
# print(angles)

class BaxCtr(object):
    def __init__(self):
        self.sub = rospy.Subscriber("/bax_irs_publisher",Point,self.callback_irs)
        self.targetpos = Point()
        pass
    def callback_irs(self,data):
        self.targetpos = data
        print("calback working")
        pass

def ik_test(limb,tx,ty,tz):
    # rospy.init_node("rsdk_ik_service_client")
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=tx,
                    y=ty,
                    z=tz,
                ),
                orientation=Quaternion(
                    x=0.1,
                    y=0.1,
                    z=0.1,
                    w=0.1,
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=0.1,
                    y=-0.5,
                    z=0,
                ),
                orientation=Quaternion(
                    x=0.1,
                    y=0.1,
                    z=0.1,
                    w=0.1,
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print ("\nIK Joint Solution:\n", limb_joints)
        print ("------------------")
        print ("Response Message:\n"), resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return limb_joints


if __name__ == "__main__":
    rospy.init_node('Hello_baxter')
    print("Hello_baxter node started")
    H = np.zeros((4,4))
    H[0] = 0,0,1,(0.2-0.1)   #(0.072+0.0381)
    H[1] = -1,0,0,(-0.04445+0.05)
    H[2] = 0,-1,0,(0.9144-0.2)
    H[3] = 0,0,0,1
    target_pos_c = np.zeros((4,1))
    target_pos_bax = np.zeros((4,1))
    
    my_baxctr = BaxCtr()
    

    

    rate = rospy.Rate(10)
    left_limb = baxter_interface.Limb("left")
    right_limb = baxter_interface.Limb("right")
    joint_solutions = ik_test("right",0,0,0)
    right_limb.move_to_joint_positions(joint_solutions)
    joint_solution1 = ik_test("left",0.7,0.5,0.2)
    left_limb.move_to_joint_positions(joint_solution1)
    command = input("Provide move cmd:")
    try:
        while not rospy.is_shutdown():
            print(my_baxctr.targetpos)
            if command =="move":
                target_pos_c[0,0] = my_baxctr.targetpos.x
                target_pos_c[1,0] = my_baxctr.targetpos.y
                target_pos_c[2,0] = my_baxctr.targetpos.z
                target_pos_c[3,0] = 1
                target_pos_bax = H@target_pos_c
                print(target_pos_bax)
                joint_solutions = ik_test("left",target_pos_bax[0,0]-0.15,target_pos_bax[1,0],target_pos_bax[2,0])
                left_limb.move_to_joint_positions(joint_solutions)
                joint_solutions = ik_test("left",(target_pos_bax[0,0]+0.05),target_pos_bax[1,0],target_pos_bax[2,0])
                left_limb.move_to_joint_positions(joint_solutions)
                # joint_solutions = ik_test("left",(target_pos_bax[0,0]),target_pos_bax[1,0],target_pos_bax[2,0])
                # left_limb.move_to_joint_positions(joint_solutions)
                break
            rate.sleep()
    except rospy.ROSInterruptException:
        
        pass
