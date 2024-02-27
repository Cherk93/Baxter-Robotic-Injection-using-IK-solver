#!/usr/bin/env python3

import numpy as np
import cv2 as cv
import pyrealsense2 as rs
import rospy
import realsense2_camera as roscam
import mediapipe as mp
import time
from geometry_msgs.msg import Point 

class Human_pose(object):
    def __init__(self,mode = False, upperbod = False, smooth = True, detection_con = 0.5, traking_con = 0.5):
        self.mode = mode
        self.upperbod = upperbod
        self.smooth = smooth
        self.detection_con = detection_con
        self.traking_con = traking_con

        self.mpDraw = mp.solutions.mediapipe.python.solutions.drawing_utils
        self.mpPose = mp.solutions.mediapipe.python.solutions.pose
        self.my_pose = self.mpPose.Pose()    
        self.jointPix_1 = np.zeros((2),dtype=int)
        self.jointPix_2 = np.zeros((2),dtype=int)

        pass

    def get_Jointpose(self,col_img,draw = True):
        pos_res = self.my_pose.process(col_img)

        
        if pos_res.pose_landmarks:
            if draw:
                self.mpDraw.draw_landmarks(col_img,pos_res.pose_landmarks)
            for id,lm in enumerate(pos_res.pose_landmarks.landmark):
                h,w,c = col_img.shape
                if id == 12:
                    self.jointPix_1[0] ,self.jointPix_1[1] = int(lm.x * w), int(lm.y * h)
                    # print(fp_x, fp_y)
                    # cv.circle(col_img,self.jointPix_1,10,(255,0,0),cv.FILLED)
                elif id == 14:
                    self.jointPix_2[0] ,self.jointPix_2[1] = int(lm.x * w), int(lm.y * h)
                    # print(fp_x, fp_y)
                # cv.circle(color_image,(fp_x1,fp_y1),10,(255,0,0),cv.FILLED)
                    # cv.circle(col_img,self.jointPix_2,10,(255,0,0),cv.FILLED)
            # print(self.jointPix_1,self.jointPix_2)
        pass

class Real_Sense(object):
    def __init__(self):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        self.pipeline_profile = self.config.resolve(self.pipeline_wrapper)
        self.device = self.pipeline_profile.get_device()
        self.device_product_line = str(self.device.get_info(rs.camera_info.product_line))
        self.RGB_device_flag = False
        self.align = None
        self.color_frame = None
        self.color_image = None
        self.depth_image = None
        self.depth_scale = None
        self.joint3D_1 = np.zeros((3))
        self.joint3D_2 = np.zeros((3))
        self.midpoint = np.zeros(3)
        self.ptime = 0
        
        pass

    def sanity_check(self):
        for dev in self.device.sensors:
            if dev.get_info(rs.camera_info.name) == 'RGB Camera':
                self.RGB_device_flag = True
                break
        # if not self.RGB_device_flag:
        #     print("The demo requires Depth camera with Color sensor")
            # exit(0)
    def device_init(self):

        self.sanity_check()
        if not self.RGB_device_flag:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        if self.device_product_line == 'L500':
            self.config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(self.config)
        # Getting the depth sensor's depth scale (see rs-align example for explanation)
        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        print("Depth Scale is: " , self.depth_scale)

        align_to = rs.stream.color
        self.align = rs.align(align_to)

    def IRS_startStreaming(self):

        # Get FPS
        ctime = time.time()
        fps = 1/(ctime-self.ptime)
        self.ptime = ctime

        # Get frameset of color and depth
        frames = self.pipeline.wait_for_frames()

        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        self.color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not self.color_frame:
            return
        
        #**************************Raw Depth*********************************#

        # raw_depth_frame = frames.get_depth_frame()
        # raw_depth_image = np.asanyarray(raw_depth_frame.get_data())
        # raw_depth_colormap = cv.applyColorMap(cv.convertScaleAbs(raw_depth_image, alpha=0.03), cv.COLORMAP_JET)

        #********************************************************************#

        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())

        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(self.depth_image, alpha=0.03), cv.COLORMAP_JET)
        # images = np.hstack((color_image, depth_colormap))

        cv.putText(self.color_image,str(int(fps)),(70,50),cv.FONT_HERSHEY_PLAIN,3,(255,0,0),3)

        pass

    def deproj_joint3D(self,jointPix_1,jointPix_2):

        color_intrinsic = self.color_frame.profile.as_video_stream_profile().intrinsics

        # print(jointPix_1[0],jointPix_2[0])
        j3d_1 = rs.rs2_deproject_pixel_to_point(color_intrinsic,(jointPix_1[0],jointPix_1[1]),self.depth_image[jointPix_1[0],jointPix_1[1]]*self.depth_scale)
        j3d_2 = rs.rs2_deproject_pixel_to_point(color_intrinsic,(jointPix_2[0],jointPix_2[1]),self.depth_image[jointPix_2[0],jointPix_2[1]]*self.depth_scale)

        self.joint3D_1 = np.asarray(j3d_1)
        self.joint3D_2 = np.asarray(j3d_2)
        # print(self.joint3D_1 , self.joint3D_2)
        self.midpoint = self.joint3D_1
        # self.midpoint = (self.joint3D_1 + self.joint3D_2)/2
        print(self.midpoint)

def ROS_publisher():

    pass

if __name__ == "__main__":
    print("IRS_CV_Ver2 Starting")
    pub = rospy.Publisher('bax_irs_publisher', Point, queue_size=10)
    rospy.init_node("IntelRealSense",anonymous=False)
    Mid_point = Point()
    joint1 = Point()

    rate = rospy.Rate(10)
    target_pose = Human_pose()
    MyRealSense = Real_Sense()

    MyRealSense.sanity_check()
    MyRealSense.device_init()

    try:
        while not rospy.is_shutdown():
            MyRealSense.IRS_startStreaming()
            target_pose.get_Jointpose(MyRealSense.color_image,draw=False)
            MyRealSense.deproj_joint3D(target_pose.jointPix_1,target_pose.jointPix_2)
            Mid_point.x = MyRealSense.midpoint[0]
            Mid_point.y = MyRealSense.midpoint[1]
            Mid_point.z = MyRealSense.midpoint[2]

            # print(jo)
            if(Mid_point.x + Mid_point.y +Mid_point.z)>0.1:
                pub.publish(Mid_point)
            rate.sleep()
            pass
    except rospy.ROSInterruptException:
        MyRealSense.pipeline.stop()
        pass

    # try:
    #     pass
    # except:
    #     pass
