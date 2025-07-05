#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import ParamValue
from std_msgs.msg import *
from gazebo_ros_link_attacher.srv import Gripper
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import math
import numpy as np

class pick_n_place:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('pick_n_place_aruco', anonymous=True)

    
    def setArm(self, value=True):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(value)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 

    def setLand(self,alt):
        rospy.wait_for_service('mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService(altitude = alt)
        except rospy.ServiceException as e:
            print("Service land call failed: %s"%e)   

    def setTakeoff(self,alt):
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = alt)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)          


    def setParam(self,param, value):
        rospy.wait_for_service('mavros/param/set')     
        
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)
        try:
            paramService = rospy.ServiceProxy('mavros/param/set', mavros_msgs.srv.ParamSet)
            paramService(param_id=param, value=val)
        except rospy.ServiceException as e:
            print("Service param set failed: %s"%e)                            

   
    def offboard_set_mode(self):
        #pass

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        
        rospy.wait_for_service('mavros/set_mode') 
        try:
            setModeService = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print ("Service setmode call failed: %s"%e)

    def gripper(self,value):
        
        rospy.wait_for_service('activate_gripper') 
        try:
            gripperService = rospy.ServiceProxy('activate_gripper', Gripper)
            result = gripperService(activate_gripper=value)
            return result

        except rospy.ServiceException as e:
            print ("Service gripper call failed: %s"%e)
        


class stateMoniter:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def pose_callback(self,msg1):
        global position_x,position_y,position_z,position_x1,position_y1,position_z1
        position_x = round(msg1.pose.position.x)
        position_y = round(msg1.pose.position.y)
        position_z = round(msg1.pose.position.z)

        position_x1 = round(msg1.pose.position.x,1)
        position_y1 = round(msg1.pose.position.y,1)
        position_z1 = round(msg1.pose.position.z,1)
        
    def gripper_callback(self,msg2):
        global gripper_bool_value
        gripper_bool_value=msg2.data  

class image_proc():

    def __init__(self):
        self.Image=0
        self.image_sub = rospy.Subscriber("/eDrone/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        self.bridge = CvBridge()
        self.img = np.empty([])    

    def image_callback(self, data):
        global img_aruco
        try:
            img_aruco = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image
            # cv2.imshow('frame', img_aruco)
            # cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            return

    def process(self,data):

        marker_size=20
        horizontal_res=400
        vertical_res=400
        # horizontal_fov = 62.2*math.pi/180
        # vertical_fov = 48.8*math.pi/180
        dist_coeff= [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_matrix=[[238.3515418007097, 0.0, 200.5], [0.0, 238.3515418007097, 200.5], [0.0, 0.0, 1.0]]
        np_camera_matrix=np.array(camera_matrix)
        np_dist_coeff=np.array(dist_coeff)

        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) 
        Detected_ArUco_markers = {}

        try:
            for i in range(len(ids)):
                Detected_ArUco_markers[ids[i][0]]=corners[i]
            
            for ids, corner1 in Detected_ArUco_markers.items():
                corner = corner1[0]
                #11
                ret=aruco.estimatePoseSingleMarkers(corner1,marker_size,cameraMatrix=np_camera_matrix,distCoeffs=np_dist_coeff)
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]
                x='{:.2f}'.format(tvec[0])
                y='{:.2f}'.format(tvec[1])
                z='{:.2f}'.format(tvec[2])

                marker_position= 'MARKER POSITION: x='+x+' y='+y+' z='+z
                print(marker_position)

                # Since angle is atan2(-y,x), then converting that to degrees
                top_right_angle = (math.degrees(math.atan2(-corner[1][1] + corner[3][1], corner[1][0] - corner[3][0]))) % 360
                angle = (top_right_angle + 45) % 360

                center_x=int((corner[0][0]+corner[2][0]+corner[1][0]+corner[3][0])/4)
                center_y=int((corner[0][1]+corner[2][1]+corner[3][1]+corner[1][1])/4)

                print(f"{ids},{center_x},{center_y},{int(angle)}")  
                print(position_x1,position_y1,position_z1)
                #print(f"img w,h {img_aruco.shape[0]},{img_aruco.shape[1]}")
                return center_x, center_y,x
        except:
            print("no aruco")

def main():

    stateMt = stateMoniter()
    ofb_ctl = pick_n_place()
    aruco_detection=image_proc()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [[0.0,0.0,3.0],[9.0,0.0,3.0],[9.0,0.0,0.0]] #List to setpoints

    # Similarly initialize other publishers 

    # Create empty message containers 
    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 0

    # Set your velocity here
    vel = Twist()
    vel.linear.x = 0
    vel.linear.y = 0
    vel.linear.z = 0
    
    # Similarly add other containers 

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped, stateMt.pose_callback)
    rospy.Subscriber("/gripper_check",String, stateMt.gripper_callback)

    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''

    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    ofb_ctl.setParam('COM_RCL_EXCEPT',4)
    ofb_ctl.setParam('COM_DISARM_LAND',-1.0)

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":

        for i in range(100):
            local_pos_pub.publish(pos)
            rate.sleep()  

        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
    

    # Publish the setpoints 
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''
        
        while ([position_x,position_y,position_z]!=setpoints[0]):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[0][0],setpoints[0][1],setpoints[0][2]
            local_pos_pub.publish(pos)
            rate.sleep()
            continue
        print("setpoint 1 reached")    

        while ([position_x1,position_y1,position_z]!=setpoints[1]):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
            vel.linear.x=1
            local_vel_pub.publish(vel)
            local_pos_pub.publish(pos)
            rate.sleep()
            try:
                center_x, center_y, x =aruco_detection.process(img_aruco)
              
                # cv2.imshow('frame', img_aruco)
                # cv2.waitKey(1)

                if center_x in range (0,400):
                    center_x_intial=center_x
                    print("can land")
                    #box_x,box_y,box_z = round(position_x1+(center_x_intial-200)*0.03,1),position_y1,position_z1
                    box_x,box_y,box_z = round(position_x1+float(x)/100,1),position_y1,position_z1
                    box_x1,box_y1,box_z1 = round(position_x+float(x)/100),position_y,position_z

                    # last_request=rospy.get_rostime() 
                    # while not ((rospy.get_rostime()-last_request)>rospy.Duration(1)):
                    #     continue  

                    while ([position_x1,position_y1,position_z1]!=[box_x,box_y,box_z]):
                        #print("1")
                        pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x,box_y,box_z
                        local_pos_pub.publish(pos)
                        rate.sleep()
                        continue
                    print ('setpoint 2 reached')
                    while (position_z1!=0):
                        while not stateMt.state.guided:
                            ofb_ctl.setLand(alt=0)
                            rate.sleep()
                        #print("Landing!!")
                        continue
                    print("setpoint 3 reached")

                    last_request=rospy.get_rostime() 

                    while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
                        continue  

                    # print(str(gripper_bool_value))
                    # if str(gripper_bool_value)=='True':
                    #     ofb_ctl.gripper(value=True)
                    
                    #     while not stateMt.state.mode=="OFFBOARD":

                    #         for i in range(100):
                    #             local_pos_pub.publish(pos)
                    #             rate.sleep()  

                    #         ofb_ctl.offboard_set_mode()
                    #         rate.sleep()
                    #     print ("OFFBOARD mode activated")

                    #     while ([position_x1,position_y1,position_z]!=setpoints[1]):
                    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
                    #         local_pos_pub.publish(pos)
                    #         rate.sleep()
                    #         continue
                    #     print("setpoint 2 reached")
                        
                    #     while ([position_x,position_y,position_z1]!=setpoints[2]):
                    #         while not stateMt.state.guided:
                    #             ofb_ctl.setLand(alt=0)
                    #             rate.sleep()
                    #         print("Landing!!")
                    #         continue
                    #     print("setpoint 3 reached")

                    #     last_request=rospy.get_rostime()
                    #     while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
                    #         continue 

                    #     ofb_ctl.gripper(value=False)

                    #     while not stateMt.state.mode=="OFFBOARD":

                    #         for i in range(100):
                    #             local_pos_pub.publish(pos)
                    #             rate.sleep()  

                    #         ofb_ctl.offboard_set_mode()
                    #         rate.sleep()
                    #     print ("OFFBOARD mode activated")

                    #     while ([position_x,position_y,position_z]!=setpoints[1]):
                    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
                    #         local_pos_pub.publish(pos)
                    #         rate.sleep()
                    #         continue
                    #     print("setpoint 7 reached")

                    #     while ([position_x,position_y,position_z]!=setpoints[0]):
                    #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[0][0],setpoints[0][1],setpoints[0][2]
                    #         local_pos_pub.publish(pos)
                    #         rate.sleep()
                    #         continue
                    #     print("setpoint 8 reached")

                    #     ofb_ctl.setParam('COM_DISARM_LAND',4.0)
                    #     while not stateMt.state.guided:
                    #         ofb_ctl.setLand(alt=0)
                    #         rate.sleep()
                    #     print("Landing!!")

                    #    break

                    while (str(gripper_bool_value)=='False'):
                        print(gripper_bool_value)
                        while not stateMt.state.mode=="OFFBOARD":

                            for i in range(100):
                                local_pos_pub.publish(pos)
                                rate.sleep()  

                            ofb_ctl.offboard_set_mode()
                            rate.sleep()
                        print ("OFFBOARD mode activated")

                        while ([position_x1,position_y1,position_z1]!=[box_x,box_y,box_z]):
                            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x,box_y,box_z
                            local_pos_pub.publish(pos)
                            rate.sleep()
                            continue
                        print("setpoint 2 reached")
                        
                        while (position_z1!=0):
                            while not stateMt.state.guided:
                                ofb_ctl.setLand(alt=0)
                                rate.sleep()
                            print("Landing!!")
                            continue
                        print("setpoint 3 reached")

                        last_request=rospy.get_rostime()
                        while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
                            continue 

                    while (not ofb_ctl.gripper(value=True).result):
                        while not stateMt.state.mode=="OFFBOARD":

                            for i in range(100):
                                local_pos_pub.publish(pos)
                                rate.sleep()  

                            ofb_ctl.offboard_set_mode()
                            rate.sleep()
                        print ("OFFBOARD mode activated")

                        while ([position_x1,position_y1,position_z]!=[box_x,box_y,box_z]):
                            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x,box_y,box_z
                            local_pos_pub.publish(pos)
                            rate.sleep()
                            continue
                        print("setpoint 2 reached")
                        
                        while (position_z1!=0):
                            while not stateMt.state.guided:
                                ofb_ctl.setLand(alt=0)
                                rate.sleep()
                            print("Landing!!")
                            continue
                        print("setpoint 3 reached")

                        last_request=rospy.get_rostime() 

                        while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
                            continue  
                        while (str(gripper_bool_value)=='False'):
                            print(gripper_bool_value)
                            while not stateMt.state.mode=="OFFBOARD":

                                for i in range(100):
                                    local_pos_pub.publish(pos)
                                    rate.sleep()  

                                ofb_ctl.offboard_set_mode()
                                rate.sleep()
                            print ("OFFBOARD mode activated")

                            while ([position_x1,position_y1,position_z]!=[box_x,box_y,box_z]):
                                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x,box_y,box_z
                                local_pos_pub.publish(pos)
                                rate.sleep()
                                continue
                            print("setpoint 2 reached")
                            
                            while (position_z1!=0):
                                while not stateMt.state.guided:
                                    ofb_ctl.setLand(alt=0)
                                    rate.sleep()
                                print("Landing!!")
                                continue
                            print("setpoint 3 reached")

                            last_request=rospy.get_rostime()
                            while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
                                continue 

                    ofb_ctl.gripper(value=True)  

                    while not stateMt.state.mode=="OFFBOARD":

                        for i in range(100):
                            local_pos_pub.publish(pos)
                            rate.sleep()  

                        ofb_ctl.offboard_set_mode()
                        rate.sleep()
                    print ("OFFBOARD mode activated")

                    while ([position_x,position_y,position_z]!=[box_x1,box_y1,box_z1]):
                        pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x1,box_y1,box_z1
                        local_pos_pub.publish(pos)
                        rate.sleep()
                        continue
                    print("setpoint 4 reached")

                    while ([position_x,position_y,position_z]!=setpoints[1]):
                        pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
                        local_pos_pub.publish(pos)
                        rate.sleep()
                        continue
                    print("setpoint 5 reached")
                    
                    while ([position_x,position_y,position_z]!=setpoints[2]):
                        while not stateMt.state.guided:
                            ofb_ctl.setLand(alt=0)
                            rate.sleep()
                        print("Landing!!")
                        continue
                    print("setpoint 6 reached")

                    last_request=rospy.get_rostime()
                    while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
                        continue 

                    ofb_ctl.gripper(value=False)

                    while not stateMt.state.mode=="OFFBOARD":

                        for i in range(100):
                            local_pos_pub.publish(pos)
                            rate.sleep()  

                        ofb_ctl.offboard_set_mode()
                        rate.sleep()
                    print ("OFFBOARD mode activated")

                    while ([position_x,position_y,position_z]!=setpoints[1]):
                        pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
                        local_pos_pub.publish(pos)
                        rate.sleep()
                        continue
                    print("setpoint 7 reached")

                    while ([position_x,position_y,position_z]!=setpoints[0]):
                        pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[0][0],setpoints[0][1],setpoints[0][2]
                        local_pos_pub.publish(pos)
                        rate.sleep()
                        continue
                    print("setpoint 8 reached")

                    ofb_ctl.setParam('COM_DISARM_LAND',4.0)
                    while not stateMt.state.guided:
                        ofb_ctl.setLand(alt=0)
                        rate.sleep()
                    print("Landing!!") 
                    break     
            except:
                pass  

            continue
        break    
        #print("setpoint 2 reached")
        #break
        
        # while ([position_x,position_y,position_z1]!=setpoints[2]):
        #     while not stateMt.state.guided:
        #         ofb_ctl.setLand(alt=0)
        #         rate.sleep()
        #     print("Landing!!")
        #     continue
        # print("setpoint 3 reached")

        # last_request=rospy.get_rostime() 

        # while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
        #     continue  
        # while (str(gripper_bool_value)=='False'):
        #     print(gripper_bool_value)
        #     while not stateMt.state.mode=="OFFBOARD":

        #         for i in range(100):
        #             local_pos_pub.publish(pos)
        #             rate.sleep()  

        #         ofb_ctl.offboard_set_mode()
        #         rate.sleep()
        #     print ("OFFBOARD mode activated")

        #     while ([position_x1,position_y1,position_z]!=setpoints[1]):
        #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
        #         local_pos_pub.publish(pos)
        #         rate.sleep()
        #         continue
        #     print("setpoint 2 reached")
            
        #     while ([position_x,position_y,position_z1]!=setpoints[2]):
        #         while not stateMt.state.guided:
        #             ofb_ctl.setLand(alt=0)
        #             rate.sleep()
        #         print("Landing!!")
        #         continue
        #     print("setpoint 3 reached")

        #     last_request=rospy.get_rostime()
        #     while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
        #         continue 

        # ####
        # while (not ofb_ctl.gripper(value=True).result):
        #     while not stateMt.state.mode=="OFFBOARD":

        #         for i in range(100):
        #             local_pos_pub.publish(pos)
        #             rate.sleep()  

        #         ofb_ctl.offboard_set_mode()
        #         rate.sleep()
        #     print ("OFFBOARD mode activated")

        #     while ([position_x1,position_y1,position_z]!=setpoints[1]):
        #         pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
        #         local_pos_pub.publish(pos)
        #         rate.sleep()
        #         continue
        #     print("setpoint 2 reached")
            
        #     while ([position_x,position_y,position_z1]!=setpoints[2]):
        #         while not stateMt.state.guided:
        #             ofb_ctl.setLand(alt=0)
        #             rate.sleep()
        #         print("Landing!!")
        #         continue
        #     print("setpoint 3 reached")

        #     last_request=rospy.get_rostime() 

        #     while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
        #         continue  
        #     while (str(gripper_bool_value)=='False'):
        #         print(gripper_bool_value)
        #         while not stateMt.state.mode=="OFFBOARD":

        #             for i in range(100):
        #                 local_pos_pub.publish(pos)
        #                 rate.sleep()  

        #             ofb_ctl.offboard_set_mode()
        #             rate.sleep()
        #         print ("OFFBOARD mode activated")

        #         while ([position_x1,position_y1,position_z]!=setpoints[1]):
        #             pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
        #             local_pos_pub.publish(pos)
        #             rate.sleep()
        #             continue
        #         print("setpoint 2 reached")
                
        #         while ([position_x,position_y,position_z1]!=setpoints[2]):
        #             while not stateMt.state.guided:
        #                 ofb_ctl.setLand(alt=0)
        #                 rate.sleep()
        #             print("Landing!!")
        #             continue
        #         print("setpoint 3 reached")

        #         last_request=rospy.get_rostime()
        #         while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
        #             continue 

        # ofb_ctl.gripper(value=True)
        #####

        # while not stateMt.state.mode=="OFFBOARD":

        #     for i in range(100):
        #         local_pos_pub.publish(pos)
        #         rate.sleep()  

        #     ofb_ctl.offboard_set_mode()
        #     rate.sleep()
        # print ("OFFBOARD mode activated")

        # while ([position_x,position_y,position_z]!=setpoints[1]):
        #     pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
        #     local_pos_pub.publish(pos)
        #     rate.sleep()
        #     continue
        # print("setpoint 4 reached")

        # while ([position_x,position_y,position_z]!=setpoints[3]):
        #     pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[3][0],setpoints[3][1],setpoints[3][2]
        #     local_pos_pub.publish(pos)
        #     rate.sleep()
        #     continue
        # print("setpoint 5 reached")

        # while ([position_x,position_y,position_z]!=setpoints[4]):
        #     while not stateMt.state.guided:
        #         ofb_ctl.setLand(alt=0)
        #         rate.sleep()
        #     print("Landing!!")
        #     continue
        # print("setpoint 6 reached") 

        # last_request=rospy.get_rostime() 
        # while not ((rospy.get_rostime()-last_request)>rospy.Duration(3)):
        #     continue  
        # ofb_ctl.gripper(value=False)

        # while not stateMt.state.mode=="OFFBOARD":

        #     for i in range(100):
        #         local_pos_pub.publish(pos)
        #         rate.sleep()  

        #     ofb_ctl.offboard_set_mode()
        #     rate.sleep()
        # print ("OFFBOARD mode activated")

        # while ([position_x,position_y,position_z]!=setpoints[3]):
        #     pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[3][0],setpoints[3][1],setpoints[3][2]
        #     local_pos_pub.publish(pos)
        #     rate.sleep()
        #     continue
        # print("setpoint 7 reached")

        # while ([position_x,position_y,position_z]!=setpoints[0]):
        #     pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[0][0],setpoints[0][1],setpoints[0][2]
        #     local_pos_pub.publish(pos)
        #     rate.sleep()
        #     continue
        # print("setpoint 8 reached")

        # ofb_ctl.setParam('COM_DISARM_LAND',4.0)
        # while not stateMt.state.guided:
        #     ofb_ctl.setLand(alt=0)
        #     rate.sleep()
        # print("Landing!!")

        #break    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass