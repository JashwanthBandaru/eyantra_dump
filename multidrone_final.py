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
import threading

rospy.init_node('multidrone', anonymous=True)

class pick_n_place1:

    def __init__(self):
        pass
    
    def setArm(self, value=True):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('edrone0/mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('edrone0/mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(value)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 

    def setLand(self):
        rospy.wait_for_service('edrone0/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('edrone0/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService()
        except rospy.ServiceException as e:
            print("Service land call failed: %s"%e)   

    def setTakeoff(self,alt):
        rospy.wait_for_service('edrone0/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('edrone0/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = alt)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)          


    def setParam(self,param, value):
        rospy.wait_for_service('edrone0/mavros/param/set')     
        
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)
        try:
            paramService = rospy.ServiceProxy('edrone0/mavros/param/set', mavros_msgs.srv.ParamSet)
            paramService(param_id=param, value=val)
        except rospy.ServiceException as e:
            print("Service param set failed: %s"%e)                            

   
    def offboard_set_mode(self):
        #pass

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        
        rospy.wait_for_service('edrone0/mavros/set_mode') 
        try:
            setModeService = rospy.ServiceProxy('edrone0/mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print ("Service setmode call failed: %s"%e)

    def gripper(self,value):
        
        rospy.wait_for_service('/edrone0/activate_gripper') 
        try:
            gripperService = rospy.ServiceProxy('/edrone0/activate_gripper', Gripper)
            result = gripperService(activate_gripper=value)
            return result

        except rospy.ServiceException as e:
            print ("Service gripper call failed: %s"%e)

class pick_n_place2:

    def __init__(self):
        pass 
    
    def setArm(self, value=True):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('edrone1/mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('edrone1/mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(value)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 

    def setLand(self):
        rospy.wait_for_service('edrone1/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('edrone1/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService()
        except rospy.ServiceException as e:
            print("Service land call failed: %s"%e)   

    def setTakeoff(self,alt):
        rospy.wait_for_service('edrone1/mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy('edrone1/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService(altitude = alt)
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s"%e)          


    def setParam(self,param, value):
        rospy.wait_for_service('edrone1/mavros/param/set')     
        
        if isinstance(value, float):
            val = ParamValue(integer=0, real=value)
        else:
            val = ParamValue(integer=value, real=0.0)
        try:
            paramService = rospy.ServiceProxy('edrone1/mavros/param/set', mavros_msgs.srv.ParamSet)
            paramService(param_id=param, value=val)
        except rospy.ServiceException as e:
            print("Service param set failed: %s"%e)                            

   
    def offboard_set_mode(self):
        #pass

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
        
        rospy.wait_for_service('edrone1/mavros/set_mode') 
        try:
            setModeService = rospy.ServiceProxy('edrone1/mavros/set_mode', mavros_msgs.srv.SetMode)
            setModeService(custom_mode="OFFBOARD")

        except rospy.ServiceException as e:
            print ("Service setmode call failed: %s"%e)

    def gripper(self,value):
        
        rospy.wait_for_service('edrone1/activate_gripper') 
        try:
            gripperService = rospy.ServiceProxy('edrone1/activate_gripper', Gripper)
            result = gripperService(activate_gripper=value)
            return result

        except rospy.ServiceException as e:
            print ("Service gripper call failed: %s"%e)            
        

class stateMoniter1:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def pose_callback(self,msg1):
        global position0_x,position0_y,position0_z,position0_x1,position0_y1,position0_z1,position0_z2,position0_x2,position0_y2,position0_z3
        position0_x = round(msg1.pose.position.x)
        position0_y = round(msg1.pose.position.y)
        position0_z = round(msg1.pose.position.z)

        position0_x1 = round(msg1.pose.position.x,1)
        position0_y1 = round(msg1.pose.position.y,1)
        position0_z1 = round(msg1.pose.position.z,1)
        position0_z2 = round(msg1.pose.position.z,4)
        position0_x2 = round(msg1.pose.position.x,4)
        position0_y2 = round(msg1.pose.position.y,4)
        position0_z3 = round(msg1.pose.position.z,2)

    def gripper_callback(self,msg2):
        global gripper_bool_value0
        gripper_bool_value0=msg2.data 

class stateMoniter2:
    def __init__(self):
        self.state = State()
        # Instantiate a setpoints message
        self.sp = PositionTarget()
        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    
    def pose_callback(self,msg1):
        global position_x,position_y,position_z,position_x1,position_y1,position_z1,position_z2,position_x2,position_y2,position_z3
        position_x = round(msg1.pose.position.x)
        position_y = round(msg1.pose.position.y)
        position_z = round(msg1.pose.position.z)

        position_x1 = round(msg1.pose.position.x,1)
        position_y1 = round(msg1.pose.position.y,1)
        position_z1 = round(msg1.pose.position.z,1)
        position_z2 = round(msg1.pose.position.z,4)
        position_x2 = round(msg1.pose.position.x,4)
        position_y2 = round(msg1.pose.position.y,4)
        position_z3 = round(msg1.pose.position.z,2)

    def gripper_callback(self,msg2):
        global gripper_bool_value
        gripper_bool_value=msg2.data   


class image_proc1():

    def __init__(self):
        self.Image=0
        self.image_sub = rospy.Subscriber("edrone0/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        self.bridge = CvBridge()
        self.img = np.empty([])    

    def image_callback(self, data):
        global img_aruco0
        try:
            img_aruco0 = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image

        except CvBridgeError as e:
            print(e)
            return

    def process(self,data):
        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) 
        Detected_ArUco_markers = {}
        
        if ids != None:
            for i in range(len(ids)):
                Detected_ArUco_markers[ids[i][0]]=corners[i]
            
            for ids, corner1 in Detected_ArUco_markers.items():
                corner = corner1[0]

                # Since angle is atan2(-y,x), then converting that to degrees
                top_right_angle = (math.degrees(math.atan2(-corner[1][1] + corner[3][1], corner[1][0] - corner[3][0]))) % 360
                angle = (top_right_angle + 45) % 360

                center0_x=int((corner[0][0]+corner[2][0]+corner[1][0]+corner[3][0])/4)
                center0_y=int((corner[0][1]+corner[2][1]+corner[3][1]+corner[1][1])/4)

                print(f"{ids},{center0_x},{center0_y},{int(angle)}")  

                return (center0_x, center0_y)
        else:
            return "No_aruco"

class image_proc2():

    def __init__(self):
        self.Image=0
        self.image_sub = rospy.Subscriber("edrone1/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
        self.bridge = CvBridge()
        self.img = np.empty([])    

    def image_callback(self, data):
        global img_aruco
        try:
            img_aruco = self.bridge.imgmsg_to_cv2(data, "bgr8") # Converting the image to OpenCV standard image

        except CvBridgeError as e:
            print(e)
            return

    def process(self,data):
        gray = cv2.cvtColor(data, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = parameters) 
        Detected_ArUco_markers = {}
        
        if ids != None:
            for i in range(len(ids)):
                Detected_ArUco_markers[ids[i][0]]=corners[i]
            
            for ids, corner1 in Detected_ArUco_markers.items():
                corner = corner1[0]

                # Since angle is atan2(-y,x), then converting that to degrees
                top_right_angle = (math.degrees(math.atan2(-corner[1][1] + corner[3][1], corner[1][0] - corner[3][0]))) % 360
                angle = (top_right_angle + 45) % 360

                center_x=int((corner[0][0]+corner[2][0]+corner[1][0]+corner[3][0])/4)
                center_y=int((corner[0][1]+corner[2][1]+corner[3][1]+corner[1][1])/4)

                print(f"{ids},{center_x},{center_y},{int(angle)}")  

                return (center_x, center_y)
        else:
            return "No_aruco"            

def drone1():

    stateMt = stateMoniter1()
    ofb_ctl = pick_n_place1()
    aruco_detection=image_proc1()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('edrone0/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('edrone0/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    d1_intial=[-1,1]
    setpoints = [[0.0,0.0,3.0],[0.0,16.0,3.0],[25.0,16.0,3.0],[round(14.7-d1_intial[0],1),-7.4-d1_intial[1],1.7],[0.0,24.0,3.0],[25.0,24.0,3.0],[round(16.4-d1_intial[0],1),-7.4-d1_intial[1],1.7]]#List to setpoints

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
    rospy.Subscriber("edrone0/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("edrone0/mavros/local_position/pose",PoseStamped, stateMt.pose_callback)
    rospy.Subscriber("/edrone0/gripper_check",String, stateMt.gripper_callback)

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
    # ofb_ctl.setParam('MPC_LAND_SPEED',0.2)

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":

        for i in range(50):
            local_pos_pub.publish(pos)
            rate.sleep()  

        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")

    def get_setpoint(sp_y):

        aruco_value_returned =aruco_detection.process(img_aruco0)
        if aruco_value_returned !='No_aruco':
            global center0_x, center0_y
            center0_x,center0_y=aruco_value_returned[0],aruco_value_returned[1]

            print("can land")
            box_x,box_y,box_z = round(position0_x2+(position0_z2*math.tan(math.radians(40))/200)*(center0_x-200),1),sp_y,position0_z1
            box_x1,box_y1,box_z1 = round(position0_x2+(position0_z2*math.tan(math.radians(40))/200)*(center0_x-200)),sp_y,position0_z

            while ([position0_x1,position0_y1,position0_z1]!=[box_x,box_y,box_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x,box_y,box_z
                local_pos_pub.publish(pos)
                rate.sleep()
                continue

            return (box_x,box_y,box_z,box_x1,box_y1,box_z1)
        else:
            return None 

    def move_to_setpoint(sp_x,sp_y,sp_z,accuracy=False):

        if accuracy==False:
            while ([position0_x,position0_y,position0_z]!=[sp_x,sp_y,sp_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x,sp_y,sp_z
                local_pos_pub.publish(pos)
                rate.sleep()
                continue       
        elif accuracy == True:
            while ([position0_x1,position0_y1,position0_z1]!=[sp_x,sp_y,sp_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x,sp_y,sp_z
                local_pos_pub.publish(pos)
                rate.sleep()
                continue

    def land_drone(pos_z=0.0):
        while (position0_z1!=pos_z):
            while not stateMt.state.guided:
                ofb_ctl.setLand()
                rate.sleep()
            #print("Landing!!")
            continue

    def land_drone_ofb(sp_x,sp_y,sp_z):
        while (position0_z3!=sp_z):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x, sp_y, sp_z
            local_pos_pub.publish(pos)
            rate.sleep()
            continue
            

    def offboard_drone():
        while not stateMt.state.mode=="OFFBOARD":

            for i in range(30):
                local_pos_pub.publish(pos)
                rate.sleep()  

            ofb_ctl.offboard_set_mode()
            rate.sleep()

    def delaying_drone(time):  
        last_request=rospy.get_rostime() 
        while not ((rospy.get_rostime()-last_request)>rospy.Duration(time)):
            continue   

    def correcting_box_pos(sp_x,sp_y,sp_z):

        move_to_setpoint(sp_x,sp_y,sp_z, accuracy=True)
        setpoint_box =get_setpoint(sp_y)    

        while (True):
            if center0_x >=199 and center0_x <=201:
                break
            else:    
                while (True):    
                    setpoint_box =get_setpoint(sp_y)
                    if setpoint_box != None:
                        break  

        box_x,box_y,box_z,box_x1,box_y1,box_z1=setpoint_box[0],sp_y,setpoint_box[2],setpoint_box[3],sp_y,setpoint_box[5]  

        # land_drone()
        # delaying_drone(time=3) 
        land_drone_ofb(box_x,box_y,0.00)

        return box_x,box_y,box_z,box_x1,box_y1,box_z1                   

    def drone_search_box(sp_x,sp_y,sp_z):

        x=3.0
        while(True):

            while ([position0_x,position0_y,position0_z]!=[x,sp_y,sp_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= x,sp_y,sp_z
                local_pos_pub.publish(pos)
                rate.sleep()

                setpoint_box =get_setpoint(sp_y)
                if setpoint_box != None:
                    break
            if setpoint_box != None:
                break        
            x+=3.0    

        # while ([position0_x1,position0_y1,position0_z1]!=[sp_x,sp_y,sp_z]):
        #     pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= x,sp_y,sp_z
        #     local_pos_pub.publish(pos)
        #     rate.sleep()

        #     setpoint_box =get_setpoint(sp_y)
        #     if setpoint_box != None:
        #         break

        #     x+=1.0    

        while (True):
            if center0_x >=199 and center0_x <=201:
                break
            else:    
                while (True):    
                    setpoint_box =get_setpoint(sp_y)
                    if setpoint_box != None:
                        break
    
        box_x,box_y,box_z,box_x1,box_y1,box_z1=setpoint_box[0],sp_y,setpoint_box[2],setpoint_box[3],sp_y,setpoint_box[5]  

        # land_drone()
        # delaying_drone(time=3) 
        land_drone_ofb(box_x,box_y,0.00)

        while (str(gripper_bool_value0)=='False'):
            print(gripper_bool_value0)
            
            # offboard_drone()
            # print ("OFFBOARD mode activated")

            box_x,box_y,box_z,box_x1,box_y1,box_z1 = correcting_box_pos(box_x,sp_y,sp_z)

        while (not ofb_ctl.gripper(value=True).result):
            
            # offboard_drone()    
            # print ("OFFBOARD mode activated")

            box_x,box_y,box_z,box_x1,box_y1,box_z1 = correcting_box_pos(box_x,sp_y,sp_z)

            while (str(gripper_bool_value0)=='False'):
                print(gripper_bool_value0)

                # offboard_drone()
                # print ("OFFBOARD mode activated")

                box_x,box_y,box_z,box_x1,box_y1,box_z1 = correcting_box_pos(box_x,sp_y,sp_z)

        ofb_ctl.gripper(value=True)  

        # offboard_drone()    
        # print ("OFFBOARD mode activated")

        move_to_setpoint(box_x1,box_y1,box_z1)    
               
    # Publish the setpoints 
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''
        move_to_setpoint(setpoints[0][0],setpoints[0][1],setpoints[0][2])

        move_to_setpoint(setpoints[1][0],setpoints[1][1],setpoints[1][2],accuracy= True)
       
        #ofb_ctl.setParam('MPC_LAND_SPEED',0.2)
        drone_search_box(setpoints[2][0],setpoints[2][1],setpoints[2][2])

        move_to_setpoint(setpoints[3][0],setpoints[3][1],5.0, accuracy = True)

        #ofb_ctl.setParam('MPC_LAND_SPEED',0.3)
        # land_drone(pos_z=setpoints[3][2])
        # delaying_drone(time=3)
        land_drone_ofb(setpoints[3][0],setpoints[3][1],1.70)

        ofb_ctl.gripper(value=False)

        #offboard_drone()    

        move_to_setpoint(round(setpoints[3][0]),round(setpoints[3][1]),5.0)    

        move_to_setpoint(setpoints[4][0],setpoints[4][1],setpoints[4][2], accuracy=True) 

        #ofb_ctl.setParam('MPC_LAND_SPEED',0.2)
        drone_search_box(setpoints[5][0],setpoints[5][1],setpoints[5][2])

        move_to_setpoint(setpoints[6][0],setpoints[6][1],5.0, accuracy= True) 

        #ofb_ctl.setParam('MPC_LAND_SPEED',0.3)
        # land_drone(pos_z=setpoints[3][2])
        # delaying_drone(time=3)
        land_drone_ofb(setpoints[6][0],setpoints[6][1],1.70)

        ofb_ctl.gripper(value=False)

        #offboard_drone()    

        move_to_setpoint(setpoints[6][0], setpoints[6][1],5.0)
        
        move_to_setpoint(setpoints[0][0],setpoints[0][1],setpoints[0][2])

        ofb_ctl.setParam('COM_DISARM_LAND',1.0)
        land_drone()
        delaying_drone(time=3)

        break   

def drone2():

    stateMt = stateMoniter2()
    ofb_ctl = pick_n_place2()
    aruco_detection=image_proc2()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('edrone1/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('edrone1/mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    
    d1_intial=[-1,61]
    setpoints = [[0.0,0.0,3.0],[0.0,-12.0,3.0],[25.0,-12.0,3.0],[round(57.35-d1_intial[0],1),round(64.75-d1_intial[1],1),1.7],[0.0,-32.0,3.0],[25.0,-32.0,3.0],[round(59.05-d1_intial[0],1),round(64.75-d1_intial[1],1),1.7]]#List to setpoints

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
    rospy.Subscriber("edrone1/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("edrone1/mavros/local_position/pose",PoseStamped, stateMt.pose_callback)
    rospy.Subscriber("edrone1/gripper_check",String, stateMt.gripper_callback)

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
    #ofb_ctl.setParam('MPC_LAND_SPEED',0.3)

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":

        for i in range(50):
            local_pos_pub.publish(pos)
            rate.sleep()  

        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
    
    def get_setpoint(sp_y):

        aruco_value_returned =aruco_detection.process(img_aruco)
        if aruco_value_returned !='No_aruco':
            global center_x, center_y
            center_x,center_y=aruco_value_returned[0],aruco_value_returned[1]

            print("can land")
            box_x,box_y,box_z = round(position_x2+(position_z2*math.tan(math.radians(40))/200)*(center_x-200),1),sp_y,position_z1
            box_x1,box_y1,box_z1 = round(position_x2+(position_z2*math.tan(math.radians(40))/200)*(center_x-200)),sp_y,position_z

            while ([position_x1,position_y1,position_z1]!=[box_x,box_y,box_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x,box_y,box_z
                local_pos_pub.publish(pos)
                rate.sleep()
                continue

            return (box_x,box_y,box_z,box_x1,box_y1,box_z1)
        else:
            return None 

    def move_to_setpoint(sp_x,sp_y,sp_z,accuracy=False):

        if accuracy==False:
            while ([position_x,position_y,position_z]!=[sp_x,sp_y,sp_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x,sp_y,sp_z
                local_pos_pub.publish(pos)
                rate.sleep()
                continue       
        elif accuracy == True:
            while ([position_x1,position_y1,position_z1]!=[sp_x,sp_y,sp_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x,sp_y,sp_z
                local_pos_pub.publish(pos)
                rate.sleep()
                continue

    def land_drone(pos_z=0.0):
        while (position_z1!=pos_z):
            while not stateMt.state.guided:
                ofb_ctl.setLand()
                rate.sleep()
            #print("Landing!!")
            continue

    def land_drone_ofb(sp_x,sp_y,sp_z):
        while (position_z3!=sp_z):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x, sp_y, sp_z
            local_pos_pub.publish(pos)
            rate.sleep()
            continue        

    def offboard_drone():
        while not stateMt.state.mode=="OFFBOARD":

            for i in range(30):
                local_pos_pub.publish(pos)
                rate.sleep()  

            ofb_ctl.offboard_set_mode()
            rate.sleep()

    def delaying_drone(time):  
        last_request=rospy.get_rostime() 
        while not ((rospy.get_rostime()-last_request)>rospy.Duration(time)):
            continue 

    def correcting_box_pos(sp_x,sp_y,sp_z):

        move_to_setpoint(sp_x,sp_y,sp_z, accuracy=True)
        setpoint_box =get_setpoint(sp_y)    

        while (True):
            if center_x >=199 and center_x <=201:
                break
            else:    
                while (True):    
                    setpoint_box =get_setpoint(sp_y)
                    if setpoint_box != None:
                        break  

        box_x,box_y,box_z,box_x1,box_y1,box_z1=setpoint_box[0],sp_y,setpoint_box[2],setpoint_box[3],sp_y,setpoint_box[5]  

        # land_drone()
        # delaying_drone(time=3) 
        land_drone_ofb(box_x,box_y,0.00)

        return box_x,box_y,box_z,box_x1,box_y1,box_z1                   

    def drone_search_box(sp_x,sp_y,sp_z):

        x=3.0
        while(True):
            
            while ([position_x,position_y,position_z]!=[x,sp_y,sp_z]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= x,sp_y,sp_z
                local_pos_pub.publish(pos)
                rate.sleep()

                setpoint_box =get_setpoint(sp_y)
                if setpoint_box != None:
                    break
            if setpoint_box != None:
                break        
            x+=3.0

        # while ([position_x1,position_y1,position_z1]!=[sp_x,sp_y,sp_z]):
        #     pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x,sp_y,sp_z
        #     local_pos_pub.publish(pos)
        #     rate.sleep()

        #     setpoint_box =get_setpoint(sp_y)
        #     if setpoint_box != None:
        #         break

        while (True):
            if center_x >=199 and center_x <=201:
                break
            else:    
                while (True):    
                    setpoint_box =get_setpoint(sp_y)
                    if setpoint_box != None:
                        break
    

        box_x,box_y,box_z,box_x1,box_y1,box_z1=setpoint_box[0],sp_y,setpoint_box[2],setpoint_box[3],sp_y,setpoint_box[5]  

        # land_drone()
        # delaying_drone(time=3) 
        land_drone_ofb(box_x,box_y,0.00)
        

        while (str(gripper_bool_value)=='False'):
            print(gripper_bool_value)
            
            # offboard_drone()
            # print ("OFFBOARD mode activated")

            box_x,box_y,box_z,box_x1,box_y1,box_z1 = correcting_box_pos(box_x,sp_y,sp_z)

        while (not ofb_ctl.gripper(value=True).result):
            
            # offboard_drone()    
            # print ("OFFBOARD mode activated")

            box_x,box_y,box_z,box_x1,box_y1,box_z1 = correcting_box_pos(box_x,sp_y,sp_z)

            while (str(gripper_bool_value)=='False'):
                print(gripper_bool_value)

                # offboard_drone()
                # print ("OFFBOARD mode activated")

                box_x,box_y,box_z,box_x1,box_y1,box_z1 = correcting_box_pos(box_x,sp_y,sp_z)

        ofb_ctl.gripper(value=True)  

        # offboard_drone()    
        # print ("OFFBOARD mode activated")

        move_to_setpoint(box_x1,box_y1,box_z1)    
                            
               
    # Publish the setpoints 
    while not rospy.is_shutdown():
        '''
        Step 1: Set the setpoint 
        Step 2: Then wait till the drone reaches the setpoint, 
        Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
        Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


        Write your algorithm here 
        '''
        move_to_setpoint(setpoints[0][0],setpoints[0][1],setpoints[0][2])

        move_to_setpoint(setpoints[1][0],setpoints[1][1],setpoints[1][2],accuracy= True)

        # ofb_ctl.setParam('MPC_LAND_SPEED',0.2)
        drone_search_box(setpoints[2][0],setpoints[2][1],setpoints[2][2])

        move_to_setpoint(setpoints[3][0],setpoints[3][1],5.0, accuracy = True)
   
        # ofb_ctl.setParam('MPC_LAND_SPEED',0.3)
        # land_drone(pos_z=setpoints[3][2])
        # delaying_drone(time=3)
        land_drone_ofb(setpoints[3][0],setpoints[3][1],1.70)

        ofb_ctl.gripper(value=False)

        # offboard_drone()    
        # print ("OFFBOARD mode activated")

        move_to_setpoint(round(setpoints[3][0]),round(setpoints[3][1]),5.0)    

        move_to_setpoint(setpoints[4][0],setpoints[4][1],setpoints[4][2], accuracy=True) 

        # ofb_ctl.setParam('MPC_LAND_SPEED',0.2)
        drone_search_box(setpoints[5][0],setpoints[5][1],setpoints[5][2])

        move_to_setpoint(setpoints[6][0],setpoints[6][1],5.0, accuracy= True) 

        # ofb_ctl.setParam('MPC_LAND_SPEED',0.3)
        # land_drone(pos_z=setpoints[3][2])
        # delaying_drone(time=3)
        land_drone_ofb(setpoints[6][0],setpoints[6][1],1.70)

        ofb_ctl.gripper(value=False)

        # offboard_drone()    

        move_to_setpoint(setpoints[6][0], setpoints[6][1],5.0, accuracy= True)
        
        move_to_setpoint(setpoints[0][0],setpoints[0][1],setpoints[0][2])

        ofb_ctl.setParam('COM_DISARM_LAND',1.0)
        land_drone()
        delaying_drone(time=3)

        break

def main():
    #t1 = threading.Thread(target=drone1)
    t2 = threading.Thread(target=drone2)
   
    #t1.start()
    t2.start()

    #t1.join()
    t2.join()                  

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass