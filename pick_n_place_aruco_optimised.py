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

    def setLand(self):
        rospy.wait_for_service('mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            landService()
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
        global position_x,position_y,position_z,position_x1,position_y1,position_z1,position_z2,position_x2,position_y2
        position_x = round(msg1.pose.position.x)
        position_y = round(msg1.pose.position.y)
        position_z = round(msg1.pose.position.z)

        position_x1 = round(msg1.pose.position.x,1)
        position_y1 = round(msg1.pose.position.y,1)
        position_z1 = round(msg1.pose.position.z,1)
        position_z2 = round(msg1.pose.position.z,4)
        position_x2 = round(msg1.pose.position.x,4)
        position_y2 = round(msg1.pose.position.y,4)

    def gripper_callback(self,msg2):
        global gripper_bool_value
        gripper_bool_value=msg2.data  

class image_proc():

    def __init__(self):
        self.Image=0
        self.image_sub = rospy.Subscriber("/iris/camera/image_raw", Image, self.image_callback) #Subscribing to the camera topic
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
    #ofb_ctl.setParam('MPC_LAND_SPEED',0.3)

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":

        for i in range(100):
            local_pos_pub.publish(pos)
            rate.sleep()  

        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
    
    def get_setpoint():

        aruco_value_returned =aruco_detection.process(img_aruco)
        if aruco_value_returned !='No_aruco':
            global center_x, center_y
            center_x,center_y=aruco_value_returned[0],aruco_value_returned[1]

            print("can land")
            box_x,box_y,box_z = round(position_x2+(position_z2*math.tan(math.radians(40))/200)*(center_x-200),1),position_y1,position_z1
            box_x1,box_y1,box_z1 = round(position_x2+(position_z2*math.tan(math.radians(40))/200)*(center_x-200)),position_y,position_z

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

    def land_drone():
        while (position_z1!=0):
            while not stateMt.state.guided:
                ofb_ctl.setLand()
                rate.sleep()
            #print("Landing!!")
            continue

    def offboard_drone():
        while not stateMt.state.mode=="OFFBOARD":

            for i in range(100):
                local_pos_pub.publish(pos)
                rate.sleep()  

            ofb_ctl.offboard_set_mode()
            rate.sleep()

    def delaying_drone(time):  
        last_request=rospy.get_rostime() 
        while not ((rospy.get_rostime()-last_request)>rospy.Duration(time)):
            continue 

    def drone_search_box(sp_x,sp_y,sp_z):

        while ([position_x1,position_y1,position_z]!=[sp_x,sp_y,sp_z]):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= sp_x,sp_y,sp_z
            local_pos_pub.publish(pos)
            rate.sleep()

            setpoint_box =get_setpoint()
            if setpoint_box != None:
                break

        while (True):
            if center_x >=200 and center_x <=200:
                break
            else:    
                while (True):    
                    setpoint_box =get_setpoint()
                    if setpoint_box != None:
                        break        

        box_x,box_y,box_z,box_x1,box_y1,box_z1=setpoint_box[0],setpoint_box[1],setpoint_box[2],setpoint_box[3],setpoint_box[4],setpoint_box[5]

        while ([position_x1,position_y1,position_z1]!=[box_x,box_y,1.5]):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= box_x,box_y,1.5
            local_pos_pub.publish(pos)
            rate.sleep()

            setpoint_box =get_setpoint()
            if setpoint_box != None:
                break

        while (True):
            if center_x >=200 and center_x <=200:
                break
            else:    
                while (True):    
                    setpoint_box =get_setpoint()
                    if setpoint_box != None:
                        break        

        box_x,box_y,box_z,box_x1,box_y1,box_z1=setpoint_box[0],setpoint_box[1],setpoint_box[2],setpoint_box[3],setpoint_box[4],setpoint_box[5]

        # move_to_setpoint(box_x,box_y,1.0, accuracy=True)
        # print("setpoint: 1m above the box reached")

        land_drone()
        print("setpoint 3 reached")

        delaying_drone(time=5)     

        while (str(gripper_bool_value)=='False'):
            print(gripper_bool_value)
            
            offboard_drone()
            print ("OFFBOARD mode activated")

            
            move_to_setpoint(box_x,box_y,box_z, accuracy=True)
            print("setpoint 2 reached")

            move_to_setpoint(box_x,box_y, 0.5, accuracy=True)
            print("reached 0.5m above the ground")
            
            land_drone()    
            print("setpoint 3 reached")

            delaying_drone(time=5)    

        while (not ofb_ctl.gripper(value=True).result):
            
            offboard_drone()    
            print ("OFFBOARD mode activated")

            move_to_setpoint(box_x,box_y,box_z,accuracy=True)
            print("setpoint 2 reached")

            move_to_setpoint(box_x,box_y, 0.5, accuracy=True)
            print("reached 0.5m above the ground")

            land_drone()    
            print("setpoint 3 reached")

            delaying_drone(time=5)    

            while (str(gripper_bool_value)=='False'):
                print(gripper_bool_value)

                offboard_drone()
                print ("OFFBOARD mode activated")

                move_to_setpoint(box_x,box_y,box_z, accuracy=True)    
                print("setpoint 2 reached")

                move_to_setpoint(box_x,box_y,0.5, accuracy=True)
                print("reached 0.5m above the ground")

                land_drone()    
                print("setpoint 3 reached")

                delaying_drone(time=5) 

        ofb_ctl.gripper(value=True)  

        offboard_drone()    
        print ("OFFBOARD mode activated")

        move_to_setpoint(box_x1,box_y1,box_z1)    
        print("setpoint 4 reached")                               
               


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
        print("setpoint 1 reached")  

        drone_search_box(setpoints[1][0],setpoints[1][1],setpoints[1][2])

        move_to_setpoint(setpoints[1][0],setpoints[1][1],setpoints[1][2],accuracy=True)    
        print("setpoint 5 reached")

        move_to_setpoint(setpoints[1][0],setpoints[1][1],0.5, accuracy=True) 
        print("reached 0.5m above the ground")

        land_drone()      
        print("setpoint 6 reached")

        delaying_drone(time=5)    

        ofb_ctl.gripper(value=False)

        offboard_drone()    
        print ("OFFBOARD mode activated")

        move_to_setpoint(setpoints[1][0],setpoints[1][1],setpoints[1][2])    
        print("setpoint 7 reached")

        move_to_setpoint(setpoints[0][0],setpoints[0][1],setpoints[0][2],accuracy=True)    
        print("setpoint 8 reached")

        move_to_setpoint(setpoints[0][0],setpoints[0][1],0.5,accuracy=True)
        print("reached 0.5m above the ground")

        ofb_ctl.setParam('COM_DISARM_LAND',4.0)
        land_drone()

        break      

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass