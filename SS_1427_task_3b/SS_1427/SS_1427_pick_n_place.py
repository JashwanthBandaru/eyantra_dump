#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import ParamValue
from std_msgs.msg import *
from gazebo_ros_link_attacher.srv import Gripper

class pick_n_place:

    def __init__(self):
        # Initialise rosnode
        rospy.init_node('pick_n_place', anonymous=True)

    
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


def main():

    stateMt = stateMoniter()
    ofb_ctl = pick_n_place()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [[0.0,0.0,3.0],[3.0,0.0,3.0],[3.0,0.0,0.0],[3.0,3.0,3.0],[3.0,3.0,0.0]] #List to setpoints

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
            local_pos_pub.publish(pos)
            rate.sleep()
            continue
        print("setpoint 2 reached")
        
        while ([position_x,position_y,position_z1]!=setpoints[2]):
            while not stateMt.state.guided:
                ofb_ctl.setLand(alt=0)
                rate.sleep()
            print("Landing!!")
            continue
        print("setpoint 3 reached")

        last_request=rospy.get_rostime() 

        while not ((rospy.get_rostime()-last_request)>rospy.Duration(8)):
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

            while ([position_x1,position_y1,position_z]!=setpoints[1]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
                local_pos_pub.publish(pos)
                rate.sleep()
                continue
            print("setpoint 2 reached")
            
            while ([position_x,position_y,position_z1]!=setpoints[2]):
                while not stateMt.state.guided:
                    ofb_ctl.setLand(alt=0)
                    rate.sleep()
                print("Landing!!")
                continue
            print("setpoint 3 reached")

            last_request=rospy.get_rostime()
            while not ((rospy.get_rostime()-last_request)>rospy.Duration(8)):
                continue 

        ####
        while (not ofb_ctl.gripper(value=True).result):
            while not stateMt.state.mode=="OFFBOARD":

                for i in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()  

                ofb_ctl.offboard_set_mode()
                rate.sleep()
            print ("OFFBOARD mode activated")

            while ([position_x1,position_y1,position_z]!=setpoints[1]):
                pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
                local_pos_pub.publish(pos)
                rate.sleep()
                continue
            print("setpoint 2 reached")
            
            while ([position_x,position_y,position_z1]!=setpoints[2]):
                while not stateMt.state.guided:
                    ofb_ctl.setLand(alt=0)
                    rate.sleep()
                print("Landing!!")
                continue
            print("setpoint 3 reached")

            last_request=rospy.get_rostime() 

            while not ((rospy.get_rostime()-last_request)>rospy.Duration(8)):
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

                while ([position_x1,position_y1,position_z]!=setpoints[1]):
                    pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[1][0],setpoints[1][1],setpoints[1][2]
                    local_pos_pub.publish(pos)
                    rate.sleep()
                    continue
                print("setpoint 2 reached")
                
                while ([position_x,position_y,position_z1]!=setpoints[2]):
                    while not stateMt.state.guided:
                        ofb_ctl.setLand(alt=0)
                        rate.sleep()
                    print("Landing!!")
                    continue
                print("setpoint 3 reached")

                last_request=rospy.get_rostime()
                while not ((rospy.get_rostime()-last_request)>rospy.Duration(8)):
                    continue 

        ofb_ctl.gripper(value=True)
        #####

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
        print("setpoint 4 reached")

        while ([position_x,position_y,position_z]!=setpoints[3]):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[3][0],setpoints[3][1],setpoints[3][2]
            local_pos_pub.publish(pos)
            rate.sleep()
            continue
        print("setpoint 5 reached")

        while ([position_x,position_y,position_z]!=setpoints[4]):
            while not stateMt.state.guided:
                ofb_ctl.setLand(alt=0)
                rate.sleep()
            print("Landing!!")
            continue
        print("setpoint 6 reached") 

        last_request=rospy.get_rostime() 
        while not ((rospy.get_rostime()-last_request)>rospy.Duration(8)):
            continue  
        ofb_ctl.gripper(value=False)

        while not stateMt.state.mode=="OFFBOARD":

            for i in range(100):
                local_pos_pub.publish(pos)
                rate.sleep()  

            ofb_ctl.offboard_set_mode()
            rate.sleep()
        print ("OFFBOARD mode activated")

        while ([position_x,position_y,position_z]!=setpoints[3]):
            pos.pose.position.x, pos.pose.position.y, pos.pose.position.z= setpoints[3][0],setpoints[3][1],setpoints[3][2]
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

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass