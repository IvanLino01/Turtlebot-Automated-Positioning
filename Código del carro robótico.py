#!/usr/bin/env python3
from pickle import TRUE
import rospy 
from nav_msgs.msg import Odometry  
from geometry_msgs.msg import Twist  
from tf.transformations import euler_from_quaternion
# Import message types needed
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
# Import numpy for receiving int8 status from visp
from rospy.numpy_msg import numpy_msg
# Bring in the SimpleActionClient
import actionlib
# Bring in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import numpy as np 
from std_msgs.msg import String
from nav_msgs.msg import Odometry 



class OdomReaderClass():
    def __init__(self):
        rospy.on_shutdown(self.cleanup) 
        ###* INIT PUBLISHERS *### 
        #pub = rospy.Publisher('setPoint', UInt16MultiArray, queue_size=1) 
        self.cmd_vel_pub= rospy.Publisher('cmd_vel', Twist, queue_size=1) 
        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("odom", Odometry, self.odom_cb) 
        rospy.Subscriber('/aruco_single/pose',PoseStamped,self.ReceiveVispPosition)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_base) 
         
        rospy.on_shutdown(self.cleanup) 
        # Create a simple action client called "move_base" with action definition file "MoveBaseAction"
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Wait until the action server has started up and started listening for goals.
        self.client.wait_for_server()
        # Set the goal: define the x_goal, y_goal positions and yaw angle  positions w.r.t "map" coordinate frame 
        # You can use the set_goal(x_goal,y_goal, yaw) function, I have prepared for you. 
        pub = rospy.Publisher('place', String, queue_size=10)     

        ############################### SUBSCRIBERS ##################################### 
        rospy.Subscriber("place", String, self.string_cb) 
       
        ############ CONSTANTS AND VARIABLES################ 
       
        self.odom = Odometry()
        self.yaw=0
        self.goal=MoveBaseGoal()
        self.text="stop"
        flag=0
        flag2=0
        flag3=0
        flag4=0

        self.window=[2.76, 1.97, 3.14]
        self.my_vel= Twist()
        self.vispPosition = PoseStamped()

        self.yaw=0
        self.my_vel.linear.x= 0
        # Sends the goal to the action server.
        self.client.send_goal(self.goal,feedback_cb=self.my_feedback_cb)  
        self.feedback_flag=0

        #* INIT NODE *### 
        r = rospy.Rate(20) #1Hz 
        print("Node initialized 20hz")
        print("Waiting goal")
        print("\n")
        
        while not rospy.is_shutdown(): 
    
            if flag2==1:
                print("Im on the goal: ", self.text)
                print("\n")

                flag=0
                flag2=0
                flag3=1  
                pub.publish("") 

            if flag3==1 and flag4==1:
                objectX=(self.vispPosition.pose.position.x+0.05)
                print("X: ", objectX)
                objectZ=self.vispPosition.pose.position.z
                print("Z: ", objectZ)
                print("\n")
    
                if (objectX>0.020000000000 or objectX<(-0.02000000000)) or (objectZ>0.50000000000 or objectZ<0.480000000000):
                    self.my_vel.angular.z = -3*objectX
                    self.cmd_vel_pub.publish(self.my_vel)
                    print("Girando")
                    if objectZ>0.500000000000 or objectZ<0.450000000000:
                        self.my_vel.linear.x = 0.8*(objectZ-.50)
                        self.cmd_vel_pub.publish(self.my_vel)
                        print("Avanzando")

                else:
                    print("Find")
                    self.my_vel.linear.x = 0 #m/seg
                    self.my_vel.angular.z = 0 #rad/seg
                    self.cmd_vel_pub.publish(self.my_vel) 
                    flag3=0
               

            if self.text=="window" and self.client.get_state() != 1 and flag==0:
                self.goal=self.set_goal(self.window[0],self.window[1],self.window[2])
                self.client.send_goal(self.goal)
                print("I'm going to the window")
                print("Robot state: ", self.client.get_state())
                flag=1
                flag4=1


            if self.text=="home" and self.client.get_state() != 1 and flag==0:
                self.goal=self.set_goal(0.19, 0.067, 0)
                self.client.send_goal(self.goal)
                print("I'm going to the home")
                print("Robot state: ", self.client.get_state())
                flag=1
                flag4=0


            if self.text=="set_window":
                new_x=self.actual_pose.pose.pose.position.x
                new_y=self.actual_pose.pose.pose.position.y
                self.window[0]=new_x
                self.window[1]=new_y
                pub.publish("")

            if self.text=="stop":
                self.client.cancel_goal()
                self.my_vel.linear.x = 0 #m/seg
                self.my_vel.angular.z = 0 #rad/seg
                self.cmd_vel_pub.publish(self.my_vel)
                pub.publish("")
                flag=0 
                

            if self.client.get_state() == 3 and flag==1:
                flag2=1
                
            if self.text == "adelante":
                self.my_vel.linear.x = 0.3
                self.my_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.my_vel) 
            elif self.text == "atras":
                self.my_vel.linear.x = -0.3
                self.my_vel.angular.z = 0.0
                self.cmd_vel_pub.publish(self.my_vel) 
            elif self.text == "izquierda":
                self.my_vel.linear.x = 0.0
                self.my_vel.angular.z = 0.3
                self.cmd_vel_pub.publish(self.my_vel) 
            elif self.text == "derecha":
                self.my_vel.linear.x = 0.0
                self.my_vel.angular.z = -0.3
                self.cmd_vel_pub.publish(self.my_vel) 


            r.sleep() 

    def odom_cb(self, msg): 
        ## This function receives a number  
        self.odom=msg
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion (orientation_list)
        
    def ReceiveVispStatus(self,status):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		#rospy.loginfo(status.data)
        self.vispStatus = status.data
		#rospy.loginfo(vispStatus)
		
    def ReceiveVispPosition(self,position):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		#rospy.loginfo("!!!!! VISP POSITION RECEIVED")
        self.vispPosition = position
		#rospy.loginfo(position)
		
    def pose_base(self,pose):
		# Although there is a lot of data in this packet, we're only interested in the state at the moment	
		#rospy.loginfo("!!!!! VISP POSITION RECEIVED")
        self.actual_pose = pose
		#rospy.loginfo(position)

    def string_cb(self, text_msg): 
        ## This function receives a number  
        self.text = text_msg.data

    def my_feedback_cb(self, robot_pose):
        #print("Im in the feedback function")
        print(robot_pose)
        self.pose=robot_pose.base_position.pose #Currente robot pose
        self.feedback_flag=1
    
    def set_goal(self, x_goal, y_goal, yaw):
        # Creates a new goal with the MoveBaseGoal constructor
        # it receives x_goal and y_goal [m] and yaw [rad]
        # All the coordinates are measured with respect to the "map" frame. 
        # It returns a MoveBaseGoal pose.
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Position
        goal.target_pose.pose.position.x = x_goal
        goal.target_pose.pose.position.y = y_goal
        # Rotation of the mobile base frame w.r.t. "map" frame as a quaternion
        quat = quaternion_from_euler(0,0,yaw)
        goal.target_pose.pose.orientation.x = quat[0]
        goal.target_pose.pose.orientation.y = quat[1]
        goal.target_pose.pose.orientation.z = quat[2]
        goal.target_pose.pose.orientation.w = quat[3]
        return goal
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.
        
        self.my_vel.linear.x = 0 #m/seg
        self.cmd_vel_pub.publish(self.my_vel)

        self.my_vel.angular.z = 0 #rad/seg
        self.cmd_vel_pub.publish(self.my_vel)   
        print("\n")  
        print("APAGANDO SISTEMA")
 
      
############################### MAIN PROGRAM #################################### 
if __name__ == '__main__':
    rospy.init_node("odom_reader", anonymous=TRUE) 
    OdomReaderClass()