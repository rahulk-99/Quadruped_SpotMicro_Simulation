import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Path
from collections import deque
import math
import numpy as np

class QuadrupedControllerNode(Node):
    def __init__(self):
        super().__init__("quadruped_controller_node")
        self.joint_position_pub = self.create_publisher(
            Float64MultiArray, "/position_controller/commands", 10
        )
        self.timer = self.create_timer(0.1, self.control_function)
        self.publisher_ = self.create_publisher(Path, 'robot_path', 10)
        self.t = 0
        self.T = 24
        self.step_length = 0.04
        self.step_height = 0.08
        self.omega = 2*math.pi/self.T
        self.l2 = 0.10921
        self.l3 = 0.13528
        self.height_from_ground = math.sqrt(self.l2**2 + self.l3**2)
        self.length = 2*0.0969619
        self.width = 2*0.03900714
        self.shift_constant = 0.02
        self.waypoints =[]
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        self.leg_origin_map = {"front_left":[self.length/2, self.width/2, 0],
                               "rear_left":[-self.length/2, self.width/2, 0],
                               "front_right":[self.length/2, -self.width/2, 0],
                               "rear_right":[-self.length/2, -self.width/2, 0]}
        self.action_list = [self.move_forward] * 5 + [self.rotate_clockwise]*5 + [self.rotate_anticlockwise]*5 + [self.lshift]*5 + [self.rshift]*5
        self.action_list = deque(self.action_list)
    def control_function(self): 
        if len(self.action_list) == 0:
            rclpy.shutdown()     
        angles = Float64MultiArray()
        angles.data = self.action_list[0]()
        self.joint_position_pub.publish(angles)
        self.t +=1
        if self.t > self.T:
            self.t = 0
            self.action_list.popleft()
            
            
    def move_forward(self):
        front_right_angles = self.inverse_kinematics(self.leg_origin_map["front_right"],self.get_coordinates(self.t,"front_right","forward"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t+self.T/2),2)
        rear_right_angles = self.inverse_kinematics(self.leg_origin_map["rear_right"],self.get_coordinates(self.t +self.T/2,"rear_right","forward"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t),3)
        front_left_angles = self.inverse_kinematics(self.leg_origin_map["front_left"],self.get_coordinates(self.t + self.T/2,"front_left","forward"),False)#self.forward_motion_inv(self.x_and_z_at_t(self.t),0)
        rear_left_angles = self.inverse_kinematics(self.leg_origin_map["rear_left"],self.get_coordinates(self.t,"rear_left","forward"),False) #self.forward_motion_inv(self.x_and_z_at_t(self.t + self.T/2),1)
        return front_right_angles +rear_right_angles +front_left_angles + rear_left_angles
    
    def lshift(self):
        front_right_angles = self.inverse_kinematics(self.leg_origin_map["front_right"],self.get_coordinates(self.t,"front_right","lshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t+self.T/2),2)
        rear_right_angles = self.inverse_kinematics(self.leg_origin_map["rear_right"],self.get_coordinates(self.t ,"rear_right","lshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t),3)
        front_left_angles = self.inverse_kinematics(self.leg_origin_map["front_left"],self.get_coordinates(self.t + self.T/2,"front_left","lshift"),False)#self.forward_motion_inv(self.x_and_z_at_t(self.t),0)
        rear_left_angles = self.inverse_kinematics(self.leg_origin_map["rear_left"],self.get_coordinates(self.t +self.T/2,"rear_left","lshift"),False) #self.forward_motion_inv(self.x_and_z_at_t(self.t + self.T/2),1)
        return front_right_angles +rear_right_angles +front_left_angles + rear_left_angles
    
    def rshift(self):
        front_right_angles = self.inverse_kinematics(self.leg_origin_map["front_right"],self.get_coordinates(self.t,"front_right","rshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t+self.T/2),2)
        rear_right_angles = self.inverse_kinematics(self.leg_origin_map["rear_right"],self.get_coordinates(self.t ,"rear_right","rshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t),3)
        front_left_angles = self.inverse_kinematics(self.leg_origin_map["front_left"],self.get_coordinates(self.t + self.T/2,"front_left","rshift"),False)#self.forward_motion_inv(self.x_and_z_at_t(self.t),0)
        rear_left_angles = self.inverse_kinematics(self.leg_origin_map["rear_left"],self.get_coordinates(self.t +self.T/2,"rear_left","rshift"),False) #self.forward_motion_inv(self.x_and_z_at_t(self.t + self.T/2),1)
        return front_right_angles +rear_right_angles +front_left_angles + rear_left_angles
    
    def rotate_clockwise(self):
        front_right_angles = self.inverse_kinematics(self.leg_origin_map["front_right"],self.get_coordinates(self.t,"front_right","rshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t+self.T/2),2)
        rear_right_angles = self.inverse_kinematics(self.leg_origin_map["rear_right"],self.get_coordinates(self.t + self.T/2 ,"rear_right","lshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t),3)
        front_left_angles = self.inverse_kinematics(self.leg_origin_map["front_left"],self.get_coordinates(self.t + self.T/2 ,"front_left","rshift"),False)#self.forward_motion_inv(self.x_and_z_at_t(self.t),0)
        rear_left_angles = self.inverse_kinematics(self.leg_origin_map["rear_left"],self.get_coordinates(self.t ,"rear_left","lshift"),False) #self.forward_motion_inv(self.x_and_z_at_t(self.t + self.T/2),1)
        return front_right_angles +rear_right_angles +front_left_angles + rear_left_angles
    
    def rotate_anticlockwise(self):
        front_right_angles = self.inverse_kinematics(self.leg_origin_map["front_right"],self.get_coordinates(self.t,"front_right","lshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t+self.T/2),2)
        rear_right_angles = self.inverse_kinematics(self.leg_origin_map["rear_right"],self.get_coordinates(self.t + self.T/2 ,"rear_right","rshift"),True)#self.forward_motion_inv(self.x_and_z_at_t(self.t),3)
        front_left_angles = self.inverse_kinematics(self.leg_origin_map["front_left"],self.get_coordinates(self.t + self.T/2 ,"front_left","lshift"),False)#self.forward_motion_inv(self.x_and_z_at_t(self.t),0)
        rear_left_angles = self.inverse_kinematics(self.leg_origin_map["rear_left"],self.get_coordinates(self.t ,"rear_left","rshift"),False) #self.forward_motion_inv(self.x_and_z_at_t(self.t + self.T/2),1)
        return front_right_angles +rear_right_angles +front_left_angles + rear_left_angles
    
    def distance(self,p1,p2):
        return math.sqrt(sum([ (p1[i] -p2[i])**2 for i in range(3)]))
    
    def inverse_kinematics(self,shoulder_origin,target,isright):
        xs,ys,zs = shoulder_origin
        xt,yt,zt = target
        l1 = 0.0255
        l2 = 0.10921
        l3 = 0.13528
        A = self.distance([xt,yt,0],[xs,ys,0])
        if isright:
            hypotenuse_angle_with_horizontal = math.atan2(yt-ys,xt-xs)
            triangle_internal_angle = math.acos(l1/A)
            value = hypotenuse_angle_with_horizontal - triangle_internal_angle
            theta1 = value + math.pi/2
            x_d = xs + l1*math.sin(theta1)
            y_d = ys - l1*math.cos(theta1)
            z_d = zs
            P = self.distance([x_d,y_d,z_d],[xt,yt,zt])
            z = zt - z_d
            M = self.distance([x_d,y_d,z_d + z],[xt,yt,zt])
            theta2 = math.pi - math.atan2(M,z) - math.acos( (l2**2 + P**2 -l3 **2)/(2*l2*P) )
            theta3 = math.pi - math.acos((l3**2 + l2**2 -P**2)/(2*l3*l2) )
            theta2 = theta2 -  math.pi/4# returning with offset removed
            theta3 = theta3 - math.pi/2
        else:
            value = math.atan2(yt-ys,xt-xs) + math.acos(l1/A)
            theta1 = value - math.pi/2
            x_d = xs + l1*math.sin(-theta1)
            y_d = ys + l1*math.cos(theta1)
            z_d = zs
            P = self.distance([x_d,y_d,z_d],[xt,yt,zt])
            z = zt - z_d
            M = self.distance([x_d,y_d,z_d + z],[xt,yt,zt])
            theta2 = -(math.pi - math.atan2(M,z) - math.acos( (l2**2 + P**2 -l3 **2)/(2*l2*P)))
            theta3 = -(math.pi - math.acos((l3**2 + l2**2 -P**2)/(2*l3*l2) ) )
            theta2 = theta2 +  math.pi/4 # returning with offset removed
            theta3 = theta3 + math.pi/2
        return theta1,theta2,theta3
    

    
    def get_coordinates(self,t,leg,action):
        if action == "forward":
            z = self.step_length * (1 - math.cos(self.omega*t)) + self.leg_origin_map[leg][2]
            y = self.leg_origin_map[leg][1]
            x = self.height_from_ground - max(self.step_height*math.sin(self.omega*t),0.0) + self.leg_origin_map[leg][0]
        if action == "lshift":
            z = self.leg_origin_map[leg][2]
            y = self.leg_origin_map[leg][1] + self.shift_constant * (1 - math.cos(self.omega*t)) 
            x = self.height_from_ground - max(self.step_height*math.sin(self.omega*t),0.0) + self.leg_origin_map[leg][0]
            
        if action == "rshift":
            z = self.leg_origin_map[leg][2]
            y = self.leg_origin_map[leg][1] - self.shift_constant * (1 - math.cos(self.omega*t)) 
            x = self.height_from_ground - max(self.step_height*math.sin(self.omega*t),0.0) + self.leg_origin_map[leg][0]
        
        return [x,y,z]          

def main(args=None):
    
    rclpy.init(args=args)

    command_publisher = QuadrupedControllerNode()

    command_publisher.control_function()
    rclpy.spin(command_publisher)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    command_publisher.destroy_node()
    rclpy.shutdown()
    


if __name__ == "__main__":
    main()
