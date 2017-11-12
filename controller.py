#!/usr/bin/env python
import rospy
import json
from something import MotorDemand
from pid import PIDControl

class Controller():

    def __init__():

        self.euler_angles = (0,0,0)
        self.external_demands = [0,0,0,0,0,0]
        self.motor_demands = [0,0,0,0,0,0]
        self.depth_demand = 0
        self.yaw_demand = 0
        self.pitch_demand = 0
        self.row_demand = 0

        rospy.init_node('cauv_controller')
        self.pub = rospy.Publisher('motor_demand', String)
        rospy.Subscriber("imu_euler_angles", String, euler_angle_callback)
        rospy.Subscriber("external_demands", String, external_demand_callback)
        rospy.Subscriber("depth_demand", String, lambda x: self.depth_demand = x.data)
        rospy.Subscriber("pitch_demand", String, lambda x: self.pitch_demand = x.data)
        rospy.Subscriber("yaw_demand", String, lambda x: self.yaw_demand = x.data)
        rospy.Subscriber("row_demand", String, lambda x: self.row_demand = x.data)

        self.rate = rospy.Rate(10) # hz
        self.yaw_pid = PIDControl()
        self.pitch_pid = PIDControl()
        self.roll_pid = PIDControl()
        self.depth_pid = PIDControl()

    def euler_angle_callback(data)
        print(data)
        self.euler_angles = (0,0,0)

    def external_demand_callback(data):
        print(data)
        self.external_demands = [0,0,0,0,0,0]

    def add_depth_motor_demands():
        depth_demand = self.depth_pid.getDemand()
        self.depth_demands[1] += depth_demand
        self.depth_demands[2] -= depth_demand

    def add_yaw_motor_demands():
        yaw_demand = self.yaw_pid.getDemand()
        self.yaw_demands[1] += yaw_demand
        self.yaw_demands[2] -= yaw_demand

    def add_pitch_motor_demands():
        pitch_demand = self.pitch_pid.getDemand()
        self.pitch_demands[3] += pitch_demand
        self.pitch_demands[4] -= pitch_demand

    def add_roll_motor_demands():
        roll_demand = self.roll_pid.getDemand()
        self.motor_demands[5] += roll_demand
        self.motor_demands[6] -= roll_demand

    def add_external_demands():
        for i in range(6):
            self.motor_demands[i] += self.external_demands[i]


    def run():
        while not rospy.is_shutdown():

            self.motor_demands = [0,0,0,0,0,0]

            self.add_yaw_demands()
            self.add_pitch_demands()
            self.add_row_demands()
            self.add_external_demands()
            
#
            motor_demand_msg = MotorDemand()
            motor_demand_msg.motor0 = self.motor_demands[0]
            motor_demand_msg.motor1 = self.motor_demands[0]
            motor_demand_msg.motor2 = self.motor_demands[0]
            motor_demand_msg.motor3 = self.motor_demands[0]
            motor_demand_msg.motor4 = self.motor_demands[0]
            motor_demand_msg.motor5 = self.motor_demands[0]

            rospy.loginfo(motor_demand_msg)
            pub.publish(motor_demand_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
