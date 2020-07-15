#!/usr/bin/env python

import rospy
from tesi.msg import Matriceacc
import numpy as np
from geometry_msgs.msg import Pose
from tf_conversions import posemath
import PyKDL


class arduino_filter(object):
    def __init__(self):
        self.filter_size = 17
        self.alx = np.zeros(shape=(self.filter_size,1))
        self.aly = np.zeros(shape=(self.filter_size,1))
        self.alz = np.zeros(shape=(self.filter_size,1))
        self.aax = np.zeros(shape=(self.filter_size,1))
        self.aay = np.zeros(shape=(self.filter_size,1))
        self.aaz = np.zeros(shape=(self.filter_size,1))
        self.alx_filt = 0.0
        self.aly_filt = 0.0
        self.alz_filt = 0.0
        self.aax_filt = 0.0
        self.aay_filt = 0.0
        self.aaz_filt = 0.0
        self.listener()
        self.robot_pose = PyKDL.Frame()
        self.pub = rospy.Publisher('accel_filt', Matriceacc, queue_size=10)

    def listener(self):
        rospy.Subscriber('accelerazioni', Matriceacc, self.callback)
        rospy.Subscriber('/comau_smart_six/cartesian_pose',Pose,self.poseCallback)

    def callback(self,acc):
        self.alx=np.delete(self.alx, 0)
        self.aly=np.delete(self.aly, 0)
        self.alz=np.delete(self.alz, 0)
        self.aax=np.delete(self.aax, 0)
        self.aay=np.delete(self.aay, 0)
        self.aaz=np.delete(self.aaz, 0)
        self.alx=np.append(self.alx, acc.acc_lin_x.data)
        self.aly=np.append(self.aly, acc.acc_lin_y.data)
        self.alz=np.append(self.alz, acc.acc_lin_z.data)
        self.aax=np.append(self.aax, acc.acc_ang_x.data)
        self.aay=np.append(self.aay, acc.acc_ang_y.data)
        self.aaz=np.append(self.aaz, acc.acc_ang_z.data)


    def poseCallback(self,pose):
        self.robot_pose = posemath.fromMsg(pose)

    def getGravityVector(self):
        print self.robot_pose.M*PyKDL.Vector(0,0,9.8067)
        return self.robot_pose.M*PyKDL.Vector(0,0,9.8067)
        


    def filtro_mediana(self,data):
        temp = []
        indexer = self.filter_size//2
        for b in range(len(data)):
            for z in range(self.filter_size):
                if b+z-indexer<0 or b+z-indexer>len(data)-1:
                    temp.append(0)
                else:
                    temp.append(data[b+z-indexer])
            temp.sort()       
        return temp[len(temp)//2]
    
    def publisher(self):
        self.rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            #Apply median filter
            self.alx_filt = self.filtro_mediana(self.alx)
            self.aly_filt = self.filtro_mediana(self.aly)
            self.alz_filt = self.filtro_mediana(self.alz)
            self.aax_filt = self.filtro_mediana(self.aax)
            self.aay_filt = self.filtro_mediana(self.aay)
            self.aaz_filt = self.filtro_mediana(self.aaz)

            #Find gravity correction term
            gravity_offset = self.getGravityVector()
            self.alx_filt = self.alx_filt - gravity_offset[0]
            self.aly_filt = self.aly_filt - gravity_offset[1]
            self.alz_filt = self.alz_filt - gravity_offset[2]

            #Define and publish filtered acceleration
            print ("Accelerazioni filtrate")
            accel_filt = Matriceacc()
            accel_filt.acc_lin_x.data = self.alx_filt
            accel_filt.acc_lin_y.data = self.aly_filt
            accel_filt.acc_lin_z.data = self.alz_filt
            accel_filt.acc_ang_x.data = self.aax_filt
            accel_filt.acc_ang_y.data = self.aay_filt
            accel_filt.acc_ang_z.data = self.aaz_filt
            rospy.loginfo(accel_filt)
            self.pub.publish(accel_filt)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('publisherarduino', anonymous=True)
    arduino = arduino_filter()
    arduino.publisher()
