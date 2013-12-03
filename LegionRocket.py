#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:  多数のロケットの一斉打ち上げ及びジャイロデータ解析ツール(GUI)
#
# Author:      NaotoMORITA
#
# Created:     03/12/2013
# Copyright:   (c) NaotoMORITA 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------

import numpy
from NaQuaternion import Quaternion

class Legion():
    def __init__(self):
        pass

class Rocket():
    def __init__(self):
        #速度系初期化
        self.vel_b = [[0.0],[0.0],[0.0]]
        self.vel_i = [[0.0],[0.0],[0.0]]
        self.vel_v = [[0.0],[0.0],[0.0]]

        #慣性座標での位置初期化
        self.r_i = [[0.0],[0.0],[0.0]]

        #飛行姿勢初期化
        self.quaternion_b2i = Quaternion()
        self.dcm_b2i = self.quaternion_b2i.quat2dcm()
        self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
        self.dcm_i2b = self.quaternion_i2b.quat2dcm()

        #速度座標への変換行列初期化
        self.dcm_b2v = numpy.zeros((3,3),dtype = "f")
        self.dcm_v2b = numpy.zeros((3,3),dtype = "f")

        #空力初期化
        self.lift_b   = [[0.0],[0.0],[0.0]]
        self.drag_b   = [[0.0],[0.0],[0.0]]
        self.moment_b = [[0.0],[0.0],[0.0]]
        self.thrust_b = [[0.0],[0.0],[0.0]]

        self.lift_i   = [[0.0],[0.0],[0.0]]
        self.drag_i   = [[0.0],[0.0],[0.0]]
        self.moment_i = [[0.0],[0.0],[0.0]]
        self.thrust_i = [[0.0],[0.0],[0.0]]

        self.lift_v   = [[0.0],[0.0],[0.0]]
        self.drag_v   = [[0.0],[0.0],[0.0]]
        self.moment_v = [[0.0],[0.0],[0.0]]
        self.thrust_v = [[0.0],[0.0],[0.0]]

        #ロガー初期化
        self.log_r     = []
        self.log_vel_i = []
        self.log_vel_b = []
        self.log_vel_v = []

        #パラメータ代入
        #TODO : Legionの傘下に入るように
        self.rocketlength = 0.42
        self.sidesurface  = 0.42 * 0.15
        self.frontsurface = 0.015**2 * numpy.pi

        self.Cd = 0.0
        self.CL = 0.0
        self.Cm = 0.0

        self.mass = 0.0712
        self.inatia = [[0.001,0,0],[0,0.0001,0],[0,0,0.0001]]

    def calc_CL(self):
        pass
    def calc_Cm(self):
        pass
    def calc_Cd(self):
        pass

    def rocket_force_morment(self):
        pass
    def integrate(self,t_start = 0):
        pass



def main():
    rocket = Rocket()





if __name__ == '__main__':
    main()
