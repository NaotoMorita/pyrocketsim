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
import matplotlib

class Legion():
    def __init__(self):
        pass

class Rocket():
    def __init__(self):
        #時間初期化
        self.t = 0
        self.dt = 0.005
        self.i = 1
        #速度系初期化
        self.vel_b = [[0.01],[0.01],[0.01]]
        self.vel_i = [[0.0],[0.0],[0.0]]
        self.vel_v = [[0.0],[0.0],[0.0]]

        #慣性座標での位置初期化
        self.r_i = [[0.0],[0.0],[0.0]]

        #飛行姿勢初期化
        self.quaternion_b2i = Quaternion()
        self.quaternion_b2i.rot2quat(88 * numpy.pi /180, 0, 1, 0)
        self.dcm_b2i = self.quaternion_b2i.quat2dcm()
        self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
        self.dcm_i2b = self.quaternion_i2b.quat2dcm()

        #速度座標への変換行列初期化
        self.dcm_b2v = numpy.zeros((3,3),dtype = "float_")
        self.dcm_v2b = numpy.zeros((3,3),dtype = "float_")

        #迎え角
        self.alpha = 0.0
        self.beta = 0.0

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

        self.gyro = [[0.0],[0.0],[0.0]]

        #ロガー初期化
        self.log_r     = []
        self.log_vel_i = []
        self.log_vel_b = []
        self.log_vel_v = []

        #積分用ログ初期化
        self.integ_dgyro_dt = []
        self.integ_dq_dt = []
        self.integ_forces_i = []
        self.integ_vel_i = []


        #パラメータ代入
        #TODO : Legionの傘下に入るように
        self.rocketlength = 0.42
        self.sidesurface  = 0.42 * 0.15
        self.frontsurface = 0.015 ** 2 * numpy.pi

        self.Cd = [[0.0],[0.0],[0.0]]
        self.CL = [[0.0],[0.0],[0.0]]

        self.mass = 0.0712
        self.inatia = [[0.0000198,0,0],[0,0.001293,0],[0,0,0.001293]]

    def calcaoa_velocityaxis(self):
        #hstackで行列を創るのでその都度初期化する必要あり
        self.dcm_b2v = numpy.zeros((3,3),dtype = "float_")
        self.dcm_v2b = numpy.zeros((3,3),dtype = "float_")

        #リストからnumpy_ndarrayに変換
        vel_b = numpy.array(self.vel_b, dtype = "float_")

        xd =  vel_b / numpy.sqrt(vel_b[0] ** 2 + vel_b[1] ** 2 + vel_b[2] ** 2)
        xd.shape = 3,1

        yd =  numpy.cross((xd+numpy.array([[0],[0],[10.0]])).T,xd.T).T
        yd /= numpy.sqrt(yd[0,0] ** 2 + yd[1,0] ** 2 + yd[2,0] ** 2)
        yd = yd - numpy.dot(yd.T,xd) * xd

        zd = numpy.cross(xd.T,yd.T).T

        dcm_v2b = numpy.hstack([xd,yd,zd])
        dcm_b2v = numpy.linalg.inv(dcm_v2b)
        self.dcm_v2b = numpy.ndarray.tolist(dcm_v2b)
        self.dcm_b2v = numpy.ndarray.tolist(dcm_b2v)


        #逆進しても力の働く方向は同じであることに注意
        self.alpha = -numpy.arctan(vel_b[2,0] / vel_b[0,0]) * abs(vel_b[0,0]) / vel_b[0,0]
        self.beta  = -numpy.arctan(vel_b[1,0] / vel_b[0,0]) * abs(vel_b[0,0]) / vel_b[0,0]

    def calcdcm_b2i_i2b(self):
        self.dcm_b2i = self.quaternion_b2i.quat2dcm()
        self.dcm_i2b = self.quaternion_i2b.quat2dcm()


    def calc_CL(self):
        #http://repository.dl.itc.u-tokyo.ac.jp/dspace/bitstream/2261/30502/1/sk009003011.pdf より
        CLaoa = 2 #[/rad]
        self.CL[2] = CLaoa * self.alpha
        self.CL[1] = CLaoa * self.beta
        self.CL[0] = 0.0




    def calc_Cd(self):
        CDaoa = (5-0.07)/(24/180*numpy.pi) + 0.07 #[/rad]
        self.Cd[2] =   0.0
        self.Cd[1] =   0.0
        self.Cd[0] =   CDaoa*(self.alpha + self.beta)

    def rocket_force_morment(self):
        #推力カーブを読み込んで推力をリターン
        def thrust(t):
            thrustcurve = numpy.loadtxt("Quest_A8.eng",comments = ";",delimiter = " ")
            return numpy.interp(t,thrustcurve[:,0],thrustcurve[:,1])


        #使用する変数をndarray化
        vel_b = numpy.array(self.vel_b, dtype = "float_")
        dcm_b2v = numpy.array(self.dcm_b2v, dtype = "float_")
        dcm_v2b = numpy.array(self.dcm_v2b, dtype = "float_")
        dcm_b2i = numpy.array(self.dcm_b2i, dtype = "float_")

        vel_v = numpy.dot(dcm_b2v,vel_b)

        rho = 1.184

        #全ての座標系での揚力、抗力、モーメントを求める
        lift_v = numpy.array(self.lift_v)
        lift_v[2] = 1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.CL[2]
        lift_v[1] = 1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.CL[1]
        lift_v[0] = 1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.CL[0]
        self.lift_v = numpy.ndarray.tolist(lift_v)


        lift_b = numpy.array(self.lift_b)
        lift_b = numpy.dot(dcm_v2b,lift_v)
        self.lift_b = numpy.ndarray.tolist(lift_b)

        lift_i = numpy.array(self.lift_i)
        lift_i = numpy.dot(dcm_b2i,lift_b)
        self.lift_i = numpy.ndarray.tolist(lift_i)

        drag_v = numpy.array(self.drag_v,dtype = "float_")
        drag_v[2] = -1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.Cd[2]
        drag_v[1] = -1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.Cd[1]
        drag_v[0] = -1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.Cd[0]
        self.drag_v = numpy.ndarray.tolist(drag_v)


        drag_b = numpy.array(self.drag_b)
        drag_b = numpy.dot(dcm_v2b,drag_v)

        drag_b[2] += 1 / 2 * rho * vel_b[2] ** 2 * self.sidesurface * 0.5
        drag_b[1] += 1 / 2 * rho * vel_b[1] ** 2 * self.sidesurface * 0.5
        drag_b[0] += 1 / 2 * rho * vel_b[0] ** 2 * self.frontsurface * 0.3

        self.drag_b = numpy.ndarray.tolist(drag_b)

        drag_i = numpy.array(self.drag_i)
        drag_i = numpy.dot(dcm_b2i,drag_b)
        self.drag_i = numpy.ndarray.tolist(drag_i)

        moment_b = numpy.array(self.moment_b, dtype = "float_")
        Cm = numpy.array([[0.0],[0.0],[0.0]], dtype = "float_")
        CL = numpy.array(self.CL, dtype = "float_")
        Cm = numpy.dot(dcm_v2b, CL) * 0.15

        print("vel_v")
        print(vel_v)
        moment_b[2] = -1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * Cm[1] * self.rocketlength
        moment_b[1] =  1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * Cm[2] * self.rocketlength
        moment_b[0] =  1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * Cm[0] * 0.0
        print(self.moment_b)
        self.moment_b = numpy.ndarray.tolist(moment_b)

        moment_i = numpy.array(self.moment_i)
        moment_i = numpy.dot(dcm_b2i,moment_b)
        self.moment_i = numpy.ndarray.tolist(moment_i)

        thrust_b = numpy.array(self.thrust_b)
        thrust_b[0] = thrust(self.t)
        thrust_i = numpy.dot(dcm_b2i,thrust_b)
        self.thrust_i = numpy.ndarray.tolist(thrust_i)
        self.thrust_b = numpy.ndarray.tolist(thrust_b)

    def integrate(self,t_start = 0):
        def quad_quad_kari(fourth,integ):
            forth = numpy.array(fourth)
            integ = numpy.array(integ)
            fst = 65 / 4 * (fourth - 3 * integ[-1] + 3 * integ[-2] - integ[-3]) / 6
            snd = 19 / 3 * (-fourth + 4 * integ[-1] - 5 * integ[-2] + 2 * integ[-3]) / 2
            trd =  5 / 2 * (2 * fourth - 9 * integ[-1] + 18 * integ[-2] - 11 * integ[-3]) / 6
            quad_value = (fst + snd + trd + integ[-3]) * self.dt
            return quad_value
        #力に重力の影響付加
        lift_i = numpy.array(self.lift_i)
        drag_i = numpy.array(self.drag_i)
        thrust_i = numpy.array(self.thrust_i)
        vel_i = numpy.array(self.vel_i)
        r_i = numpy.array(self.r_i)

        forces_i = lift_i + drag_i + thrust_i + self.mass * numpy.array([[0.0],[0.0],[9.8]])

        inatia   = numpy.array(self.inatia)
        gyro     = numpy.array(self.gyro)
        #print(gyro)
        moment_i = numpy.array(self.moment_i)
        quat     = numpy.array(self.quaternion_b2i.quat)
        dt = numpy.array(self.dt)

        if self.i == 1:
            vel_i += forces_i / self.mass * self.dt
            r_i   += vel_i * self.dt

            self.vel_i = numpy.ndarray.tolist(vel_i)
            self.r_i = numpy.ndarray.tolist(r_i)

            gyro += numpy.dot(numpy.linalg.inv(inatia),moment_i) * self.dt
            dq_dt = self.quaternion_b2i.dq_dt(gyro[0],gyro[1],gyro[2])
            quat += numpy.array(dq_dt) * dt

            self.gyro = numpy.ndarray.tolist(gyro)
            self.moment_i = numpy.ndarray.tolist(moment_i)
            self.quaternion_b2i.quat = numpy.ndarray.tolist(quat)
            self.quaternion_b2i.normalize()
            self.dcm_b2i = self.quaternion_b2i.quat2dcm()
            self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
            self.quaternion_i2b.normalize()
            self.dcm_i2b = self.quaternion_i2b.quat2dcm()

            self.integ_dgyro_dt.append(numpy.ndarray.tolist(numpy.dot(numpy.linalg.inv(inatia),moment_i)))
            self.integ_dq_dt.append(dq_dt)
            self.integ_forces_i.append(numpy.ndarray.tolist(forces_i / self.mass))
            self.integ_vel_i.append(self.vel_i)


        elif self.i == 2 or self.i == 3:
            vel_i += (forces_i / self.mass + numpy.array(self.integ_forces_i[self.i-2])) / 2 * self.dt
            r_i   += (vel_i + self.vel_i[self.i-2]) / 2 * self.dt

            self.vel_i = numpy.ndarray.tolist(vel_i)
            self.r_i = numpy.ndarray.tolist(r_i)

            gyro += (numpy.dot(numpy.linalg.inv(inatia),moment_i) + self.integ_forces_i[self.i-2]) / 2 * self.dt
            dq_dt = self.quaternion_b2i.dq_dt(gyro[0],gyro[1],gyro[2])
            quat += (numpy.array(dq_dt) + numpy.array(self.integ_dq_dt[self.i-2])) / 2 * dt

            self.gyro = numpy.ndarray.tolist(gyro)
            self.moment_i = numpy.ndarray.tolist(moment_i)
            self.quaternion_b2i.quat = numpy.ndarray.tolist(quat)
            self.quaternion_b2i.normalize()
            self.dcm_b2i = self.quaternion_b2i.quat2dcm()
            self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
            self.quaternion_i2b.normalize()
            self.dcm_i2b = self.quaternion_i2b.quat2dcm()

            self.integ_dgyro_dt.append(numpy.ndarray.tolist(numpy.dot(numpy.linalg.inv(inatia),moment_i)))
            self.integ_dq_dt.append(dq_dt)
            self.integ_forces_i.append(numpy.ndarray.tolist(forces_i / self.mass))
            self.integ_vel_i.append(self.vel_i)


        else:
            vel_i += quad_quad_kari(forces_i / self.mass, self.integ_forces_i)
            r_i   += quad_quad_kari(vel_i,self.integ_vel_i)

            self.vel_i = numpy.ndarray.tolist(vel_i)
            self.r_i = numpy.ndarray.tolist(r_i)

            gyro += quad_quad_kari(numpy.dot(numpy.linalg.inv(inatia),moment_i),self.integ_dgyro_dt)
            dq_dt = self.quaternion_b2i.dq_dt(gyro[0],gyro[1],gyro[2])
            quat += quad_quad_kari(numpy.array(dq_dt),self.integ_dq_dt)

            self.gyro = numpy.ndarray.tolist(gyro)
            self.moment_i = numpy.ndarray.tolist(moment_i)
            self.quaternion_b2i.quat = numpy.ndarray.tolist(quat)
            self.quaternion_b2i.normalize()
            self.dcm_b2i = self.quaternion_b2i.quat2dcm()
            self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
            self.quaternion_i2b.normalize()
            self.dcm_i2b = self.quaternion_i2b.quat2dcm()

            self.integ_dgyro_dt.append(numpy.ndarray.tolist(numpy.dot(numpy.linalg.inv(inatia),moment_i)))
            self.integ_dq_dt.append(dq_dt)
            self.integ_forces_i.append(numpy.ndarray.tolist(forces_i / self.mass))
            self.integ_vel_i.append(self.vel_i)

            self.integ_dgyro_dt.pop(0)
            self.integ_dq_dt.pop(0)
            self.integ_forces_i.pop(0)
            self.integ_vel_i.pop(0)




        self.i += 1
        self.t += self.dt


    def redifinitions(self):
        vel_i = numpy.array(self.vel_i)
        r_i = numpy.array(self.r_i)


        vel_b = numpy.dot(numpy.array(self.dcm_i2b),vel_i)
        self.vel_b = numpy.ndarray.tolist(vel_b)
        self.quaternion_b2i.normalize()
        self.quaternion_i2b.normalize()












def main():

    rocket = Rocket()
    for i in range(700):
        rocket.calcaoa_velocityaxis()
        rocket.calc_CL()
        rocket.calc_Cd()
        rocket.calcdcm_b2i_i2b()
        rocket.rocket_force_morment()
        rocket.integrate()
        rocket.redifinitions()









if __name__ == '__main__':
    main()
