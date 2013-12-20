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
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Legion():
    def __init__(self):
        pass

class Rocket():
    def __init__(self):
        #時間初期化
        self.t = 0
        self.dt = 0.002
        self.i = 1
        #速度系初期化
        self.vel_b = [[0.0],[0.0],[0.0]]
        self.vel_i = [[0.0],[0.0],[0.0]]
        self.vel_v = [[0.0],[0.0],[0.0]]

        #慣性座標での位置初期化
        self.r_i = [[0.0],[0.0],[0.0]]

        #飛行姿勢初期化
        self.quaternion_b2i = Quaternion()
        self.quaternion_b2i.rot2quat(80 * numpy.pi /180, 0, 1, 0)
        self.quaternion_b2i.normalize()
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
        self.thrust_i = [[0.0],[0.0],[0.0]]

        self.lift_v   = [[0.0],[0.0],[0.0]]
        self.drag_v   = [[0.0],[0.0],[0.0]]
        self.moment_v = [[0.0],[0.0],[0.0]]

        self.gyro = [[0.0],[0.0],[0.0]]

        #風
        self.wind = [[0.0],[2.0],[0.0]]

        #減衰係数
        self.K = [[0.1],[0.1],[0.1]]
        self.Ko = 2.5

        #ロガー初期化
        self.log_r     = [[],[],[]]
        self.log_vel_i = [[],[],[]]
        self.log_vel_b = [[],[],[]]
        self.log_vel_v = [[],[],[]]
        self.log_euler = [[],[],[]]
        self.log_force_i = [[],[],[]]
        self.log_lift_i = [[],[],[]]
        self.log_quat = []

        self.log_t = []
        self.log_alpha = []
        self.log_beta = []


        #積分用ログ初期化
        self.integ_dgyro_dt = []
        self.integ_dq_dt = []
        self.integ_forces_i = []
        self.integ_vel_i = []


        #パラメータ代入
        #TODO : Legionの傘下に入るように
        self.rocketlength = 0.32
        self.sidesurface  = 0.32 * 0.015
        self.frontsurface = 0.015 ** 2 * numpy.pi

        self.Cd = [[0.0],[0.0],[0.0]]
        self.CL = [[0.0],[0.0],[0.0]]

        self.mass = 0.0712
        self.inatia = [[0.0000198],[0.001293],[0.001293]]

        #パラシュート
        self.parashoot_t = 20
        self.parashoot_surface = 0.1 ** 2 * numpy.pi

    def calcaoa_velocityaxis(self):
        #hstackで行列を創るのでその都度初期化する必要あり
        #主要となる咆哮を定義する必要があるかも
        self.dcm_b2v = numpy.zeros((3,3),dtype = "float_")
        self.dcm_v2b = numpy.zeros((3,3),dtype = "float_")

        #リストからnumpy_ndarrayに変換
        vel_b = numpy.array(self.vel_b, dtype = "float_")

        if vel_b[0] != 0:
            xd =  vel_b / numpy.sqrt(vel_b[0] ** 2 + vel_b[1] ** 2 + vel_b[2] ** 2)
            yd =  numpy.cross((xd+numpy.array([[0],[0],[10.0]])).T,xd.T).T
            yd /= numpy.sqrt(yd[0,0] ** 2 + yd[1,0] ** 2 + yd[2,0] ** 2)
            yd = yd - numpy.dot(yd.T,xd) * xd

            zd = numpy.cross(xd.T,yd.T).T

            dcm_v2b = numpy.hstack([xd,yd,zd])
            dcm_b2v = numpy.linalg.inv(dcm_v2b)
            self.dcm_v2b = numpy.ndarray.tolist(dcm_v2b)
            self.dcm_b2v = numpy.ndarray.tolist(dcm_b2v)
        else:
            dcm_v2b = numpy.identity(3)
            dcm_b2v = numpy.identity(3)
            self.dcm_v2b = numpy.ndarray.tolist(dcm_v2b)
            self.dcm_b2v = numpy.ndarray.tolist(dcm_b2v)


        #逆進しても力の働く方向は同じであることに注意
##        if self.i == 1:
        if numpy.sqrt(vel_b[0,0] ** 2 + vel_b[1,0] ** 2 + vel_b[2,0] ** 2) != 0:
            self.alpha = numpy.arcsin(vel_b[2,0] / numpy.sqrt(vel_b[0,0] ** 2 + vel_b[1,0] ** 2 + vel_b[2,0] ** 2))
        else:
            self.alpha = 0
        if vel_b[0,0] != 0:
            self.beta  = numpy.arctan(vel_b[1,0] / vel_b[0,0])
        else:
            self.beta = 0

        #迎え角の急激な変化による相対迎え角を定義
        if self.i != 1:
            self.alpha -= self.Ko * self.rocketlength * (self.alpha - self.log_alpha[-1])
            self.beta -= self.Ko * self.rocketlength * (self.beta - self.log_beta[-1])
        #self.alpha2 = numpy.arcsin(-dcm_v2b[2,0])
        #self.beta2 = numpy.arctan(dcm_v2b[1,0] / dcm_v2b[0,0] )
        #self.alpha = (self.alpha2 + self.alpha) / 2
        #self.beta = (self.beta2 + self.alpha) / 2
    def calcdcm_b2i_i2b(self):
        self.quaternion_b2i.normalize()
        self.dcm_b2i = self.quaternion_b2i.quat2dcm()
        self.quaternion_i2b.normalize()
        self.dcm_i2b = self.quaternion_i2b.quat2dcm()


    def calc_CL(self):
        #http://repository.dl.itc.u-tokyo.ac.jp/dspace/bitstream/2261/30502/1/sk009003011.pdf より
        #失速を考慮すること　過大なCLは高迎え角時推力となってしまう
        CLaoa = 0.3 #[/rad]
        #if abs(self.alpha) <= 40:
        self.CL[2] = CLaoa * abs(self.alpha)
        #else:
            #self.CL[2] = (- 0.01 * CLaoa * (abs(self.alpha) - 40) + CLaoa * 40)

        #if abs(self.beta) <= 40:
        self.CL[1] = CLaoa * abs(self.beta)
        #else:
            #self.CL[1] = (- 0.01 * CLaoa * (abs(self.beta) - 40) + CLaoa * 40)
        self.CL[0] = 0.0

    def calc_Cd(self):
        CDaoa = 0.3
        self.Cd[2] =   0.0
        self.Cd[1] =   0.0
        self.Cd[0] =   CDaoa * (abs(self.alpha) + abs(self.beta))

        if self.t >= self.parashoot_t:
            self.Cd[0] = 1.3




    def rocket_force_morment(self):
        #推力カーブを読み込んで推力をリターン
        def thrust(t):
            #thrustcurve = numpy.loadtxt("Quest_A8.eng",comments = ";",delimiter = " ")
            #return numpy.interp(t,thrustcurve[:,0],thrustcurve[:,1])
            if t <= 0.1:
                return 10 / 0.1 * self.t
            else:
                return 10 * numpy.exp(-100*(t-0.1))


        #使用する変数をndarray化
        vel_b = numpy.array(self.vel_b, dtype = "float_")
        dcm_b2v = numpy.array(self.dcm_b2v, dtype = "float_")
        dcm_v2b = numpy.array(self.dcm_v2b, dtype = "float_")
        dcm_b2i = numpy.array(self.dcm_b2i, dtype = "float_")

        vel_v = numpy.dot(dcm_b2v,vel_b)
        self.vel_v = numpy.ndarray.tolist(vel_v)

        rho = 1.184

        #全ての座標系での揚力、抗力、モーメントを求める
        lift_v = numpy.array(self.lift_v)
        if vel_b[2] != 0.0:
            lift_v[2] = -1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.CL[2] * abs(vel_b[2]) / vel_b[2]
        else:
            lift_v[2] = 0.0

        if vel_b[1] != 0.0:
            lift_v[1] = -1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.CL[1] * abs(vel_b[1]) / vel_b[1]
        else:
            lift_v[1] = 0.0
        if vel_b[0] != 0.0:
            lift_v[0] = 1 / 2 * rho * vel_v[0] ** 2 * self.sidesurface * self.CL[0] * abs(vel_b[0]) / vel_b[0]
        else:
            lift_v[0] = 0.0

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

        drag_b[2] -= 0#1 / 2 * rho * vel_b[2] ** 2 * self.sidesurface * 0.5 * abs(vel_b[2]) / vel_b[2]
        drag_b[1] -= 0#1 / 2 * rho * vel_b[1] ** 2 * self.sidesurface * 0.5 * abs(vel_b[1]) / vel_b[1]
        if vel_b[0] != 0:
            drag_b[0] -= 1 / 2 * rho * vel_b[0] ** 2 * self.frontsurface * 1.0 * abs(vel_b[0]) / vel_b[0]

        self.drag_b = numpy.ndarray.tolist(drag_b)

        drag_i = numpy.array(self.drag_i)
        drag_i = numpy.dot(dcm_b2i,drag_b)
        self.drag_i = numpy.ndarray.tolist(drag_i)

        moment_b = numpy.array(self.moment_b, dtype = "float_")
        Cm = numpy.array([[0.0],[0.0],[0.0]], dtype = "float_")
        CL = numpy.array(self.CL, dtype = "float_")
        Cd = numpy.array(self.Cd, dtype = "float_")
        Cm = numpy.dot(dcm_v2b, (CL - Cd)) * 0.28

        #Cdによる復元力を考慮すること。
        moment_b[2] = -(drag_b[1,0] + lift_b[1,0]) * 0.35 * self.rocketlength
        moment_b[1] =  (drag_b[2,0] + lift_b[2,0]) * 0.35 * self.rocketlength
        moment_b[0] =  0.0
        self.moment_b = numpy.ndarray.tolist(moment_b)

        thrust_b = numpy.array(self.thrust_b)
        thrust_b[0,0] = thrust(self.t)
        thrust_i = numpy.dot(dcm_b2i,thrust_b)
        self.thrust_i = numpy.ndarray.tolist(thrust_i)
        self.thrust_b = numpy.ndarray.tolist(thrust_b)

    def integrate(self,t_start = 0):
        def quad_quad_kari(self,fourth,integ):
            fourth = numpy.array(fourth,dtype = "float_")
            integ = numpy.array(integ,dtype = "float_")
            fst = 65 / 4 * (fourth - 3 * integ[-1] + 3 * integ[-2] - integ[-3]) / 6
            snd = 19 / 3 * (-fourth + 4 * integ[-1] - 5 * integ[-2] + 2 * integ[-3]) / 2
            trd =  5 / 2 * (2 * fourth - 9 * integ[-1] + 18 * integ[-2] - 11 * integ[-3]) / 6
            quad_value = (fst + snd + trd + integ[-3]) * self.dt
            return quad_value
        #力に重力の影響付加
        lift_i = numpy.array(self.lift_i, dtype = "float_")
        drag_i = numpy.array(self.drag_i, dtype = "float_")
        thrust_i = numpy.array(self.thrust_i, dtype = "float_")
        vel_i = numpy.array(self.vel_i, dtype = "float_")
        vel_b = numpy.array(self.vel_b, dtype = "float_")
        r_i = numpy.array(self.r_i, dtype = "float_")

        forces_i = self.mass * numpy.array([[0.0],[0.0],[9.8]],dtype = "float_") + thrust_i + lift_i + drag_i

        self.force_i = numpy.ndarray.tolist(forces_i)

        inatia   = numpy.array(self.inatia)
        gyro     = numpy.array(self.gyro)
        moment_b = numpy.array(self.moment_b)
        quat     = numpy.array(self.quaternion_b2i.quat)
        dt = numpy.array(self.dt)
        K = numpy.array(self.K)

        if self.i == 1:
            vel_i += (forces_i) / self.mass * self.dt
            r_i   += vel_i * self.dt

            self.vel_i = numpy.ndarray.tolist(vel_i)
            self.r_i = numpy.ndarray.tolist(r_i)

            gyro += (moment_b)/ inatia * self.dt
            dq_dt = self.quaternion_b2i.dq_dt(gyro[0],gyro[1],gyro[2])
            quat += numpy.array(dq_dt,dtype = "float_") * self.dt

            self.gyro = numpy.ndarray.tolist(gyro)
            self.quaternion_b2i.quat = numpy.ndarray.tolist(quat)
            self.quaternion_b2i.normalize()
            self.dcm_b2i = self.quaternion_b2i.quat2dcm()
            self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
            self.quaternion_i2b.normalize()
            self.dcm_i2b = self.quaternion_i2b.quat2dcm()

            self.integ_dgyro_dt.append((moment_b - K * gyro) / inatia)
            self.integ_dq_dt.append(dq_dt)
            self.integ_forces_i.append(numpy.ndarray.tolist(forces_i / self.mass))
            self.integ_vel_i.append(self.vel_i)


        elif self.i == 2 or self.i == 3:
            vel_i += ((forces_i) / self.mass + numpy.array(self.integ_forces_i[-1])) / 2 * self.dt
            r_i   += (vel_i + self.vel_i[-1]) / 2 * self.dt

            self.vel_i = numpy.ndarray.tolist(vel_i)
            self.r_i = numpy.ndarray.tolist(r_i)

            gyro += ((moment_b - K * gyro) / inatia + self.integ_dgyro_dt[-1]) / 2 * self.dt
            dq_dt = self.quaternion_b2i.dq_dt(gyro[0],gyro[1],gyro[2])
            quat += (numpy.array(dq_dt,dtype = "float") + numpy.array(self.integ_dq_dt[-1],dtype ="float_")) / 2 * self.dt


            self.integ_dgyro_dt.append((moment_b - K * numpy.array(self.gyro,dtype = "float_")) / inatia)
            self.gyro = numpy.ndarray.tolist(gyro)
            self.quaternion_b2i.quat = numpy.ndarray.tolist(quat)
            self.quaternion_b2i.normalize()
            self.dcm_b2i = self.quaternion_b2i.quat2dcm()
            self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
            self.quaternion_i2b.normalize()
            self.dcm_i2b = self.quaternion_i2b.quat2dcm()


            self.integ_dq_dt.append(dq_dt)
            self.integ_forces_i.append(numpy.ndarray.tolist(forces_i / self.mass))
            self.integ_vel_i.append(self.vel_i)

        else:
            vel_i += quad_quad_kari(self,(forces_i) / self.mass, self.integ_forces_i)
            r_i   += quad_quad_kari(self,vel_i,self.integ_vel_i)

            self.vel_i = numpy.ndarray.tolist(vel_i)
            self.r_i = numpy.ndarray.tolist(r_i)

            gyro += quad_quad_kari(self,(moment_b - K * gyro) / inatia,self.integ_dgyro_dt)
            dq_dt = self.quaternion_b2i.dq_dt(gyro[0],gyro[1],gyro[2])
            quat += quad_quad_kari(self,dq_dt,self.integ_dq_dt)

            self.integ_dgyro_dt.append((moment_b - K * numpy.array(self.gyro,dtype = "float_")) / inatia)
            self.gyro = numpy.ndarray.tolist(gyro)
            self.quaternion_b2i.quat = numpy.ndarray.tolist(quat)
            self.quaternion_b2i.normalize()
            self.dcm_b2i = self.quaternion_b2i.quat2dcm()
            self.quaternion_i2b = Quaternion(self.quaternion_b2i.inverse())
            self.quaternion_i2b.normalize()
            self.dcm_i2b = self.quaternion_i2b.quat2dcm()


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
        vel_i = numpy.array(self.vel_i,dtype = "float_")
        wind = numpy.array(self.wind,dtype = "float_")

        r_i = numpy.array(self.r_i,dtype = "float_")


        vel_b = numpy.dot(numpy.array(self.dcm_i2b),vel_i + wind)
        #vel_b -= self.Ko * (vel_b - numpy.array(self.vel_b,dtype = "float_"))
        self.vel_b = numpy.ndarray.tolist(vel_b)
        self.quaternion_b2i.normalize()
        self.quaternion_i2b.normalize()

    def log(self):
        self.log_r = numpy.hstack((self.log_r,self.r_i))
        self.log_vel_b = numpy.hstack((self.log_vel_b,self.vel_b))
        self.log_vel_v = numpy.hstack((self.log_vel_v,self.vel_v))
        self.log_force_i = numpy.hstack((self.log_force_i,self.force_i))
        self.log_lift_i = numpy.hstack((self.log_lift_i,self.lift_i))
        self.log_euler = numpy.hstack((self.log_euler,self.quaternion_b2i.quat2euler()))
        self.log_t = numpy.hstack((self.log_t,self.t))
        self.log_alpha = numpy.hstack((self.log_alpha,self.alpha))
        self.log_beta = numpy.hstack((self.log_beta,self.beta))
        self.log_quat.append(self.quaternion_b2i.quat)

class Rocket_Graphic():
    def __init__(self):
        self.step = 50


    def matplotlib_3d(self,rocket):
        fig = plt.figure()
        ax = fig.gca(projection='3d')
        ax.set_aspect("box")

        plt.hold(True)


        bodyline_vector = numpy.array([[1.0],[0.0],[0.0]],dtype = "float_")
        quat_bodyline = Quaternion()
        for i_fig in range(0,rocket.i-1,self.step):
            fig_list = [[rocket.log_r[0,i_fig]],[rocket.log_r[1,i_fig]],[-rocket.log_r[2,i_fig]]]
            ax.scatter3D(fig_list[0],fig_list[1],fig_list[2])

            quat_bodyline.__init__(rocket.log_quat[i_fig][:])
            dcm_bodyline = numpy.array(quat_bodyline.quat2dcm())
            bodyline = numpy.dot(dcm_bodyline,bodyline_vector)
            r_vec = numpy.array([[rocket.log_r[0][i_fig]],[rocket.log_r[1][i_fig]],[rocket.log_r[2][i_fig]]])
            bodyline_rear = (- bodyline + r_vec)
            fig_list_bodyline = numpy.hstack([r_vec,bodyline_rear])

            ax.plot3D(fig_list_bodyline[0],fig_list_bodyline[1],-fig_list_bodyline[2])

        plt.show()


def main():

    rocket = Rocket()
    rocket_graphic = Rocket_Graphic()
    while -rocket.r_i[2][0] >= -5.0:
        rocket.calcaoa_velocityaxis()
        rocket.calc_CL()
        rocket.calc_Cd()
        rocket.calcdcm_b2i_i2b()
        rocket.rocket_force_morment()
        rocket.integrate()
        rocket.redifinitions()
        rocket.log()
        rocket.quaternion_b2i.normalize()
        rocket.quaternion_i2b.normalize()
        if 1.0 != round(numpy.sqrt(rocket.quaternion_b2i.quat[0][0] ** 2 + rocket.quaternion_b2i.quat[1][0] ** 2 + rocket.quaternion_b2i.quat[2][0] ** 2 + rocket.quaternion_b2i.quat[3][0] ** 2),4):
            print(rocket.i)

    plt.plot(rocket.log_r[0][:],-rocket.log_r[2][:])

    plt.hold(True)
    plt.plot(rocket.log_r[0][:],rocket.log_r[1][:])
    plt.axis("equal")
    plt.show()
    #plt.plot(rocket.log_t,rocket.log_vel_b[1][:])
    #plt.plot(rocket.log_t,rocket.log_vel_b[0][:],"o")
    #plt.plot(rocket.log_r[0][:],rocket.log_t)
    #plt.plot(rocket.log_t,rocket.log_lift_i[0][:] * 1000)
    plt.plot(rocket.log_t,rocket.log_alpha * 180/ numpy.pi)
    plt.plot(rocket.log_t,rocket.log_beta * 180/ numpy.pi)
    #plt.plot(rocket.log_t,rocket.log_lift_i[2][:] * 10)
    #plt.plot(rocket.log_t,rocket.log_quat[0][:])
    #plt.plot(rocket.log_t,rocket.log_quat[1][:])
    #plt.plot(rocket.log_t,rocket.log_quat[2][:])
    #plt.plot(rocket.log_t,rocket.log_quat[3][:])
    plt.plot(rocket.log_t,rocket.log_euler[0] * 180/ numpy.pi)
    plt.plot(rocket.log_t,rocket.log_euler[1] * 180/ numpy.pi)
    plt.plot(rocket.log_t,rocket.log_euler[2] * 180/ numpy.pi)
    plt.show()

    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(rocket.log_r[0][:],rocket.log_r[1][:],-rocket.log_r[2][:])
    plt.axis("equal")
    plt.show()

    rocket_graphic.matplotlib_3d(rocket)










if __name__ == '__main__':
    main()
