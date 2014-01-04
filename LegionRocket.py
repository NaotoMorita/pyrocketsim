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

import numpy, sys, os, random, copy
from NaQuaternion import Quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.backends.backend_qt4agg
import matplotlib.backends.backend_agg

from PyQt4 import QtGui, QtCore

class Dataplot(matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg):
    def __init__(self, parent=None, width=6, height=3, dpi=50):
        fig = matplotlib.figure.Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        self.axes.tick_params(axis='both', which='major', labelsize=10)
        self.axes.hold(False)

        matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg.__init__(self, fig)
        self.setParent(parent)

        matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg.setSizePolicy(self,QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Expanding)
        matplotlib.backends.backend_qt4agg.FigureCanvasQTAgg.updateGeometry(self)

    def drowplot(self,x,y):
        if not isinstance(x,list) or not isinstance(y,list):
            print("not list")
        else:
            self.axes.plot(x,y)
            self.draw()

class ResultTabWidget(QtGui.QTabWidget):
    def __init__(self, parent = None):
        QtGui.QTabWidget.__init__(self, parent = parent)

        self.track_graph = Dataplot()
        self.track_t_graph = Dataplot()
        self.attitude_t_graph = Dataplot()


        self.addTab(self.track_graph,"軌跡")
        self.addTab(self.track_t_graph,"経路時間履歴")
        self.addTab(self.attitude_t_graph,"姿勢時間履歴")
        self.addTab(self.attitude_t_graph,"速度時間履歴")
        self.addTab(self.attitude_t_graph,"迎角・推力時間履歴")


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
        self.quaternion_b2i.rot2quat(88 * numpy.pi /180, 0, 1, 0)
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
        self.wind = [[0.0],[0.0],[0.0]]

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

        #ランチャー長さ
        self.louncher = 3.0


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
        CLaoa = 0.5 #[/rad]
        if abs(self.alpha) <= 40:
            self.CL[2] = CLaoa * abs(self.alpha)
        else:
            self.CL[2] = (- 0.01 * CLaoa * (abs(self.alpha) - 40) + CLaoa * 40)

        if abs(self.beta) <= 40:
            self.CL[1] = CLaoa * abs(self.beta)
        else:
            self.CL[1] = (- 0.01 * CLaoa * (abs(self.beta) - 40) + CLaoa * 40)
        self.CL[0] = 0.0

    def calc_Cd(self):
        CDaoa = 0.3
        self.Cd[2] =   0.0
        self.Cd[1] =   0.0
        self.Cd[0] =   CDaoa * (abs(self.alpha) + abs(self.beta))

        if self.t >= self.parashoot_t:
            self.Cd[0] = 1.3




    def simulate_force_moment(self):
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


    def simulate_integrate(self,t_start = 0):
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
        dcm_b2i = numpy.array(self.dcm_b2i, dtype = "float_")
        dcm_i2b = numpy.array(self.dcm_i2b, dtype = "float_")
        r_i = numpy.array(self.r_i, dtype = "float_")

        forces_i = self.mass * numpy.array([[0.0],[0.0],[9.8]],dtype = "float_") + thrust_i + lift_i + drag_i
        if numpy.sqrt(r_i[0] ** 2 + r_i[1] ** 2 + r_i[2] ** 2) <= self.louncher :
            print(self.i)
            force_b = numpy.dot(dcm_i2b,forces_i)
            force_b[1] = 0.0
            force_b[2] = 0.0
            forces_i = numpy.dot(dcm_b2i,force_b)

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


    def sensing_integrate(self):
        def read_csv(self):
            csv_name = "CanSatApage2.csv"
            csvall = numpy.loadtxt(csv_name,delimiter = ",", skiprows = 1)
            acc_scale = 2048.0
            acc_mean = numpy.array([32768,32768,32768])
            gyro_scale = 16.4
            gyro_mean = numpy.array([32755,32783,32769])

            self.senser_t = csvall[:,1]
            self.csv_acc = csvall[:,2:5] - acc_mean
            self.csv_gyro = csvall[:,5:8] -gyro_mean
            self.csv_acc /= (acc_scale)
            self.csv_acc *= 9.8
            self.csv_gyro /= (gyro_scale)

            self.acc_x = self.csv_acc[:,0]
            self.acc_y = self.csv_acc[:,1]
            self.acc_z = self.csv_acc[:,2]

            self.gyro_x = self.csv_gyro[:,0]
            self.gyro_y = self.csv_gyro[:,1]
            self.gyro_z = self.csv_gyro[:,2]




        read_csv(self)
        #初期クオータニオンを定義
        self.quaternion_b2i.rot2quat(100 * numpy.pi /180, 0, 1, 0)

        #初期dcmを計算
        self.quaternion_b2i.normalize()
        self.dcm_b2i = self.quaternion_b2i.quat2dcm()
        self.quaternion_i2b.normalize()
        self.dcm_i2b = self.quaternion_i2b.quat2dcm()
        print(numpy.shape(self.senser_t)[0])

        #積分を実行
        #台形積分補助
        acc_b = numpy.array([[self.acc_x[0]],[self.acc_y[0]],[self.acc_z[0]]])
        sub_acc_i = numpy.dot(self.dcm_b2i,acc_b)
        sub_acc_i += numpy.array([[-9.8],[0],[0]])
        gyro_b = numpy.array([[self.gyro_x[0]],[self.gyro_y[0]],[self.gyro_z[0]]])
        sub_dq_dt = numpy.array(self.quaternion_b2i.dq_dt(gyro_b[0,0],gyro_b[1,0],gyro_b[2,0]))



        vel_i = numpy.zeros([3,1])
        r_i = numpy.zeros([3,1])
        for i_senser in range(1,numpy.shape(self.senser_t)[0]):
            #加速度を慣性座標系へ
            acc_b = numpy.array([[self.acc_x[i_senser]],[self.acc_y[i_senser]],[self.acc_z[i_senser]]])
            acc_i = numpy.dot(self.dcm_b2i,acc_b)
            acc_i += numpy.array([[-9.8],[0],[0]])

            #速度へ
            vel_i += (acc_i + sub_acc_i) / 2 * (self.senser_t[i_senser] - self.senser_t[i_senser - 1])

            sub_acc_i = acc_i

            #座標へ
            if i_senser == 1:
                r_i += vel_i * (self.senser_t[i_senser] - self.senser_t[i_senser - 1])
            else:
                r_i += (vel_i + sub_vel_i) / 2 * (self.senser_t[i_senser] - self.senser_t[i_senser - 1])
            sub_vel_i = vel_i
            print(r_i)

            #quaternion更新
            gyro_b = numpy.array([[self.gyro_x[i_senser]],[self.gyro_y[i_senser]],[self.gyro_z[i_senser]]])
            dq_dt = numpy.array(self.quaternion_b2i.dq_dt(gyro_b[0,0],gyro_b[1,0],gyro_b[2,0]))
            new_quat = numpy.array(self.quaternion_b2i.quat) +(dq_dt + sub_dq_dt) / 2 * (self.senser_t[i_senser] - self.senser_t[i_senser - 1])
            self.quaternion_b2i.quat = numpy.ndarray.tolist(new_quat)
            self.quaternion_b2i.normalize()
            self.dcm_b2i = self.quaternion_b2i.quat2dcm()

            self.quaternion_i2b.quat = self.quaternion_b2i.inverse()
            self.quaternion_i2b.normalize()
            self.dcm_i2b = self.quaternion_i2b.quat2dcm()

            self.log_r = numpy.hstack((self.log_r,r_i))
            self.log_quat.append(self.quaternion_b2i.quat)
            self.i += 1


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
        self.step = 500


    def matplotlib_3d(self,rocket):
        def axis_equal(X,Y,Z):
            max_range = numpy.array([numpy.array(X).max()-numpy.array(X).min(), numpy.array(Y).max()-numpy.array(Y).min(), numpy.array(Z).max()-numpy.array(Z).min()]).max()
            print(max_range)
            Xb = 0.5*max_range*numpy.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(numpy.array(X).max()+numpy.array(X).min())
            Yb = 0.5*max_range*numpy.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(numpy.array(Y).max()+numpy.array(Y).min())
            Zb = 0.5*max_range*numpy.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(numpy.array(Z).max()+numpy.array(Z).min())
            # Comment or uncomment following both lines to test the fake bounding box:
            return [Xb,Yb,Zb]


        fig = plt.figure()
        ax = fig.gca(projection='3d')
        plt.hold(True)




        bodyline_vector = numpy.array([[1000.0],[0.0],[0.0]],dtype = "float_")
        quat_bodyline = Quaternion()
        for i_fig in range(0,rocket.i-1,self.step):
            fig_list = [[rocket.log_r[0,i_fig]],[rocket.log_r[1,i_fig]],[-rocket.log_r[2,i_fig]]]
            print(fig_list)
            ax.scatter3D(fig_list[0],fig_list[1],fig_list[2])
            ax.set_aspect('equal')
            quat_bodyline.__init__(rocket.log_quat[i_fig][:])
            dcm_bodyline = numpy.array(quat_bodyline.quat2dcm())
            bodyline = numpy.dot(dcm_bodyline,bodyline_vector)
            print(numpy.sqrt(bodyline[0] ** 2 + bodyline[1] ** 2 + bodyline[2] ** 2))
            r_vec = numpy.array([[rocket.log_r[0][i_fig]],[rocket.log_r[1][i_fig]],[rocket.log_r[2][i_fig]]])
            bodyline_rear = (- bodyline + r_vec)
            fig_list_bodyline = numpy.hstack([r_vec,bodyline_rear])

            ax.plot3D(fig_list_bodyline[0],fig_list_bodyline[1],-fig_list_bodyline[2])

        XYZb = axis_equal(rocket.log_r[0][:],rocket.log_r[1][:],rocket.log_r[2][:])
        print(XYZb)
        for xb, yb, zb in zip(XYZb[0], XYZb[1], XYZb[2]):
            ax.plot([xb], [yb], [zb],"w")

        plt.show()


def main():

    rocket = Rocket()
    rocket_graphic = Rocket_Graphic()
    rocket.sensing_integrate()
##    while -rocket.r_i[2][0] >= -5.0:
##        rocket.calcaoa_velocityaxis()
##        rocket.calc_CL()
##        rocket.calc_Cd()
##        rocket.calcdcm_b2i_i2b()
##        rocket.simulate_force_moment()
##        rocket.simulate_integrate()
##        rocket.redifinitions()
##        rocket.log()
##        rocket.quaternion_b2i.normalize()
##        rocket.quaternion_i2b.normalize()
##        if 1.0 != round(numpy.sqrt(rocket.quaternion_b2i.quat[0][0] ** 2 + rocket.quaternion_b2i.quat[1][0] ** 2 + rocket.quaternion_b2i.quat[2][0] ** 2 + rocket.quaternion_b2i.quat[3][0] ** 2),4):
##            print(rocket.i)

    print(rocket.log_r[0,:])
    matplotlib.pyplot.plot(rocket.log_r[0,:],rocket.log_r[2,:])
    matplotlib.pyplot.show()
    rocket_graphic.matplotlib_3d(rocket)










if __name__ == '__main__':
    main()
