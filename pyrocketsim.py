import numpy,scipy
import matplotlib.figure, matplotlib.pyplot
import math

class Rocket():
    def __init__(self):
        self.phi = 0
        self.theta = 0
        self.psi = 0

class Quaternion():
    def __init__(self, q0 = 1, q1 = 0, q2 = 0, q3 = 0):
        if "self.q0" in locals():
            pass
        else:
            self.q0 = float(q0)
            self.q1 = float(q1)
            self.q2 = float(q2)
            self.q3 = float(q3)

    def __mul__(self, other):
        q_buff = [[self.q0,-self.q1,-self.q2,-self.q3],
        [self.q1,self.q0,-self.q3,self.q2],
        [self.q2,self.q3,self.q0,-self.q1],
        [self.q3,-self.q2,self.q1,self.q0]]
        q_mat = scipy.mat(q_buff)

        p_buff =[[other.q0],
        [other.q1],
        [other.q2],
        [other.q3]]

        p_mat = scipy.mat(p_buff)
        product_buff = q_mat * p_mat

        buff = product_buff.tolist()
        quat0 = float(product_buff[0])
        quat1 = float(product_buff[1])
        quat2 = float(product_buff[2])
        quat3 = float(product_buff[3])

        return Quaternion(quat0, quat1, quat2, quat3)


    def normalize(self):
        norm = numpy.sqrt(self.q0**2+self.q1**2+self.q2**2+self.q3**2)
        self.q0 = self.q0 / norm
        self.q1 = self.q1 / norm
        self.q2 = self.q2 / norm
        self.q3 = self.q3 / norm

    def inverse(self):
        norm = numpy.linalg.norm([self.q0,self.q1,self.q2,self.q3])
        qd0 = self.q0 / norm
        qd1 = -self.q1 / norm
        qd2 = -self.q2 / norm
        qd3 = -self.q3 / norm

        return Quaternion(qd0,qd1,qd2,qd3)

    def rot2quat(self,theta,x,y,z):
        self.q0 = math.cos(theta / 2)
        self.q1 = math.sin(theta / 2) * x
        self.q2 = math.sin(theta / 2) * y
        self.q3 = math.sin(theta / 2) * z

    def dq_dt(self,ox, oy, oz):
        buff = Quaternion()
        buff = self * Quaternion(0, ox, oy, oz)
        buff.q0 /= 2.0
        buff.q1 /= 2.0
        buff.q2 /= 2.0
        buff.q3 /= 2.0
        return buff

    def quat2euler(self):
        self.phi = math.atan((2 * (self.q0 * self.q1 + self.q2 * self.q3 )) / (1 - 2 * (self.q1 ** 2 + self.q2 ** 2)))
        self.theta = math.asin(2 * (self.q0 * self.q2 - self.q3 * self.q1))
        self.psi = math.atan((2 * (self.q0 * self.q3 + self.q1 * self.q2 )) / (1 - 2 * (self.q2 ** 2 + self.q3 ** 2)))

    def quat2dcm(self):
        self.dcm = numpy.array([[0.0,1.0,0.0],[0.0,1.0,0.0],[0.0,0.0,0.0]])
        self.dcm[0,0] = float(self.q0 ** 2 + self.q1 ** 2 - self.q2 ** 2 - self.q3 ** 2)
        self.dcm[0,1] = 2 * (self.q1 * self.q2 - self.q0 * self.q3)
        self.dcm[0,2] = 2 * (self.q0 * self.q2 + self.q1 * self.q3)
        self.dcm[1,0] = 2 * (self.q1 * self.q2 + self.q0 * self.q3)
        self.dcm[1,1] = self.q0 ** 2 - self.q1 ** 2 + self.q2 ** 2 - self.q3 ** 2
        self.dcm[1,2] = 2 * (self.q2 * self.q3 - self.q0 * self.q1)
        self.dcm[2,0] = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
        self.dcm[2,1] = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
        self.dcm[2,2] = self.q0 ** 2 - self.q1 ** 2 - self.q2 ** 2 + self.q3 ** 2


    def euler2quat(self,phi,theta,psi):
        self.q0 = math.cos(theta / 2) * math.cos(theta / 2) * math.cos(psi / 2) + math.sin(theta / 2) * math.sin(theta / 2) * math.sin(psi / 2)
        self.q1 = math.sin(theta / 2) * math.cos(theta / 2) * math.cos(psi / 2) - math.cos(theta / 2) * math.sin(theta / 2) * math.sin(psi / 2)
        self.q2 = math.cos(theta / 2) * math.sin(theta / 2) * math.cos(psi / 2) + math.sin(theta / 2) * math.cos(theta / 2) * math.sin(psi / 2)
        self.q3 = math.cos(theta / 2) * math.cos(theta / 2) * math.sin(psi / 2) - math.sin(theta / 2) * math.sin(theta / 2) * math.cos(psi / 2)



class Rocket():
    def __init__(self):
        #-----蛻晄悄蟋ｿ蜍｢繧定ｨ倩ｿｰ
        if "self.vel_b" in locals():
            print('a')
            pass

        else:
            print("b")
            self.attitude = Quaternion()
            self.attitude.rot2quat(-90*math.pi/180,0,1,0)
            print([self.attitude.q0,self.attitude.q1,self.attitude.q2,self.attitude.q3])
            self.attitude.quat2euler()
            print(self.attitude.theta)

            self.vel_b = numpy.array([[0],[0],[0]], dtype = 'f')
            self.angvel_b = numpy.array([[0.0],[0.0],[0.0]], dtype = 'f')

            self.attitude.quat2dcm()
            self.attitude.dcm_b2i = self.attitude.dcm
            inv_atti = self.attitude.inverse()
            inv_atti.quat2dcm()
            self.attitude.dcm_i2b = inv_atti.dcm
            self.vel_i = numpy.dot(self.attitude.dcm_b2i , self.vel_b)
            self.angvel_i = numpy.dot(self.attitude.dcm_b2i , self.angvel_b)
            self.r_i = numpy.array([[0],[0],[-10]],dtype = 'f')

            self.dt = 0.005
            self.t = 0

            self.wind = numpy.array([[0],[0],[0]], dtype = 'f')

            self.m = 0.0714
            self.Ix = 0.0000198
            self.Iy = 0.001293
            self.Iz = 0.001293

            self.launcher_length = 2
            self.launcher_height = numpy.dot(self.attitude.dcm_b2i,numpy.array([[self.launcher_length],[0],[0]],dtype = 'f'))[2,0]

            self.i = 0

            self.log_t = self.t
            self.log_r_i = self.r_i
            self.log_vel_i = self.vel_i
            self.attitude.quat2euler()
            self.log_phi = self.attitude.phi
            self.log_theta = self.attitude.phi
            self.log_psi = self.attitude.phi



            #-----閭ｴ菴捺恕蜉帛だ譁・
            self.CLab = float(2)
            self.lp = 0.06
            self.sf = (0.023) ** 2 / 4 * math.pi
            self.ss = (2.5*42.5) * 10 ** (-4)
            self.CD = 0.8
            self.CDab = 0.008
            self.parashoot_time = 4
            self.sp = (0.25) ** 2 * math.pi / 4

    def force_moment(self):

        print(numpy.arcsin(self.vel_b[2,0] /numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2)))

        if self.vel_b[0,0] != 0:
            self.attitude.quat2euler()
            print(self.attitude.theta*180/math.pi)
            print(self.vel_b)
            self.alpha = numpy.arcsin(self.vel_b[2,0] /numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2))
            self.beta = numpy.arctan(self.vel_b[1,0] / self.vel_b[0,0]) * self.vel_b[0,0] / abs(self.vel_b[0,0])
            print(self.alpha*180/math.pi)
        else:
            self.alpha = float(0)
            self.beta = float(0)

        if numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2) != 0:
            xd = self.vel_b / numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2)
            yd = numpy.cross((xd+numpy.array([[0],[0],[10]])).T,xd.T).T
            yd = yd / numpy.sqrt(yd[0,0] ** 2 + yd[1,0] ** 2 + yd[2,0] ** 2)
            zd = numpy.cross(xd.T,yd.T).T
            self.dcm_v2b = numpy.hstack([xd,yd,zd])
            self.dcm_b2v = numpy.linalg.inv(self.dcm_v2b)

        else:
            self.dcm_b2v = numpy.eye(3,3)
            self.dcm_v2b = numpy.eye(3,3)

        self.vel_v = numpy.dot(self.dcm_b2v,self.vel_b)

        #-----以下揚力計算
        rho = 1.184
        if self.vel_v[0,0] == 0:
            self.lift = numpy.array([[0],[0],[0]],dtype = 'f')

        else:
            self.lift = numpy.array([[0],[0],[0]],dtype = 'f')
            self.lift[1,0] = 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.ss * self.CLab * self.beta
            self.lift[2,0] = 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.ss * self.CLab * self.alpha

        #-----以下抗力計算
        if self.vel_v[0,0] == 0:
            self.drag = numpy.array([[0],[0],[0]],dtype = 'f')

        elif self.parashoot_time <= self.t:
            self.drag = numpy.array([1 / 2 * rho * self.vel_v[0,0]^2 * 1.3 * self.sp],[0],[0],dtype = 'f')

        else:
            self.drag = numpy.array([[0],[0],[0]],dtype = 'f')
            self.drag[0,0] = 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.sf * self.CD
            if -self.r_i[2,0] >= -self.launcher_height:
                self.drag[0,0] += 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.ss * self.CDab * (abs(self.alpha)+abs(self.beta))

        self.F_v = -self.drag + self.lift
        self.F_b = numpy.dot(self.dcm_v2b,self.F_v)
        self.thrust = float(0)
        self.F_b[0,0] += self.thrust
        self.M_b = numpy.array([[0],[self.F_b[2,0] * self.lp],[-self.F_b[1,0] * self.lp]])

    def integral_at_i(self):

                    #dcm更新
        self.attitude.quat2dcm()
        self.attitude.dcm_b2i = self.attitude.dcm
        inv_atti = self.attitude.inverse()
        inv_atti.quat2dcm()
        self.attitude.dcm_i2b = inv_atti.dcm

        self.F_i = numpy.dot(self.attitude.dcm_b2i,self.F_b)
        self.M_i = numpy.dot(self.attitude.dcm_b2i,self.M_b)

        self.F_i[2,0] += 9.8 * self.m

        #-----ランチャーから離れるまえ
        if -self.r_i[2,0] <= -self.launcher_height:
            self.F_e = numpy.dot(self.attitude.dcm_i2b,self.F_i)
            self.F_e[1,0] = 0
            self.F_e[2,0] = 0
            self.F_i = numpy.dot(self.attitude.dcm_b2i,self.F_e)
            if self.F_i[2,0] >= 0:
                self.F_i[2,0] = 0

        self.acc_i = self.F_i / self.m
        self.gyro_i = self.M_i / numpy.array([[self.Ix],[self.Iy],[self.Iz]])
        #-----慣性座標系での積分
        if self.i == 0:
            self.vel_i += self.acc_i * self.dt
            self.r_i += self.vel_i * self.dt
            self.angvel_i += self.gyro_i * self.dt
            self.sub_vel_i = self.vel_i
            self.sub_acc_i = self.acc_i
            self.sub_gyro_i = self.gyro_i
            self.sub_angvel_i = self.angvel_i

            if -self.r_i[2,0] >= -self.launcher_height:
                dq_dt = Quaternion()
                dq_dt.dq_dt(self.angvel_i[0,0],self.angvel_i[1,0],self.angvel_i[2,0])
                self.attitude.q0 += dq_dt.q0 * self.dt
                self.attitude.q1 += dq_dt.q1 * self.dt
                self.attitude.q2 += dq_dt.q2 * self.dt
                self.attitude.q3 += dq_dt.q3 * self.dt
                self.attitude.normalize()



        else:
            self.vel_i += (self.acc_i + self.sub_acc_i) / 2 * self.dt
            self.r_i += (self.vel_i + self.sub_vel_i) / 2 * self.dt
            self.angvel_i += (self.gyro_i + self.sub_gyro_i) / 2 * self.dt
            self.sub_vel_i = self.vel_i
            self.sub_acc_i = self.acc_i
            self.sub_gyro_i = self.gyro_i
            self.sub_angvel_i = self.angvel_i

            if -self.r_i[2,0] >= -self.launcher_height:
                dq_dt = Quaternion()
                dq_dt.dq_dt((self.angvel_i[0,0]+self.sub_angvel_i[0,0])/2,(self.angvel_i[1,0]+self.sub_angvel_i[1,0])/2,(self.angvel_i[2,0]+self.sub_angvel_i[2,0])/2)
                self.attitude.q0 += dq_dt.q0 * self.dt
                self.attitude.q1 += dq_dt.q1 * self.dt
                self.attitude.q2 += dq_dt.q2 * self.dt
                self.attitude.q3 += dq_dt.q3 * self.dt
                self.attitude.normalize()
        print(self.vel_i)
        print(self.wind)
        #------機体座標系へ変換
        self.angvel_b = numpy.dot(self.attitude.dcm_i2b,self.angvel_i)
        self.vel_b = numpy.dot(self.attitude.dcm_i2b,(self.vel_i+self.wind))
        print(self.attitude.dcm_i2b)
        print(self.vel_b)
        print(self.F_i)
        print(self.F_b)


    def logger(self):
        self.log_t = numpy.hstack([self.log_t,self.t])
        self.log_r_i = numpy.hstack([self.log_r_i,self.r_i])
        self.log_vel_i = numpy.hstack([self.log_vel_i,self.vel_i])
        self.attitude.quat2euler()
        self.log_phi = numpy.hstack([self.log_phi,self.attitude.phi])
        self.log_theta = numpy.hstack([self.log_theta,self.attitude.theta])
        self.log_psi = numpy.hstack([self.log_psi,self.attitude.psi])




    def exesimulate(self):
        while 1:
            self.force_moment()
            self.integral_at_i()
            self.logger()
            self.i += 1
            self.t = self.dt * self.i
            if -self.r_i[2,0] <= 0 and self.vel_i[2,0] >= 0:
                self.i -= 1
                break




class LegionControler():
    pass







def main():

    rocket = Rocket()
    rocket.exesimulate()
    matplotlib.pyplot.plot(rocket.log_r_i[0,:],-rocket.log_r_i[2,:])
    matplotlib.pyplot.axis("equal")
    #matplotlib.pyplot.show()
    print(rocket.log_theta*180/math.pi)





if __name__ == '__main__':
    main()
