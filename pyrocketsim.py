import numpy,scipy
import matplotlib.figure, matplotlib.pyplot
import math, copy
from decimal import Decimal

class Quaternion():
    def __init__(self, q0 = 1, q1 = 0, q2 = 0, q3 = 0):
        if "self.q0" in locals():
            pass
        else:
            self.q0 = Decimal(q0)
            self.q1 = Decimal(q1)
            self.q2 = Decimal(q2)
            self.q3 = Decimal(q3)

    def __repr__(self):
        return "%f,%f, %f,%f" % (self.q0, self.q1, self.q2, self.q3)

    def quat_product(self, other):
        q_buff = [[self.q0,-self.q1,-self.q2,-self.q3],
        [self.q1,self.q0,-self.q3,self.q2],
        [self.q2,self.q3,self.q0,-self.q1],
        [self.q3,-self.q2,self.q1,self.q0]]
        q_mat = numpy.array(q_buff)

        p_buff =[[other.q0],
        [other.q1],
        [other.q2],
        [other.q3]]

        p_mat = numpy.array(p_buff)
        product_buff = numpy.dot(q_mat,p_mat)

        buff = product_buff.tolist()
        quat0 = Decimal(product_buff[0])
        quat1 = Decimal(product_buff[1])
        quat2 = Decimal(product_buff[2])
        quat3 = Decimal(product_buff[3])

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
        dq_dt = Quaternion()
        omega = Quaternion(q0 = 0.0, q1 = ox, q2 = oy, q3 = oz)
        dq_dt = self.quat_product(omega)

        dq_dt.q0 /= 2.0
        dq_dt.q1 /= 2.0
        dq_dt.q2 /= 2.0
        dq_dt.q3 /= 2.0
        return dq_dt

    def quat2euler(self):
        phi = math.atan((2 * (self.q0 * self.q1 + self.q2 * self.q3 )) / (1 - 2 * (self.q1 ** 2 + self.q2 ** 2)))
        theta = math.asin(2 * (self.q0 * self.q2 - self.q3 * self.q1))
        psi = math.atan((2 * (self.q0 * self.q3 + self.q1 * self.q2 )) / (1 - 2 * (self.q2 ** 2 + self.q3 ** 2)))
        return [phi,theta,psi]

    def quat2dcm(self):
        dcm = numpy.array([[0.0,1.0,0.0],[0.0,1.0,0.0],[0.0,0.0,0.0]],dtype = numpy.dtype(Decimal))
        dcm[0,0] = (self.q0 ** 2 + self.q1 ** 2 - self.q2 ** 2 - self.q3 ** 2)
        dcm[0,1] = 2 * (self.q1 * self.q2 - self.q0 * self.q3)
        dcm[0,2] = 2 * (self.q0 * self.q2 - self.q1 * self.q3)
        dcm[1,0] = 2 * (self.q1 * self.q2 + self.q0 * self.q3)
        dcm[1,1] = (self.q0 ** 2 - self.q1 ** 2 + self.q2 ** 2 - self.q3 ** 2)
        dcm[1,2] = 2 * (self.q2 * self.q3 - self.q0 * self.q1)
        dcm[2,0] = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
        dcm[2,1] = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
        dcm[2,2] = (self.q0 ** 2 - self.q1 ** 2 - self.q2 ** 2 + self.q3 ** 2)
        return dcm

    def euler2quat(self,phi,theta,psi):
        self.q0 = math.cos(theta / 2) * math.cos(theta / 2) * math.cos(psi / 2) + math.sin(theta / 2) * math.sin(theta / 2) * math.sin(psi / 2)
        self.q1 = math.sin(theta / 2) * math.cos(theta / 2) * math.cos(psi / 2) - math.cos(theta / 2) * math.sin(theta / 2) * math.sin(psi / 2)
        self.q2 = math.cos(theta / 2) * math.sin(theta / 2) * math.cos(psi / 2) + math.sin(theta / 2) * math.cos(theta / 2) * math.sin(psi / 2)
        self.q3 = math.cos(theta / 2) * math.cos(theta / 2) * math.sin(psi / 2) - math.sin(theta / 2) * math.sin(theta / 2) * math.cos(psi / 2)




































class Rocket():
    def __init__(self):
        #-----
        if "self.vel_b" in locals():
            pass

        else:

            self.quat = Quaternion()
            self.quat.rot2quat(60*math.pi/180,0,1,0)
            self.euler = self.quat.quat2euler()


            self.vel_b = numpy.array([[20],[0],[0]], dtype=numpy.dtype(Decimal))
            self.angvel_b = numpy.array([[0.0],[0.0],[0.0]], dtype=numpy.dtype(Decimal))

            self.dcm_b2i = self.quat.quat2dcm()
            self.dcm_i2b = (self.quat.inverse()).quat2dcm()

            self.vel_i = numpy.dot(self.dcm_b2i , self.vel_b)
            self.angvel_i = numpy.dot(self.dcm_b2i , self.angvel_b)
            self.r_i = numpy.array([[0],[0],[0]],dtype=numpy.dtype(Decimal))

            self.dt = Decimal(0.05)
            self.t = Decimal(0)

            self.wind = numpy.array([[0],[0],[0]], dtype=numpy.dtype(Decimal))

            self.m = Decimal(0.0714)
            self.Ix = 0.0000198
            self.Iy = 0.001293
            self.Iz = 0.001293

            self.launcher_length = 2.0
            self.launcher_height = numpy.dot(self.dcm_b2i,numpy.array([[self.launcher_length],[0],[0]],dtype=numpy.dtype(Decimal)))[2,0]

            self.i = 0

            self.log_t = copy.deepcopy(self.t)
            self.log_r_i = copy.deepcopy(self.r_i)
            self.log_vel_i = copy.deepcopy(self.vel_i)
            self.log_vel_b = copy.deepcopy(self.vel_b)
            self.quat.quat2euler()
            self.log_euler = self.quat.quat2euler()

            self.log_alpha = 0.0
            self.log_angvel_b = copy.deepcopy(self.angvel_b)
            self.log_M_i = numpy.array([[0],[0],[0]],dtype=numpy.dtype(Decimal))



            #-----鬮｢・ｭ繝ｻ・ｴ髣厄ｽｴ隰撰ｽｺ隲ｱ謌頑剰涕蟶吮味髫ｴ竏壹・
            self.CLab = Decimal(2)
            self.lp = 0.023
            self.sf = (0.023) ** 2 / 4 * math.pi
            self.ss = (2.5*42.5) * 10 ** (-4)
            self.CD = 0.8
            self.CDab = 0.008
            self.parashoot_time = 400
            self.sp = (0.25) ** 2 * math.pi / 4

    def __repr__(self):
        return "%f, %f, %f\n%f, %f, %f\n%f, %f, %f\n" % (self.dcm[0,0], self.dcm[0,1], self.dcm[0,2], self.dcm[1,0], self.dcm[1,1], self.dcm[1,2], self.dcm[2,0], self.dcm[2,1], self.dcm[2,2])






























    def force_moment(self):

        if self.vel_b[0,0] != 0:
            self.alpha = -numpy.arcsin(self.vel_b[2,0] / numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2))
            self.beta = -numpy.arctan(self.vel_b[1,0] / self.vel_b[0,0]) * self.vel_b[0,0] / abs(self.vel_b[0,0])
        else:
            self.alpha = Decimal(0)
            self.beta = Decimal(0)

        if numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2) != 0:
            xd = self.vel_b / numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2)
            yd = numpy.cross((xd+numpy.array([[0],[0],[10]],dtype=numpy.dtype(Decimal))).T,xd.T).T
            yd = yd / numpy.sqrt(yd[0,0] ** 2 + yd[1,0] ** 2 + yd[2,0] ** 2)
            zd = numpy.cross(xd.T,yd.T).T
            self.dcm_v2b = numpy.hstack([xd,yd,zd])
            self.dcm_b2v = numpy.linalg.inv(self.dcm_v2b)

        else:
            self.dcm_b2v = numpy.eye(3,3)
            self.dcm_v2b = numpy.eye(3,3)

        self.vel_v = numpy.dot(self.dcm_b2v,self.vel_b)





        #-----闔会ｽ･闕ｳ蛹ｺ諱戊怏蟷・ｽｨ閧ｲ・ｮ繝ｻ
        rho = 1.184
        if self.vel_v[0,0] == 0:
            self.lift = numpy.array([[0],[0],[0]],dtype=numpy.dtype(Decimal))

        else:
            self.lift = numpy.array([[0],[0],[0]],dtype=numpy.dtype(Decimal))
            self.lift[1,0] = 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.ss * self.CLab * self.beta
            self.lift[2,0] = 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.ss * self.CLab * self.alpha

        #-----闔会ｽ･闕ｳ蛹ｺ闥ｲ陷牙ｹ・ｽｨ閧ｲ・ｮ繝ｻ
        if self.vel_v[0,0] == 0:
            self.drag = numpy.array([[0],[0],[0]],dtype=numpy.dtype(Decimal))

        elif self.parashoot_time <= self.t:
            self.drag = numpy.array([1 / 2 * rho * self.vel_v[0,0]^2 * 1.3 * self.sp],[0],[0],dtype=numpy.dtype(Decimal))

        else:
            self.drag = numpy.array([[0],[0],[0]],dtype=numpy.dtype(Decimal))
            self.drag[0,0] = 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.sf * self.CD
            if -self.r_i[2,0] >= -self.launcher_height:
                self.drag[0,0] += 1 / 2 * rho * self.vel_v[0,0] ** 2 * self.ss * self.CDab * (abs(self.alpha)+abs(self.beta))

        self.F_v = numpy.array([[0],[0],[0]],dtype='f')
        self.F_v[0,0] = -self.drag[0,0] + self.lift[0,0]
        self.F_v[1,0] = -self.lift[1,0]
        self.F_v[2,0] = -self.lift[2,0]

        self.F_b = numpy.dot(self.dcm_v2b,self.F_v)
        self.thrust = Decimal(0)
        self.F_b[0,0] += self.thrust
        self.M_b = numpy.array([[0],[self.F_b[2,0] * self.lp * (-1)],[self.F_b[1,0] * self.lp * (1)]])


    def integral_at_i(self):

                    #dcm隴厄ｽｴ隴・ｽｰ
        self.dcm_b2i = self.quat.quat2dcm()
        self.dcm_i2b =(self.quat.inverse()).quat2dcm()


        self.F_i = numpy.dot(self.dcm_b2i,self.F_b)
        self.M_i = numpy.dot(self.dcm_b2i,self.M_b)
        self.F_i[2,0] += 9.8 * self.m

        #-----郢晢ｽｩ郢晢ｽｳ郢昶・ﾎ慕ｹ晢ｽｼ邵ｺ荵晢ｽ蛾ｫｮ・｢郢ｧ蠕鯉ｽ狗ｸｺ・ｾ邵ｺ繝ｻ
        if -self.r_i[2,0] <= -self.launcher_height:
            self.F_b = numpy.dot(self.dcm_i2b,self.F_i)
            self.F_b[1,0] = 0.0
            self.F_b[2,0] = 0.0
            self.F_i = numpy.dot(self.dcm_b2i,self.F_b)
            if self.F_i[2,0] >= 0:
                self.F_i[2,0] = 0

        self.acc_i = self.F_i / self.m

        self.gyro_i = self.M_i
        self.gyro_i[0,0] /= self.Ix
        self.gyro_i[1,0] /= self.Iy
        self.gyro_i[2,0] /= self.Iz

        #-----隲ｷ・｣隲､・ｧ陟趣ｽｧ隶灘衷・ｳ・ｻ邵ｺ・ｧ邵ｺ・ｮ驕ｨ讎翫・
        if self.i == 0:
            self.vel_i += self.acc_i * self.dt
            self.r_i += self.vel_i * self.dt
            self.angvel_i += self.gyro_i * self.dt
            self.sub_vel_i = copy.deepcopy(self.vel_i)
            self.sub_acc_i = copy.deepcopy(self.acc_i)







        else:
            self.vel_i += (self.acc_i + self.sub_acc_i) / 2 * self.dt
            self.r_i += (self.vel_i + self.sub_vel_i) / 2 * self.dt
            self.angvel_i += self.gyro_i * self.dt
            self.sub_vel_i = copy.deepcopy(self.vel_i)
            self.sub_acc_i = copy.deepcopy(self.acc_i)

        #if -self.r_i[2,0] >= -self.launcher_height:
        #------隶匁ｻ会ｽｽ轣假ｽｺ・ｧ隶灘衷・ｳ・ｻ邵ｺ・ｸ陞溽判驪､
        self.angvel_b = numpy.dot(self.dcm_i2b,self.angvel_i)
        self.vel_b = numpy.dot(self.dcm_i2b,(self.vel_i+self.wind))
        if self.i == 0:
            self.sub_angvel_b = copy.deepcopy(self.angvel_b)
        dq_dt = self.quat.dq_dt((self.angvel_b[0,0]+self.sub_angvel_b[0,0])/2,(self.angvel_b[1,0]+self.sub_angvel_b[1,0])/2,(self.angvel_b[2,0]+self.sub_angvel_b[2,0])/2)
        self.sub_angvel_b = copy.deepcopy(self.angvel_b)
        self.quat.q0 += dq_dt.q0 * self.dt
        self.quat.q1 += dq_dt.q1 * self.dt
        self.quat.q2 += dq_dt.q2 * self.dt
        self.quat.q3 += dq_dt.q3 * self.dt
        self.quat.normalize()





    def logger(self):
        self.log_t = numpy.hstack([self.log_t,self.t])
        self.log_r_i = numpy.hstack([self.log_r_i,self.r_i])
        self.log_vel_b = numpy.hstack([self.log_vel_b,self.vel_b])
        self.log_vel_i = numpy.hstack([self.log_vel_i,self.vel_i])
        self.log_euler = numpy.vstack([self.log_euler,self.euler])
        self.log_alpha = numpy.hstack([self.log_alpha,self.alpha])
        self.log_angvel_b = numpy.hstack([self.log_angvel_b,self.angvel_b])
        self.log_M_i = numpy.hstack([self.log_M_i,self.M_i])




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
    matplotlib.pyplot.show()






if __name__ == '__main__':
    main()
