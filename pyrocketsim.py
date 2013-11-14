import numpy,scipy
import matplotlib.figure
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
        q_mat=scipy.mat(q_buff)

        p_buff =[[other.q0],
        [other.q1],
        [other.q2],
        [other.q3]]

        p_mat = scipy.mat(p_buff)
        product_buff =q_mat * p_mat

        buff = product_buff.tolist()
        quat0 = float(product_buff[0])
        quat1 = float(product_buff[1])
        quat2 = float(product_buff[2])
        quat3 = float(product_buff[3])

        return Quaternion(quat0, quat1, quat2, quat3)


    def normalize(self):
        norm = numpy.linalg.norm([self.q0,self.q1,self.q2,self.q3])
        self.n_q0 = self.q0 / norm
        self.n_q1 = self.q1 / norm
        self.n_q2 = self.q2 / norm
        self.n_q3 = self.q3 / norm

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
        buff.q0  = buff.q0 / 2
        buff.q1 / 2
        buff.q2 / 2
        buff.q3 / 2
        return buff

    def quat2euler(self):
        self.phi = math.atan((2 * (self.q0 * self.q1 + self.q2 * self.q3 )) / (1 - 2 * (self.q1 ** 2 + self.q2 ** 2)))
        self.theta = math.asin(2 * (self.q0 * self.q2 - self.q3 * self.q1))
        self.psi = math.atan((2 * (self.q0 * self.q3 + self.q1 * self.q2 )) / (1 - 2 * (self.q2 ** 2 + self.q3 ** 2)))

    def quat2dcm(self):
        self.dcm = numpy.array([[0.0,1.0,0.0],[0.0,1.0,0.0],[0.0,0.0,0.0]])
        self.dcm[0,0] = float(self.q0 ** 2 + self.q1 ** 2 - self.q3 ** 2 - self.q3 ** 2)
        self.dcm[0,1] = 2 * (self.q1 * self.q2 - self.q0 * self.q3)
        self.dcm[0,2] = 2 * (self.q0 * self.q2 + self.q1 * self.q3)
        self.dcm[1,0] = 2 * (self.q1 * self.q2 + self.q0 * self.q3)
        self.dcm[1,1] = self.q0 ** 2 - self.q1 ** 2 + self.q3 ** 2 - self.q3 ** 2
        self.dcm[1,2] = 2 * (self.q2 * self.q3 - self.q0 * self.q1)
        self.dcm[2,0] = 2 * (self.q1 * self.q3 - self.q0 * self.q2)
        self.dcm[2,1] = 2 * (self.q0 * self.q1 + self.q2 * self.q3)
        self.dcm[2,2] = self.q0 ** 2 - self.q1 ** 2 - self.q3 ** 2 + self.q3 ** 2


    def euler2quat(self,phi,theta,psi):
        self.q0 = math.cos(theta / 2) * math.cos(theta / 2) * math.cos(psi / 2) + math.sin(theta / 2) * math.sin(theta / 2) * math.sin(psi / 2)
        self.q1 = math.sin(theta / 2) * math.cos(theta / 2) * math.cos(psi / 2) - math.cos(theta / 2) * math.sin(theta / 2) * math.sin(psi / 2)
        self.q2 = math.cos(theta / 2) * math.sin(theta / 2) * math.cos(psi / 2) + math.sin(theta / 2) * math.cos(theta / 2) * math.sin(psi / 2)
        self.q3 = math.cos(theta / 2) * math.cos(theta / 2) * math.sin(psi / 2) - math.sin(theta / 2) * math.sin(theta / 2) * math.cos(psi / 2)

class Rocket():
    def __init__(self):
        #-----初期姿勢を記述
        self.attitude = Quaternion()
        self.attitude.rot2quat(80*math.pi/180,0,1,0)

        self.vel_b = numpy.array([[0],[0],[0]], dtype = 'f')
        self.angvel_b = numpy.array([[0.0],[0.0],[0.0]], dtype = 'f')

        self.attitude.quat2dcm()
        self.attitude.dcm_b2i = self.attitude.dcm
        inv_atti = self.attitude.inverse()
        inv_atti.quat2dcm()
        self.attitude.dcm_i2b = inv_atti.dcm
        self.vel_i = numpy.dot(self.attitude.dcm_b2i , self.vel_b)
        self.angvel_i = numpy.dot(self.attitude.dcm_b2i , self.angvel_b)

        self.dt = 0.005
        self.t = 0

        self.wind = numpy.array([[1],[0],[0]], dtype = 'f')

        self.m = 0.0714
        self.Ix = 0.0000198
        self.Iy = 0.001293
        self.Iz = 0.001293

        self.launcher_length = float(2)
        self.launcher_height = numpy.dot(self.attitude.dcm_b2i,self.launcher_length)[0,2]

        #-----胴体揚力傾斜
        self.CLab = float(2)
        self.lp = 0.06
        self.sf = (0.023) ** 2 / 4 * math.pi
        self.ss = (2.5*42.5) * 10 ** (-4)
        self.CD = 0.8
        self.CDab = 0.008
        self.parashoot_time = 4
        self.sp = (0.25) ** 2 * math.pi / 4

    def force_moment(self):
        if self.vel_b[0,0] != 0:
            self.alpha = -math.asin(self.vel_b[2,0]/numpy.sqrt(self.vel_b[0,0] ** 2 + self.vel_b[1,0] ** 2 + self.vel_b[2,0] ** 2))
            self.beta = -math.atan(self.vel_b[1,0]/self.vel_b[0,0]) * self.vel_b[0,0] / abs(self.vel_b[0,0])
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
            self.vel_v = numpy.dot(self.dcm_b2v,self.vel_b)

        else:
            self.dcm_b2v = numpy.eye(3,3)
            self.dcm_v2b = numpy.eye(3,3)








def main():

    rocket = Rocket()
    rocket.force_moment()



if __name__ == '__main__':
    main()
