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
        print(p_mat)
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
        qd1 = self.q1 / norm
        qd2 = self.q2 / norm
        qd3 = self.q3 / norm

        return Quaternion(qd0,qd1,qd2,qd3)

    def rot2quat(self,theta,x,y,z):
        if self == (Quaternion):
            return Quaternion(math.cos(theta / 2), x * math.sin(theta / 2), y * math.sin(theta / 2), z * math.sin(theta / 2))
        else:
            self = Quaternion()
            return Quaternion(math.cos(theta / 2), x * math.sin(theta / 2), y * math.sin(theta / 2), z * math.sin(theta / 2))

    def dq_dt(self,ox, oy, oz):
        buff = Quaternion()
        buff = self * Quaternion(0, ox, oy, oz)
        buff.q0  = buff.q0 / 2
        buff.q1 / 2
        buff.q2 / 2
        buff.q3 / 2
        print(buff.q0)
        return buff



def main():
    q = Quaternion();
    q.q0 = 2
    p = Quaternion()
    p.q2 = 1

    j = Quaternion()
    j = j.dq_dt(0,0,1)


    print(j.q0)
    print(j.q1)
    print(j.q2)
    print(j.q3)

if __name__ == '__main__':
    main()
