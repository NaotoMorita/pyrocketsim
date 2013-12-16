#-------------------------------------------------------------------------------
# Name:        module1
# Purpose:
#
# Author:      NaotoMORITA
#
# Created:     03/12/2013
# Copyright:   (c) NaotoMORITA 2013
# Licence:     <your licence>
#-------------------------------------------------------------------------------
import numpy
import math

class Quaternion():
    def __init__(self,quat = [[1.0],[0.0],[0.0],[0.0]]):
        self.quat = quat


    def __truediv__(self,p):
        if isinstance(p,Quaternion):
            p_inv = p.inverse()
            return self * p_inv

        elif isinstance(p,int) or isinstance(p,float):
            q = numpy.array(self.quat,dtype = "float_")
            q /= float(p)
            return numpy.ndarray.tolist(q)

    def __mul__(self,p):

        if isinstance(p,Quaternion):
            q = numpy.array(self.quat,dtype = "float_")
            p = numpy.array(p.quat,dtype = "float_")
            q_mat = numpy.zeros((4,4))

            q_mat[0,0] =  q[0]
            q_mat[0,1] = -q[1]
            q_mat[0,2] = -q[2]
            q_mat[0,3] = -q[3]

            q_mat[1,0] =  q[1]
            q_mat[1,1] =  q[0]
            q_mat[1,2] = -q[3]
            q_mat[1,3] =  q[2]

            q_mat[2,0] =  q[2]
            q_mat[2,1] =  q[3]
            q_mat[2,2] =  q[0]
            q_mat[2,3] = -q[1]

            q_mat[3,0] =  q[3]
            q_mat[3,1] = -q[2]
            q_mat[3,2] =  q[1]
            q_mat[3,3] =  q[0]

            quat_qp = numpy.dot(q_mat,p)

            return numpy.ndarray.tolist(quat_qp)

        if isinstance(p,list):
            q = numpy.array(self.quat,dtype = "float_")
            p = numpy.array(p,dtype = "float_")
            q_mat = numpy.zeros((4,4))

            q_mat[0,0] =  q[0]
            q_mat[0,1] = -q[1]
            q_mat[0,2] = -q[2]
            q_mat[0,3] = -q[3]

            q_mat[1,0] =  q[1]
            q_mat[1,1] =  q[0]
            q_mat[1,2] = -q[3]
            q_mat[1,3] =  q[2]

            q_mat[2,0] =  q[2]
            q_mat[2,1] =  q[3]
            q_mat[2,2] =  q[0]
            q_mat[2,3] = -q[1]

            q_mat[3,0] =  q[3]
            q_mat[3,1] = -q[2]
            q_mat[3,2] =  q[1]
            q_mat[3,3] =  q[0]
            quat_qp = numpy.dot(q_mat,p)

            return numpy.ndarray.tolist(quat_qp)

        elif isinstance(p,int) or isinstance(p,float):
            q = numpy.array(self.quat,dtype = "float_")
            q *= p
            return numpy.ndarray.tolist(q)

    def norm(self):
        quat = numpy.array(self.quat,dtype = "float_")
        return numpy.sqrt(quat[0,0] ** 2  + quat[1,0] ** 2 + quat[2,0] ** 2 + quat[3,0] ** 2)

    def inverse(self):
        norm = self.norm()
        quat = numpy.array(self.quat,dtype = "float_")
        quat[1] = float(-quat[1])
        quat[2] = float(-quat[2])
        quat[3] = float(-quat[3])
        return numpy.ndarray.tolist(quat / norm)

    def normalize(self):
        norm = self.norm()
        quat = numpy.array(self.quat,dtype = "float_")
        self.quat = numpy.ndarray.tolist(quat / norm )



    def rot2quat(self,theta,x,y,z):
        self.quat[0][0] =     numpy.cos(theta / 2)
        self.quat[1][0] = x * numpy.sin(theta / 2)
        self.quat[2][0] = y * numpy.sin(theta / 2)
        self.quat[3][0] = z * numpy.sin(theta / 2)
        self.normalize()



    def dq_dt(self, Ox, Oy, Oz):
        self.normalize()
        quat = numpy.array(self.quat,dtype = "float_")
        dq = numpy.zeros((4,1),dtype = "float_")
        dq[0,0] = -0.5 * (quat[1] * Ox + quat[2] * Oy + quat[3] * Oz)
        dq[1,0] =  0.5 * (quat[0] * Ox - quat[3] * Oy + quat[2] * Oz)
        dq[2,0] =  0.5 * (quat[3] * Ox + quat[0] * Oy - quat[1] * Oz)
        dq[3,0] = -0.5 * (quat[2] * Ox - quat[1] * Oy - quat[0] * Oz)
        norm = numpy.sqrt(dq[0,0] ** 2 + dq[1,0] ** 2 + dq[2,0] ** 2 + dq[3,0] ** 2)
        dq = dq / norm
        return numpy.ndarray.tolist(dq)
        #omega_quat = Quaternion([[0],[Ox],[Oy],[Oz]])
        #dq = numpy.array(self * omega_quat,dtype = "float_") / 2

        #return numpy.ndarray.tolist(dq)

    def quat2dcm(self):
        dcm = numpy.zeros((3,3),dtype = "float_")
        quat = numpy.array(self.quat,dtype = "float_")
        dcm[0,0] =  float(quat[0] ** 2 + quat[1] ** 2 - quat[2] ** 2 - quat[3] ** 2)
        dcm[0,1] = 2.0 * (quat[1] * quat[2] - quat[0] * quat[3])
        dcm[0,2] = 2.0 * (quat[0] * quat[2] - quat[1] * quat[3])
        dcm[1,0] = 2.0 * (quat[1] * quat[2] + quat[0] * quat[3])
        dcm[1,1] =  float(quat[0] ** 2 - quat[1] ** 2 + quat[2] ** 2 - quat[3] ** 2)
        dcm[1,2] = 2.0 * (quat[2] * quat[3] - quat[0] * quat[1])
        dcm[2,0] = 2.0 * (quat[1] * quat[3] - quat[0] * quat[2])
        dcm[2,1] = 2.0 * (quat[0] * quat[1] + quat[2] * quat[3])
        dcm[2,2] =  float(quat[0] ** 2 - quat[1] ** 2 - quat[2] ** 2 + quat[3] ** 2)
        return numpy.ndarray.tolist(dcm)

    def quat2euler(self):
        quat = numpy.array(self.quat,dtype = "float_")
        phi = numpy.arctan((2 * (quat[0,0] * quat[1,0] + quat[2,0] * quat[3,0] )) / (1 - 2 * (quat[1,0] ** 2 + quat[2,0] ** 2)))
        theta = numpy.arcsin(2 * (quat[0,0] * quat[2,0] - quat[3,0] * quat[1,0]))
        psi = numpy.arctan((2 * (quat[0,0] * quat[3,0] + quat[1,0] * quat[2,0] )) / (1 - 2 * (quat[2,0] ** 2 + quat[3,0] ** 2)))
        return numpy.ndarray.tolist(numpy.array([[phi],[theta],[psi]]))

def main():
    pass

if __name__ == '__main__':
    main()
