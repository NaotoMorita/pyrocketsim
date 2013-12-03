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

class Quaternion():
    def __init__(self,quat = [[1],[0],[0],[0]]):
        self.quat = quat

    def __mul__(self,r):
        if isinstance(r,Quaternion):
            q = numpy.array(self.quat)
            r = numpy.array(r.quat)
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

            quat_qr = numpy.dot(q_mat,r)

            return numpy.ndarray.tolist(quat_qr)

        elif isinstance(r,int) or isinstance(r,float):
            q = numpy.array(self.quat)
            q *= r
            print("b")
            return numpy.ndarray.tolist(q)

    def inverse(self):
        return

    def __div__(self,r):
        if isinstance(r,Quaternion):
            q_inv =

        elif isinstance(r,int) or isinstance(r,float):
            q = numpy.array(self.quat)
            q *= float(r)
            print("b")
            return numpy.ndarray.tolist(q)


        #q_buff = [[self.q0,-self.q1,-self.q2,-self.q3],
        #[self.q1,self.q0,-self.q3,self.q2],
        #[self.q2,self.q3,self.q0,-self.q1],
        #[self.q3,-self.q2,self.q1,self.q0]]




def main():
    q = Quaternion()
    p = Quaternion()
    qp = q*p
    print(q*4)




if __name__ == '__main__':
    main()
