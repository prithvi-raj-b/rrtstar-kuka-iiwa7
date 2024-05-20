import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from rrt_util import RrtBase, node

def point_on_line(p1, p2, pcheck,r):

    # Point of intersection of line and plane
    vec = p2 - p1

    if vec[2]==0:
        return None
    
    lamda = (pcheck[2] - p1[2])/vec[2]

    if lamda < 0 or lamda > 1:
        return None

    #Find the point on the line
    p = p1 + lamda*vec

    #Check if the point is within the radius
    if np.linalg.norm(p-pcheck) < r:
        return lamda
    else:
        return None

class RrtRcm(RrtBase):

    def __init__(self, start = np.array([0.55, 0, 0.3])):
        self.r = 0.025
        RrtBase.__init__(self, start)
        del1 = np.array([0.025,0.025,0.1])
        self.prcm = np.array([0.55, 0, 0.2])
        self.qrcm = np.array([0, 0.463, 0, -1.786, 0, 0.595, 0, 0])
        self.qmin = self.prcm - del1
        self.qmax = self.prcm + del1
        self.nodes = [node((start,(0,0,-1)))]

    def gaussian_sample(self):
        res = self.prcm + np.random.normal(0,0.01,3)
        ang = np.array([0,0,-1]) + np.random.normal(0,0.1,3)
        return res,ang
    
    
    # def expand(self):
    #     for i in range(1000):
    #         q = self.gaussian_sample()
    #         if all(q>self.qmin) and all(q<self.qmax):
    #             self.nodes.append(q)

    def FK(self,q):
        # Finding coordinates of each joint in the base frame
        n = len(q)
        qupd = np.zeros(n)
        for i in range(n):
            qupd[i] = q[i]

        poses = np.zeros((n, 3))
        for i in range(n):
            poses[i][0] = self.robot.A(i, qupd).t[0]
            poses[i][1] = self.robot.A(i, qupd).t[1]
            poses[i][2] = self.robot.A(i, qupd).t[2]

        return poses
    
    def IK(self, pose):
        # T = self.robot.fkine(old_pose)
        #SE3 at angle 45 degrees
        pose,ang = pose
        T = SE3(pose[0],pose[1],pose[2])*SE3.OA([0,1,0],ang)
        q = self.robot.ikine_LM(T,self.qrcm)
        return q.q
    
    def collision_check(self, pose):
        q = self.IK(pose)
        if self.rcm(q,self.qrcm):
            return self.kuka_sim.collisionCheck(q)
        return False
    
    def nearest(self, q):
        return np.argmin((np.linalg.norm(node.q - q) for node in self.nodes))
    
    def extend(self):
        q = self.gaussian_sample()
        nn = self.nearest(q)
        if not self.collision_check(q):
            self.nodes.append(node(q, parent=nn, cost=0))
    
    def do_rcm(self):
        for i in range(100):
            self.extend()
    
    def put_trajectory(self):
        traj = [self.IK(node.q) for node in self.nodes]
        self.kuka_sim.performTrajectory(traj)
    
    def rcm(self,q, q_old):
        
        normal = np.array([0.0, 0.0, 1.0]) # Normal to the plane of the surface

        q_old = self.FK(q_old)
        p7_old = np.array(q_old[6])
        p8_old = np.array(q_old[7])

        q = self.FK(q)
        p7 = np.array(q[6])
        p8 = np.array(q[7])

        epsilon = 0
        pentr = p7_old + epsilon*(p8_old - p7_old)

        vec_old = p8_old - p7_old
        vec = p8 - p7

        lamda_old = point_on_line(p7_old, p8_old, pentr, self.r)
        lamda = point_on_line(p7, p8, pentr,self.r)

        if lamda_old == None or lamda == None :
            return False
        else:
            lamda_old = lamda_old
            lamda = lamda

            if np.linalg.norm(np.cross(vec_old,normal))==0 and np.linalg.norm(np.cross(vec,normal))==0 :
                return True
            elif np.linalg.norm(np.cross(vec_old,normal))==0 and np.linalg.norm(np.cross(vec,normal))!=0 :
                if abs(lamda - lamda_old) < 0.1 :
                    return True
            elif np.linalg.norm(np.cross(vec_old,normal))!=0:
                if abs(lamda - lamda_old)<0.1 :
                    return True
                
        return False
    


def main():
    rcm = RrtRcm()
    rcm.do_rcm()
    # rcm.put_trajectory()
    
    # print([np.linalg.norm(node.q[0],rcm.FK([0, 0.463, 0, -1.786, 0, 0.595, 0, 0])[-1]) for node in rcm.nodes])

    ls = []
    for node in rcm.nodes:
        s = 0
        for x,y in zip(node.q[0],rcm.FK([0, 0.463, 0, -1.786, 0, 0.595, 0, 0])[-1]):
            s += x**2 + y**2
        ls.append(np.sqrt(s))
    print('\n',max(ls),'\n')
    with open("rcm.txt",'a') as f:
        f.write(str(max(ls))+'\n')

    # rcm.kuka_sim.performTrajectory([node.q for node in rcm.nodes])
    

if __name__ == "__main__":
    main()

# start [0.009667144368148826, 0.04853695552499554, -0.8674214203447641]
# end [-0.07380120336364547, -0.06900521324995111, -0.9432854143674948]