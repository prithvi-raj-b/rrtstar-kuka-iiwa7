import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3
from rrt_util import RrtBase, node

class RrtRcm(RrtBase):

    def __init__(self, start = np.array([0.55, 0, 0.3])):
        # r = 0.025
        RrtBase.__init__(self, start)
        del1 = np.array([0.025,0.025,0.1])
        self.prcm = np.array([0.55, 0, 0.3])
        self.qrcm = np.array([0, 0.463, 0, -1.786, 0, 0.595, 0, 0])
        self.qmin = self.prcm - del1
        self.qmax = self.prcm + del1
        self.nodes = [node(start)]

    def gaussian_sample(self):
        return self.prcm + np.random.normal(0,0.01,3)
    
    # def expand(self):
    #     for i in range(1000):
    #         q = self.gaussian_sample()
    #         if all(q>self.qmin) and all(q<self.qmax):
    #             self.nodes.append(q)

    def FK(self,q):
        # Finding coordinates of each joint in the base frame
        n = len(q)
        qupd = np.zeros(n+1)
        for i in range(n):
            qupd[i] = q[i]

        poses = np.zeros((n+1, 3))
        for i in range(n+1):
            poses[i][0] = self.robot.A(i, qupd).t[0]
            poses[i][1] = self.robot.A(i, qupd).t[1]
            poses[i][2] = self.robot.A(i, qupd).t[2]

        return poses
    
    def IK(self, pose, old_pose):
        # T = self.robot.fkine(old_pose)
        T = SE3(pose[0],pose[1],pose[2])*SE3.OA([0,1.0],[0,0,-1])
        q = self.robot.ikine_LM(T,self.qrcm)
        return q.q
    
    def collision_check(self, pose, old_pose):
        q = self.IK(pose, old_pose)
        return self.kuka_sim.collisionCheck(q)
    
    def nearest(self, q):
        return np.argmin((np.linalg.norm(node.q - q) for node in self.nodes))
    
    def extend(self):
        q = self.gaussian_sample()
        nn = self.nearest(q)
        if all(q>self.qmin) and all(q<self.qmax) and not self.collision_check(q, self.nodes[nn].q):
            self.nodes.append(node(q, parent=nn, cost=0))
    
    def do_rcm(self):
        for i in range(100):
            self.extend()
    
    def put_trajectory(self):
        traj = [self.IK(node.q,self.prcm) for node in self.nodes]
        self.kuka_sim.performTrajectory(traj)
        return
    


def main():
    rcm = RrtRcm()
    rcm.do_rcm()
    rcm.put_trajectory()

    # rcm.kuka_sim.performTrajectory([node.q for node in rcm.nodes])
    

if __name__ == "__main__":
    main()