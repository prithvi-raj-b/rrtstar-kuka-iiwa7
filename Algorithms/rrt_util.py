import numpy as np
from kuka_sim import kukaSimulator
import roboticstoolbox as rtb

class node:
    def __init__(self, q=None, parent=None, cost=0) :
        self.q = q
        self.parent = parent
        self.cost = cost

class RrtBase:
    def __init__(self,start):
        self.kuka_sim = kukaSimulator(start_state=start)
        self.robot = rtb.DHRobot([
            rtb.RevoluteDH(d=0.34, a=0, alpha=-np.pi/2, offset=0),
            rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=0),
            rtb.RevoluteDH(d=0.4, a=0, alpha=np.pi/2, offset=0),
            rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2, offset=0),
            rtb.RevoluteDH(d=0.4, a=0, alpha=-np.pi/2, offset=0),
            rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=0),
            rtb.RevoluteDH(d=0.126, a=0, alpha=0, offset=0),
            rtb.RevoluteDH(d=0.2, a=0, alpha=0, offset=0)
        ])
    
    def collision_check(self, q):
        return self.kuka_sim.collisionCheck(q)
    

