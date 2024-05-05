import numpy as np
import roboticstoolbox as rtb

# Define the DH parameters for KUKA iiwa7 
robot = rtb.DHRobot([
    rtb.RevoluteDH(d=0.34, a=0, alpha=-np.pi/2, offset=0),
    rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=0),
    rtb.RevoluteDH(d=0.4, a=0, alpha=np.pi/2, offset=0),
    rtb.RevoluteDH(d=0, a=0, alpha=-np.pi/2, offset=0),
    rtb.RevoluteDH(d=0.4, a=0, alpha=-np.pi/2, offset=0),
    rtb.RevoluteDH(d=0, a=0, alpha=np.pi/2, offset=0),
    rtb.RevoluteDH(d=0.126, a=0, alpha=0, offset=0)
])

def FK(q):
    # Finding coordinates of each joint in the base frame
    n = len(q)
    poses = np.zeros((n, 3))
    for i in range(n):
        poses[i][0] = float(robot.A(i, q).t[0])
        poses[i][1] = float(robot.A(i, q).t[1])
        poses[i][2] = float(robot.A(i, q).t[2])

    return poses

#Test
# q = np.array([np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2, np.pi/2])
# print(FK(q))

                  

    





