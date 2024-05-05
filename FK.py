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
    rtb.RevoluteDH(d=0.126, a=0, alpha=0, offset=0),
    rtb.RevoluteDH(d=0.2, a=0, alpha=0, offset=0)
])

def FK(q):
    # Finding coordinates of each joint in the base frame
    n = len(q)
    qupd = np.zeros(n+1)
    for i in range(n):
        qupd[i] = q[i]

    poses = np.zeros((n+1, 3))
    for i in range(n+1):
        poses[i][0] = robot.A(i, qupd).t[0]
        poses[i][1] = robot.A(i, qupd).t[1]
        poses[i][2] = robot.A(i, qupd).t[2]

    return poses

#Test 
# q= [0, 0, 0, 0, 0, 0, 0]
# print(FK(q)) 