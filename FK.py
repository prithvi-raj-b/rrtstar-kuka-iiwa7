import numpy as np 
import roboticstoolbox as rtb
from spatialmath import SE3
from kuka_sim import kukaSimulator


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

def point_on_line(p1, p2, pcheck):
    
        # Vector from p1 to p2
        v = p2 - p1
        # Vector from p1 to the point
        w = pcheck - p1

        # Check if the point is on the line by checking the cross product
        cross_product = np.cross(v, w)
        if np.linalg.norm(cross_product) != 0:
            return False

        # Check if the point is within the bounds of the segment
        dot_product = np.dot(w, v)
        if dot_product < 0:
            return False

        squared_length_p0_p1 = np.dot(v, v)
        if dot_product > squared_length_p0_p1:
            return False

        return True

def rcm(q, q_old):
        
        normal = np.array([0.0, 0.0, 1.0]) # Normal to the plane of the surface

        p7_old = np.array(FK(q_old)[6])
        p8_old = np.array(FK(q_old)[7])

        p7 = np.array(FK(q)[6])
        p8 = np.array(FK(q)[7])

        epsilon = 0
        pentr = p7_old + epsilon*(p8_old - p7_old)

        vec_old = p8_old - p7_old
        vec = p8 - p7

        lamda_old = point_on_line(p7_old, p8_old, pentr)
        lamda = point_on_line(p7, p8, pentr)

        if lamda_old[0] == False or lamda[0] == False :
            return False
        else:
            lamda_old = lamda_old[1]
            lamda = lamda[1]

            if np.linalg.norm(np.cross(vec_old,normal))==0 and np.linalg.norm(np.cross(vec,normal))==0 :
                return True
            elif np.linalg.norm(np.cross(vec_old,normal))==0 and np.linalg.norm(np.cross(vec,normal))!=0 :
                if abs(lamda - lamda_old) < 0.1 :
                    return True
            elif np.linalg.norm(np.cross(vec_old,normal))!=0:
                if abs(lamda - lamda_old)<0.1 :
                    return True
                
        return False

def rcm2(q,qold):
     
        normal = np.array([0.0, 0.0, 1.0]) # Normal to the plane of the surface
        prcm = np.array([0.35,0,0.6])

        #Old path
        p7_0 = np.array(FK(qold)[6])
        p8_0 = np.array(FK(qold)[7])

        lamda_0 = np.dot(prcm-p7_0,p8_0-p7_0)/np.dot(p8_0-p7_0,p8_0-p7_0) #Valid since prcm is on the line

        p7_1 = np.array(FK(q)[6])
        p8_1 = np.array(FK(q)[7])

        #Check if rcm point is on the new path
        if point_on_line(p7_1, p8_1, prcm) ==False:
            return False
        
        #Calculate lamda
        lamda_1 = np.dot(prcm-p7_1,p8_1-p7_1)/np.dot(p8_1-p7_1,p8_1-p7_1)

        #Case :1 Original line was along normal => New line is also along normal
        if np.linalg.norm(np.cross(p8_0-p7_0,normal))==0 and np.linalg.norm(np.cross(p8_1-p7_1,normal))==0 :
            return True
        #Case :2 Original line was not along normal => lamda should be same
        elif np.linalg.norm(np.cross(p8_0-p7_0,normal))!=0 and lamda_1 == lamda_0:
            return True
        #Case :3 Original line was along normal => lamda should be same
        elif np.linalg.norm(np.cross(p8_0-p7_0,normal))==0 and lamda_1 == lamda_0:
            return True
        
        return False

# def rcm3(p7,p7_old):
#      #Sampling p7 to simplify
#     epsilon = 0.1
#     prcm = np.array([0.5,0,0])

#     #Sample p7_new around a semicircle with centre prcm,radius 0.2
#     x = np.random.uniform(prcm[0]-0.1,prcm[0]+0.1)
#     y = np.random.uniform(prcm[1]-0.1,prcm[1]+0.1)
#     z = np.random.uniform(prcm[2],prcm[2]+0.1)
#     p7= np.array([x,y,z])


       
path = []
q_start = np.array([-0.132, -0.198, 0.265, -1.19, -0.132, 1.587, 0,0])

#Target point
rcm = [0.35,0,0.4]
T = SE3(0.35,0,0.6) * SE3.OA([0, 1, 0], [0, 0, -1])
# Do the inverse kinematics to find the joint angles
q = robot.ikine_LM(T, q_start)
q2 = np.array(q.q)
q2 = q2[0:7]
print(q2)

path.append(q2)

p7 = np.array(FK(q2)[6])
p8 = np.array(FK(q2)[7])
lamda = np.dot(rcm-p7,p8-p7)/np.dot(p8-p7,p8-p7)
print(lamda)

#Sample new p7
for i in range(1, 1000):
    p7_new = p7 + np.random.uniform(-0.01,0.01,3)
    q_new = robot.ikine_LM(SE3(p7_new, T.R), q_start)


# for i in range(1, 1000):
     
#     #Sample new p7
    

obj = kukaSimulator(start_state=q_start)
obj.performTrajectory(path)



