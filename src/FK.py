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
        prcm = np.array([0.55,0,0.8])
        r = 0.1

        #Old path
        qold = FK(qold)
        p7_0 = np.array(qold[6])
        p8_0 = np.array(qold[7])

        lamda_0 = point_on_line(p7_0,p8_0,prcm,r)

        q = FK(q)
        p7_1 = np.array(q[6])
        p8_1 = np.array(q[7])

        #Check if rcm point is on the new path
        lamda_1 = point_on_line(p7_1,p8_1,prcm,r)
        if point_on_line(p7_1,p8_1,prcm,r) == None:
            return False
        
        #Calculate lamda
        

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
goal = np.array([0, 0.463, 0, -1.786, 0, 0.595, 0])
path.append(goal)

p7 = np.array(FK(goal)[6])
p8 = np.array(FK(goal)[7])
prcm = np.array([0.55,0,0.3])
r = 0.1
print(point_on_line(p7,p8,prcm,r))

#Sample new p7
for i in range(1, 10000):
    #Sample new q
    q = path[-1] + np.random.normal(0,0.01,7)

    if rcm2(q,path[-1]):
        path.append(q)
        print(q)

obj = kukaSimulator(start_state=goal)
obj.performTrajectory(path)





