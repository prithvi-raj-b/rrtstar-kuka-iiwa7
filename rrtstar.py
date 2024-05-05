

import numpy as np
from kuka_sim import kukaSimulator
import roboticstoolbox as rtb

# Define the DH parameters for KUKA iiwa7

class node:
    def __init__(self, q=None, parent=None, cost=0) :
        self.q = q
        self.parent = parent
        self.cost = cost

class RRTStar:
    randgen = np.random.default_rng()

    def __init__(self, start, goal, goal_radius, step_size, max_iter, rewire_radius=0.1) :
        self.start = node(q=np.array(start), parent=None, cost=0)
        self.goal = node(q=np.array(goal))
        self.qmin = [-np.pi for _ in range(7)]
        self.qmax = [np.pi for _ in range(7)]
        self.step_size = step_size
        self.max_iter = max_iter
        self.goal_radius = goal_radius
        self.nodes = [self.start]
        self.rewire_radius = rewire_radius
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
    
    def sample_point(self):
        return self.randgen.uniform(self.qmin, self.qmax)
    
    def nearest_node(self, q):
        min_dist = np.inf
        min_index = -1
        for i, node in enumerate(self.nodes) :
            dist = np.linalg.norm(node.q - q)
            if dist < min_dist :
                min_dist = dist
                min_index = i
        return min_index
        
    def move_step(self, q1, q2):
        mag = np.linalg.norm(q2 - q1)
        if(mag < self.step_size) :
            return q2
        return q1 + self.step_size * (q2 - q1) / mag
    
    def point_on_line(self, p1, p2, pcheck):
        
        if np.linalg.norm(np.cross(p1-pcheck,p2-pcheck) == 0) :
            lamda = np.dot(pcheck-p1,p2-p1)/np.dot(p2-p1,p2-p1)

            if(lamda>=0 and lamda<=1) :
                return True, lamda
        
        return False, None
    
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
    
    def rcm(self, q, q_old):
        
        pentr= np.array([0.35,0,0.8]) # Entry point of the needle
        normal = np.array([0.0, 0.0, 1.0]) # Normal to the plane of the surface
        
        p7_old = np.array(self.FK(q_old)[6])
        p8_old = np.array(self.FK(q_old)[7])

        p7 = np.array(self.FK(q)[6])
        p8 = np.array(self.FK(q)[7])

        vec_old = p8_old - p7_old
        vec = p8 - p7

        lamda_old = self.point_on_line(p7_old, p8_old, pentr)
        lamda = self.point_on_line(p7, p8, pentr)

        if lamda_old[0] == False or lamda[0] == False :
            return False
        else:
            lamda_old = lamda_old[1]
            lamda = lamda[1]

            if np.linalg.norm(np.cross(vec_old,normal))==0 and np.linalg.norm(np.cross(vec,normal))==0 :
                return True
            elif np.linalg.norm(np.cross(vec_old,normal))==0 or np.linalg.norm(np.cross(vec,normal))!=0 :
                if lamda_old == lamda :
                    return True
            elif np.linalg.norm(np.cross(vec_old,normal))!=0:
                if lamda_old == lamda :
                    return True
                
        return False


    def collision_check(self, q):
        return self.kuka_sim.collisionCheck(q)
    
    def goal_check(self, q):
        return np.linalg.norm(q - self.goal.q) < self.goal_radius
    
    def rewire(self, new_node, radius):

        nearestRadiusNodes = [ i for i,x in enumerate(self.nodes) if np.linalg.norm(x.q - new_node.q) < radius]
        for i in nearestRadiusNodes:
            node = self.nodes[i]
            if np.linalg.norm(node.q - new_node.q) < radius and node.cost + np.linalg.norm(node.q - new_node.q) < new_node.cost :
                # if self.collision_check(new_node.q) :
                #     continue
                new_node.parent = i
                new_node.cost = node.cost + np.linalg.norm(node.q - new_node.q)
        
        for i in nearestRadiusNodes:
            node = self.nodes[i]
            if np.linalg.norm(node.q - new_node.q) < radius and new_node.cost + np.linalg.norm(node.q - new_node.q) < node.cost :
                # if self.collision_check(new_node.q, node.q) :
                #     continue
                node.parent = len(self.nodes)
                node.cost = new_node.cost + np.linalg.norm(node.q - new_node.q)

    
    def run(self):
        
        for i in range(1,self.max_iter) :
            if i % 100 == 0 :
                if self.goalBias()==True:
                    return True
            else:
                q = self.sample_point()

                nnint = self.nearest_node(q)
                nearest_node = self.nodes[nnint]

                new_q = self.move_step(nearest_node.q, q)
                if self.collision_check(new_q):
                    continue
                new_node = node(q=new_q, parent=nnint, cost=nearest_node.cost + np.linalg.norm(new_q - nearest_node.q))
                
                if self.rewire_radius!=None :
                    self.rewire(new_node, self.rewire_radius)
                self.nodes.append(new_node)

                if self.goal_check(new_q) :
                    self.goal.parent = len(self.nodes)-1
                    self.goal.cost = new_node.cost + np.linalg.norm(new_q - self.goal.q)
                    self.nodes.append(self.goal)
                    return True
        
        return False
    
    def goalBias(self):
        q = self.goal.q
        while(True):
            nnint = self.nearest_node(q)
            nearest_node = self.nodes[nnint]
            new_q = self.move_step(nearest_node.q, q)
            if self.collision_check(new_q) :
                return False
            new_node = node(q=new_q, parent=nnint, cost=nearest_node.cost + np.linalg.norm(new_q - nearest_node.q))
            if self.rewire_radius!=None :
                self.rewire(new_node, self.rewire_radius)
            self.nodes.append(new_node)
            if self.goal_check(new_q) :
                self.goal.parent = len(self.nodes)-1
                self.goal.cost = new_node.cost + np.linalg.norm(new_q - self.goal.q)
                self.nodes.append(self.goal)
                return True

# class RRTStarROS(RRTStar):
#     def __init__(self, start, goal, goal_radius, step_size, max_iter, rewire_radius=0.1) :
#         super().__init__(start, goal, goal_radius, step_size, max_iter, rewire_radius)
#         self.pub = rospy.Publisher('joint_trajectory', JointTrajectory, queue_size=10)
    
#     def run(self):
#         if not super().run() :
#             return False
        
#         path = []
#         nint = -1
#         while nint != None :
#             path.append(self.nodes[nint].q)
#             nint = self.nodes[nint].parent

#         jointTrajectoryPoints = [ JointTrajectoryPoint(positions=q, velocities=None, accelerations=None) for q in path ]
#         jointTrajectory = JointTrajectory(joint_names=['iiwa_joint_'+str(x) for x in range(1,8)], points=jointTrajectoryPoints)
#         self.publish(jointTrajectory)
#         return True


def main():
    
    start = np.array([1.455, -1.51, 1.25, 0, 0, 0, 0])
    # start = np.array([0, 1.72, 0,0, 0, 0, 0])
    # goal = np.array([0, 0.066, 0.2, -1.257, -0.265, 1, 0.066])
    goal = np.array([-0.132, -0.198, 0.265, -1.19, -0.132, 1.587, 0])
    goal_radius = 0.1
    step_size = 0.1
    max_iter = 500000
    rewire_radius = None

    rrt = RRTStar(start, goal, goal_radius, step_size, max_iter, rewire_radius)
    rrt.run()

    path = []
    nint = -1
    while nint != None :
        path.append(rrt.nodes[nint].q)
        nint = rrt.nodes[nint].parent
    
    path.reverse()

    print(path, len(rrt.nodes))
    
    rrt.kuka_sim.performTrajectory(path)
    input()

if __name__ == '__main__':
    main()
    

