#!/usr/bin/env python3

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from kuka_sim import kukaSimulator

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

    def collision_check(self, q):
        return self.kuka_sim.collisionCheck(q)
    
    def goal_check(self, q):
        return np.linalg.norm(q - self.goal.q) < self.goal_radius
    
    def rewire(self, new_node, radius):
        for i,node in enumerate(self.nodes) :
            if np.linalg.norm(node.q - new_node.q) < radius and node.cost + np.linalg.norm(node.q - new_node.q) < new_node.cost :
                new_node.parent = i
                new_node.cost = node.cost + np.linalg.norm(node.q - new_node.q)
    
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
                if self.collision_check(new_q) :
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
    goal = np.array([0, 0.066, 0.2, -1.257, -0.265, 1, 0.066])
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
    

