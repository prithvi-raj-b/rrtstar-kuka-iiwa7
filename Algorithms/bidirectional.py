import numpy as np
import time
from rrt_util import RrtBase, node

class BidirectionalRrt(RrtBase):

    class agent:
        randgen = np.random.default_rng()

        def __init__(self, outer_class, start, connect_radius, step_size):
            self.start = node(q=np.array(start), parent=None, cost=0)
            self.connect_radius = connect_radius
            self.step_size = step_size
            self.qmin = [-np.pi for _ in range(7)]
            self.qmax = [np.pi for _ in range(7)]
            self.nodes = [self.start]
            self.path_cost = 0
            self.outer_class = outer_class

        def extend(self):
            q = self.sample_point()
            nnint = self.nearest_node(q)
            nearest_node = self.nodes[nnint]
            new_q = self.move_step(nearest_node.q, q)
            if self.outer_class.collision_check(new_q):
                return []
            new_node = node(q=new_q, parent=nnint, cost=nearest_node.cost + np.linalg.norm(new_q - nearest_node.q))
            self.nodes.append(new_node)
            return new_q

        def connect(self, q):
            if len(q) == 0:
                return False
            nnint = self.nearest_node(q)
            nearest_node = self.nodes[nnint]
            
            new_q = self.move_step(nearest_node.q, q)
            while self.outer_class.collision_check(new_q) == False:
                new_node = node(q=new_q, parent=nnint, cost=nearest_node.cost + np.linalg.norm(new_q - nearest_node.q))
                self.nodes.append(new_node)
                if self.connect_check(new_q, q):
                    return True
                new_q = self.move_step(new_q, q)
                nnint = len(self.nodes) - 1
            return False
        
        def connect_check(self, q1, q2):
            if np.linalg.norm(q1 - q2) < self.connect_radius:
                print(q1,q2)
                return True
            return False
        
        def nearest_node(self, q):
            return np.argmin((np.linalg.norm(node.q - q) for node in self.nodes))
        
        def move_step(self, q1, q2):
            mag = np.linalg.norm(q1 - q2)
            if(mag < self.step_size):
                return q2
            else:
                return q1 + self.step_size * (q2 - q1) / mag
        
        def sample_point(self):
            return self.randgen.uniform(self.qmin, self.qmax)
        

    def __init__(self, start, goal, connect_radius, step_size) :
        RrtBase.__init__(self,start)
        self.agent1 = self.agent(self, start, connect_radius, step_size)
        self.agent2 = self.agent(self, goal, connect_radius, step_size)
    
    def solve(self, max_iter):
        start = time.time()
        for i in range(1, max_iter):
            new_q = self.agent1.extend()
            if len(new_q) != 0 and self.agent2.connect(new_q):
                time_taken = time.time() - start
                return (time_taken, len(self.agent1.nodes) + len(self.agent2.nodes))
            new_q = self.agent2.extend()
            if len(new_q) != 0  and self.agent1.connect(new_q):
                time_taken = time.time() - start
                return (time_taken, len(self.agent1.nodes) + len(self.agent2.nodes))
        return None,None
    
    def find_path(self):
        x = len(self.agent1.nodes)-1
        path = []
        while x is not None:
            path.append(self.agent1.nodes[x].q)
            x = self.agent1.nodes[x].parent
        # print(path)
        
        x = len(self.agent2.nodes)-1
        path1 = []
        while x is not None:
            path1.append(self.agent2.nodes[x].q)
            x = self.agent2.nodes[x].parent
        # print(path1)
        
        path.reverse()
        path.extend(path1)
        print(len(path))
        return path
    

def main():
    # start = np.array([1.455, -1.51, 1.25, 0, 0, 0, 0])
    start = np.array([0.7, 1.72, 0, 0, 0, 0, 0])
    # goal = np.array([0, 0.463, 0, -1.786, 0, 0.595, 0])
    goal = np.array([-0.132, -0.198, 0.265, -1.19, -0.132, 1.587, 0])

    connect_radius = 0.2
    step_size = 0.2
    max_iter = 100000
    rrt = BidirectionalRrt(start, goal, connect_radius, step_size)

    time_taken, num_nodes = rrt.solve(max_iter)
    if num_nodes != None:
        path = rrt.find_path()
        eff_pathlen = np.sum(np.linalg.norm(path[i] - path[i+1]) for i in range(len(path)-1))
        print("End-Effector Path Length: ", eff_pathlen)
        # rrt.kuka_sim.performTrajectory(path)
        print(path)
        print("Time taken: ", time_taken)
        print("Number of nodes: ", num_nodes)
        with open("data2.csv", "a") as f:
            f.write(f"b,{time_taken},{num_nodes},{eff_pathlen}\n")
    else:
        print("No path found")

if __name__=='__main__':
    main()
    

    

    
    

            
