import pybullet as pyb
import pybullet_data
import pyb_utils
from time import sleep
import numpy as np
#import quaternion as quat

TIMESTEP = 1.0 / 60

class kukaSimulator:
    def __init__(self, start_state, margin=0.01):
        self.col_id = pyb.connect(pyb.DIRECT)
        self.gui_id = pyb.connect(pyb.GUI)
        # pyb.setRealTimeSimulation(1, physicsClientId=self.col_id)
        # pyb.setRealTimeSimulation(1, physicsClientId=self.gui_id)
        pyb.resetDebugVisualizerCamera(1.8, 2.0, 0, [0.2, -0.2, 0.7], physicsClientId=self.gui_id)

        self.robot, self.obstacles = self.load_environment(self.col_id)
        self.visual_bot, _ = self.load_environment(self.gui_id)
        self.visual_bot.reset_joint_configuration(start_state)

        self.obstacles.values()
        self.links = [(self.robot.uid, f"lbr_iiwa_link_{x}") for x in range(1, 8)]
        #self.links = [(self.robot.uid,"needle")] + self.links

        self.col_detector = pyb_utils.CollisionDetector(
            self.col_id,
            [(link, obs) for obs in self.obstacles.values() for link in self.links],
        )

        self.margin = margin

    def load_environment(self, client_id):
        pyb.setTimeStep(TIMESTEP, physicsClientId=client_id)
        pyb.setAdditionalSearchPath(
            pybullet_data.getDataPath(), physicsClientId=client_id
        )

        # ground plane
        ground_id = pyb.loadURDF(
            "plane.urdf", [0, 0, 0], useFixedBase=True, physicsClientId=client_id
        )

        # KUKA iiwa robot arm
        kuka_id = pyb.loadURDF(
            "kuka_iiwa/model.urdf",
            [0, 0, 0],
            useFixedBase=True,
            physicsClientId=client_id,
        )
        robot = pyb_utils.Robot(kuka_id, client_id=client_id)

        # some cubes for obstacles
        human_id = pyb.loadURDF(
            "/home/prithvi/bulletSim/obstacles/humanoid.urdf", [0.8, -0.3, 0.6], [0.5, -0.5, 0.5, 0.5], useFixedBase=True, physicsClientId=client_id
        )
        # human_id = pyb.loadURDF(
        #     "obstacles/hollow_box_description/urdf/hollow_box.urdf", [0.7, -0.35, 0.0], [0.5, 0.5, 0.5, 0.5], useFixedBase=True, physicsClientId=client_id
        # )
        # cube1_id = pyb.loadSoftBody(
        #     "/home/prithvi/bulletSim/obstacles/FinalBaseMesh.obj", physicsClientId=client_id
        # )
        cube2_id = pyb.loadURDF(
            "/home/prithvi/bulletSim/obstacles/table/table.urdf", [1.0, 0.2, -0.1],[0,0,0.7068, 0.7073], useFixedBase=True, physicsClientId=client_id
        )
        # cube2_id = pyb.loadURDF(
        #     "obstacles/table/table.urdf", [0.7, 0.2, -0.1],[0,0,0.7068, 0.7073], useFixedBase=True, physicsClientId=client_id
        # )
        # cube3_id = pyb.loadURDF(
        #     "cube.urdf", [1, -1, 0.5], useFixedBase=True, physicsClientId=client_id
        # )

        # store body indices in a dict with more convenient key names
        obstacles = {
            "ground": ground_id,
            "cube1": human_id,
            "cube2": cube2_id,
            # "cube3": cube3_id,
        }

        return robot,obstacles

    def collisionCheck(self, q_start):
        self.robot.reset_joint_configuration(q_start)
        self.visual_bot.reset_joint_configuration(q_start)
        
        # pyb.setJointMotorControlArray(
        #     self.robot.uid,
        #     self.robot._moveable_joint_indices,
        #     pyb.POSITION_CONTROL,
        #     targetPositions=q_end,
        #     targetVelocities=[0.2] * 7,
        #     # maxVelocities=[0.5] * 7,
        #     forces=[500] * 7,
        #     positionGains=[kp] * len(self.robot._moveable_joint_indices),
        #     velocityGains=[kd] * len(self.robot._moveable_joint_indices),
        #     physicsClientId=self.col_id,
        # )
        
        # self.visual_bot.reset_joint_configuration(q_start)
        # pyb.setJointMotorControlArray(
        #     self.visual_bot.uid,
        #     self.visual_bot._moveable_joint_indices,
        #     pyb.POSITION_CONTROL,
        #     targetPositions=q_end,
        #     targetVelocities=[0.2] * 7,
        #     # maxVelocities=[0.5] * 7,
        #     forces=[500] * 7,
        #     positionGains=[kp] * len(self.robot._moveable_joint_indices),
        #     velocityGains=[kd] * len(self.robot._moveable_joint_indices),
        #     physicsClientId=self.gui_id,
        # )
        # pyb.stepSimulation(physicsClientId=self.col_id)
        # pyb.stepSimulation(physicsClientId=self.gui_id)

        if not self.col_detector.in_collision(margin=self.margin):
            return False
        else:
            return True
    
    def performTrajectory(self, points):
        for q in points:
            self.visual_bot.reset_joint_configuration(q)
            sleep(0.2)
            


def main():
    K = kukaSimulator([0,0,0,0,0,0,0])
    K.collisionCheck([0, 1.7, 0, -1.0, 0, 0, 1])
    input()

if __name__ == "__main__":
    main()