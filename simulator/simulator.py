import os
import time
import pybullet as p
from enum import Enum

from .tools import getPath


class RobotVersion(Enum):
    GRIPPER = 0
    PEN = 1


class Simulator:

    gui: bool
    joint_mapping = {0: 2, 1: 3, 2: 4, 3: 5, 4: 6, 5: 7}

    def __init__(
        self,
        robot_version: RobotVersion = RobotVersion.PEN,
        gui: bool = False,
        deltaT: float = 1 / 100,
        log=False,
    ):

        self.gui = gui


        self.log = log

        if self.gui:
            p.connect(p.GUI)
            self.stepSimu = lambda: p.stepSimulation();time.sleep(deltaT)
        else:
            p.connect(p.DIRECT)
            self.stepSimu = lambda: p.stepSimulation()
        p.setTimeStep(deltaT)

        if robot_version == RobotVersion.PEN:
            self.robot_id = p.loadURDF(
                getPath("urdfs/iscoin_pen.urdf"),
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION,
            )
        elif robot_version == RobotVersion.GRIPPER:
            self.robot_id = p.loadURDF(
                getPath("urdfs/iscoin_gripper.urdf"),
                useFixedBase=True,
                flags=p.URDF_USE_SELF_COLLISION,
            )
        else:
            raise Exception("Robot version not supported")

        self.ground_id = p.loadURDF(getPath("urdfs/plane.urdf"), basePosition=[0, 0, 0])

        self.pen_joint = 11

        self._log("Simulator started")

       
    
    def _log(self, *args):
        if self.log:
            print("[SIMULATOR] ", *args)

    def _isAtPosition(self, angles, tolerance=1e-2)-> bool:
        joint_positions = [p.getJointState(self.robot_id, self.joint_mapping[i])[0] for i in self.joint_mapping]
        for i, a in enumerate(joint_positions):
            if abs(angles[i] - a) > tolerance:
                return False
        return True
    
    def _getLinkName(self, joint_id)->str:
        return (p.getJointInfo(self.robot_id, joint_id)[12]).decode()


    def resetAtPosition(self, angles):
        for i, joint_id in enumerate(self.joint_mapping.values()):
            p.resetJointState(self.robot_id, joint_id, angles[i])


    def isPositionValid(self, angles) -> tuple[bool, list[tuple[str, str]]]:
        self.resetAtPosition(angles)
        p.stepSimulation()
        return self.check_collision()




    def isMoveValid(self, startAngles, destAngles):
        self.resetAtPosition(startAngles)
        return self.moveToAngles(destAngles, checkCollision=True)


    def motorMove(self, angles, force=10):
        for i, a in enumerate(angles):
            p.setJointMotorControl2(
                bodyUniqueId=self.robot_id,
                jointIndex=self.joint_mapping[i],
                controlMode=p.POSITION_CONTROL,
                targetPosition=a,
                force=force
            )

    def moveToAngles(self, angles, checkCollision=False):
        self.motorMove(angles)
        collided = False
        while not self._isAtPosition(angles):
            self.stepSimu()
            if checkCollision and self.check_collision():
                collided = True
        return collided

    def isPenInArea(self, x_bounds=(-0.0, 0.5), y_bounds=(-0.5, 0.5), z_bounds=(0, 0.5)):
        pen_pos = p.getLinkState(self.robot_id, self.pen_joint)[0]
        isInArea = True
        if pen_pos[0] < x_bounds[0] or pen_pos[0] > x_bounds[1]:
            self._log(f"The pen is currently out of the working area in X. (current X : {pen_pos[0]}, min - max : {x_bounds[0]} - {x_bounds[1]})")
            isInArea = False
        if pen_pos[1] < y_bounds[0] or pen_pos[1] > y_bounds[1]:
            self._log(f"The pen is currently out of the working area in Y. (current Y : {pen_pos[0]}, min - max : {y_bounds[0]} - {y_bounds[1]})")
            isInArea = False
        if pen_pos[2] < z_bounds[0] or pen_pos[2] > z_bounds[1]:
            self._log(f"The pen is currently out of the working area in Z. (current Z : {pen_pos[0]}, min - max : {z_bounds[0]} - {z_bounds[1]})")
            isInArea = False
        # If the pen is within the working area, return True
        return isInArea

    def check_collision(self) -> bool:
        contact_points_robot = p.getContactPoints(self.robot_id, self.robot_id)  # Self-collision check
        contact_points_ground = p.getContactPoints(self.robot_id, self.ground_id)  # Ground collision check

        isNotInCollision = True

        def getLinkName(id):
            return (p.getJointInfo(self.robot_id, id)[12]).decode()

        # Check for self-collisions (ignore wrist-pen collision)
        if contact_points_robot:
            for contact in contact_points_robot:
                if getLinkName(contact[3]) == "wrist_3_link" and getLinkName(contact[4]) == "pen_link":
                    continue
                self._log(f"⚠️ Self-Collision: {getLinkName(contact[3])} and {getLinkName(contact[4])} are colliding.")
                isNotInCollision = False

        # Check for ground collision
        if contact_points_ground:
            for contact in contact_points_ground:
                if getLinkName(contact[3]) == "base_link_inertia":
                    continue
                if getLinkName(contact[3]) == "pen_link":
                    self._log("Collision with Pen: The pen touched the ground!, but we ignore it")
                    continue
                self._log(f"⚠️ Collision with Ground: {getLinkName(contact[3])} touched the ground!")
                isNotInCollision = False  # Collision detected

        #if not self.check_working_area():
        #    isInCollision = False

        return isNotInCollision
        




if __name__ == "__main__":
    simu = Simulator(gui = True, log=True)
    print(simu._getLinkName(11))

    # Check for potential self-collisions within 5mm
    # close_contacts_robot = p.getClosestPoints(self.robot_id, self.robot_id, distance=threshold_distance)
    # close_contacts_ground = p.getClosestPoints(self.robot_id, self.ground_id, distance=threshold_distance)

    # Check if a collision is **about to happen** within 5mm
    # for contact in close_contacts_robot:
    #     if (contact[3] == contact[4] or
    #         abs(contact[3] - contact[4]) == 1 or
    #         getLinkName(contact[3]) == "pen_link" or
    #         getLinkName(contact[4]) == "pen_link"):  # Exclude collisions between the same link, adjacent links, or involving the pen link
    #         continue
    #     if self.logs:
    #         print(f"WARNING: {getLinkName(contact[3])} is too close to {getLinkName(contact[4])} (within {threshold_distance})!")
    #     isInCollision = False  # Prevent movement if too close

    # for contact in close_contacts_ground:
    #     if getLinkName(contact[3]) == "base_link_inertia":
    #         continue
    #     if self.logs:
    #         print(f"WARNING: {getLinkName(contact[3])} is too close to the ground (within 5mm)!")
    #     isInCollision = False  # Prevent movement if too close