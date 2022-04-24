import os
import time
import pybullet as p


def loadWorld():
    p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    p.loadURDF(os.path.join(os.getcwd(), "plane.urdf"), 0, 0, 0)


def loadGripper():
    robotPos = [0, 0, 0]
    robotOrn = p.getQuaternionFromEuler([0, 0, 0])
    gripper = p.loadURDF(os.path.join(os.getcwd(), "gripper.urdf"), robotPos, robotOrn)
    p.createConstraint(gripper, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, -1], [0, 0, 0])
    return gripper


def reachXY(x, y, u, g, gripper):
    while abs(p.getLinkState(gripper, 3)[0][0] - x) > 0.0005 or abs(p.getLinkState(gripper, 3)[0][1] - y + 0.176 - g) > 0.005:
        p.setJointMotorControl2(gripper, 0, p.POSITION_CONTROL, -7.5 + x, force=10, maxVelocity=20)
        p.setJointMotorControl2(gripper, 1, p.POSITION_CONTROL, -7.5 + y - 0.076, force=10, maxVelocity=20)
        p.setJointMotorControl2(gripper, 4, p.POSITION_CONTROL, u, force=60, maxVelocity=20)
        time.sleep(1. / 240.)
        p.stepSimulation()


def lowerDown(u, gripper):
    while p.getLinkState(gripper, 3)[0][2] > 0.05:
        p.setJointMotorControl2(gripper, 2, p.POSITION_CONTROL, -1.45, force=70)
        p.setJointMotorControl2(gripper, 4, p.POSITION_CONTROL, u, force=60)
        time.sleep(1. / 240.)
        p.stepSimulation()


def liftUp(u, gripper):
    while p.getLinkState(gripper, 3)[0][2] < 0.5:
        p.setJointMotorControl2(gripper, 2, p.POSITION_CONTROL, 0, force=100)
        p.setJointMotorControl2(gripper, 4, p.POSITION_CONTROL, u, force=60)
        time.sleep(1. / 240.)
        p.stepSimulation()


def Gripper(a, gripper):
    i = 0
    while i < 200:
        p.setJointMotorControl2(gripper, 3, p.POSITION_CONTROL, a, force=10)
        p.setJointMotorControl2(gripper, 4, p.POSITION_CONTROL, 0, force=60)
        i += 1
        time.sleep(1. / 240.)
        p.stepSimulation()


def lock(a, gripper):
    i = 0
    while i < 200:
        p.setJointMotorControl2(gripper, 4, p.POSITION_CONTROL, a, force=60)
        i += 1
        time.sleep(1. / 240.)
        p.stepSimulation()


def pick(x, y, gripper):
    reachXY(x, y, 0, 0, gripper)
    lowerDown(0, gripper)
    Gripper(0.13, gripper)
    lock(0.02, gripper)
    liftUp(0.02, gripper)


def place(x, y, gripper):
    reachXY(x, y, 0.02, 0.13, gripper)
    lowerDown(0.02, gripper)
    lock(0, gripper)
    Gripper(0, gripper)
    liftUp(0, gripper)
