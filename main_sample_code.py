import pybullet as p
import time
import math
import numpy as np

############################################### Environment Setup ####################################################
p.connect(p.GUI)

p.resetSimulation()

p.setGravity(0, 0, -10)
useRealTimeSim = 0

p.setRealTimeSimulation(useRealTimeSim)  # either this

# load plane
track = p.loadURDF("data/plane/plane.urdf")
# load car
car = p.loadURDF("f10_racecar/racecar_differential.urdf", [0, 0, 0])


# load obstacles, in this projects, we used six cube as obstacles

def random_obstacles():
    np.random.seed()
    xy_position = [0, 0]
    xy_position_float = np.random.rand(2)
    x_poistion_range = np.random.randint(1, 10)
    y_poistion_range = np.random.randint(1, 10)

    xy_position[0] = xy_position_float[0] + x_poistion_range
    xy_position[1] = xy_position_float[1] + y_poistion_range

    np.asarray(xy_position)
    position = np.append(xy_position, 0.2)
    return position


cube_1_position = random_obstacles()
cube_1 = p.loadURDF('data/cube/marble_cube.urdf',cube_1_position)

cube_2_position = random_obstacles()
cube_2 = p.loadURDF('data/cube/marble_cube.urdf',cube_2_position)

cube_3_position = random_obstacles()
cube_3 = p.loadURDF('data/cube/marble_cube.urdf',cube_3_position)

cube_4_position = random_obstacles()
cube_4 = p.loadURDF('data/cube/marble_cube.urdf',cube_4_position)

cube_5_position = random_obstacles()
cube_5 = p.loadURDF('data/cube/marble_cube.urdf',cube_5_position)

cube_6_position = random_obstacles()
cube_6 = p.loadURDF('data/cube/marble_cube.urdf',cube_6_position)


cube_7_position = random_obstacles()
cube_7 = p.loadURDF('data/cube/marble_cube.urdf',cube_7_position)

cube_8_position = random_obstacles()
cube_8 = p.loadURDF('data/cube/marble_cube.urdf',cube_8_position)


for wheel in range(p.getNumJoints(car)):
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
    p.getJointInfo(car, wheel)

wheels = [8, 15]
print("----------------")

c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)
c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0, 1, 0], parentFramePosition=[0, 0, 0],
                       childFramePosition=[0, 0, 0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

steering = [0, 2]

hokuyo_joint = 4


replaceLines = True

# numRays = 30
numRays = 50
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1, 0, 0]
rayMissColor = [0, 1, 0]
rayLen = 8
rayStartLen = 0.25
for i in range(numRays):
    rayFrom.append([rayStartLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                    rayStartLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    rayTo.append([rayLen * math.sin(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays),
                  rayLen * math.cos(-0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays), 0])
    if (replaceLines):
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car,
                                         parentLinkIndex=hokuyo_joint))
    else:
        rayIds.append(-1)

frame = 0
lineId = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId2 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
lineId3 = p.addUserDebugLine([0, 0, 0], [0, 0, 1], [1, 0, 0])
print("lineId=", lineId)
lastTime = time.time()
lastControlTime = time.time()
lastLidarTime = time.time()

def drive_the_mobile(carPos, carOrn, turn_angle,final_goal_pos):

    steeringAngle = 1 * turn_angle
    if steeringAngle > 1:
        steeringAngle = 1
    if steeringAngle < -1:
        steeringAngle = -1

    distance = np.sqrt(np.square(final_goal_pos[0] - carPos[0]) + np.square(final_goal_pos[1] - carPos[1]))

    stop_threshold = 1
    if distance < stop_threshold:
        targetVelocity = 0
    else:
        targetVelocity = 25

    for wheel in wheels:
        p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=targetVelocity, force=maxForce)
    for steer in steering:
        print("steeringAngle", np.degrees(steeringAngle))
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)


def navigate_mobile(carPos, carOrn, sensor_readings, targetPos):
    carEuler = p.getEulerFromQuaternion(carOrn)
    carYaw = carEuler[2]
    hitTo_angle = []
    hitTo_Fraction = []

    angle_from_target_to_car = np.arctan2((targetPos[1] - carPos[1]), (targetPos[0] - carPos[0])) - carYaw
    if angle_from_target_to_car < -math.pi:
        angle_from_target_to_car = angle_from_target_to_car + 2 * math.pi
    if angle_from_target_to_car > math.pi:
        angle_from_target_to_car = angle_from_target_to_car - 2 * math.pi

    for i, sensor_reading in enumerate(sensor_readings):
        hitFraction = sensor_reading[2]

        localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                      rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                      rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]

        rayangle_in_car_1 = np.arctan2((localHitTo[1] - rayFrom[i][1]), (localHitTo[0] - rayFrom[i][0]))

        hitTo_angle.append(rayangle_in_car_1)
        hitTo_Fraction.append(hitFraction)

    min_angle_difference = 2 * math.pi
    min_rayIndex = np.round(numRays / 2)
    for i in range(0,len(hitTo_Fraction)):
        if hitTo_Fraction[i] > 0.3: #0.7s
            angle_difference = abs(hitTo_angle[i] - angle_from_target_to_car)
            if angle_difference < min_angle_difference:
                min_angle_difference = angle_difference
                min_rayIndex = i       

    return hitTo_angle[min_rayIndex]



while (True):
    nowTime = time.time()
    nowControlTime = time.time()
    nowLidarTime = time.time()
    # lidar at 20Hz
    if (nowLidarTime - lastLidarTime > .01): #0.03

        numThreads = 0
        results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        final_goal_pos = [11, 11]
        target_ori = [0, 0]
        carPos, carOrn = p.getBasePositionAndOrientation(car)
        maxForce = 20
        targetVelocity = 0
        steeringAngle = 0
        turn_angle = 0

        for i in range(numRays):
            hitObjectUid = results[i][0]
            hitFraction = results[i][2]
            hitPosition = results[i][3]
            if (hitFraction == 1.):
                p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
            else:
                localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                              rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                              rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
                p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor, replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car, parentLinkIndex=hokuyo_joint)
        lastLidarTime = nowLidarTime
        test_angle = navigate_mobile(carPos,carOrn,results,final_goal_pos)
        drive_the_mobile(carPos,carOrn,test_angle,final_goal_pos)

        if (useRealTimeSim == 0):
            frame += 1
            p.stepSimulation()
        lastControlTime = nowControlTime





