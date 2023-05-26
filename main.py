import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot

tableId = p.loadURDF("./table/table.urdf", [1.1, 0.000000, -0.3],
                     p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)

######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
# objectId = p.loadURDF("random_urdfs/001/001.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

xpos = 0.95
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, 0, ang])

object_path = "random_urdfs/094/094.urdf"
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])

######################################################### Load tray Here!!!!#############################################################################

tray_x = 1.1915786458147377
tray_y = 0.292952885700891

trayId = p.loadURDF("./tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])

###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]

# hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
# handReading = [0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252,
# 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976,
# 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])


##################################################################### Input Value Here ##########################################################################################################
#thumb
handInitial = [0.8741202136793738, 0.1099997195304071, 1.5543423825848732, 0.7770214088708354, 0.17044062815241814, 0.6533448024488079, 0.9313089565393872, 0.23927868926000787, 1.3394692474535013, 0.9293021655331047, 0.24381803030871918, 0.17000655833880468, 0.8035977108942631, 1.0247053342965473, 0.17001618555899642, 0.8035604210362718, 1.0267641083544845, 0.16999999999999996, 0.884236408293297, 0.9839238935094305, 0.16999949322169697, 0.8835529868261729, 0.9815225475375963, 0.16999978618716835, 1.569289888343933, 0.4197256660498646, 0.41936662681521286]
palmPosition = [0.8970631221309304, -0.1024960192348226, -0.025263196267187584]
orientation =  [0.92892775535583496, 2.827588395276342, 1.8190245628356934]
#final palm
final_palmPosition = [1.1915786458147377, 0.312952885700891, 0.1]
final_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

##################################################################################################################################################################################################
initial_palmPosition = [0.85, -0.05, 0.2]
initial_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

palmPosition = [palmPosition[0]-0.035,palmPosition[1]+0.05,palmPosition[2]]
##################################################################################################################################################################################################
wristpos = [handInitial[24], handInitial[25], handInitial[18], handInitial[19], handInitial[12], handInitial[13], handInitial[6], handInitial[7], handInitial[0], handInitial[1]]
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(cameraDistance=0.9, cameraYaw=70, cameraPitch=-120, cameraTargetPosition=[0.7, 0.3, 0.2])
k = 540
for i in range(k * 5):
    p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING)
    if i < k:
        palmP([initial_palmPosition[0], initial_palmPosition[1], initial_palmPosition[2]], p.getQuaternionFromEuler([initial_orientation[0], initial_orientation[1], initial_orientation[2]]))
    elif k < i < k * 2:
        palmP([palmPosition[0], palmPosition[1], palmPosition[2]], p.getQuaternionFromEuler([orientation[0], orientation[1], orientation[2]]))
    elif k* 2 < i < k * 3:
        thumb(wristpos[0], wristpos[1])
        indexF(wristpos[2], wristpos[3])
        midF(wristpos[4], wristpos[5])
        ringF(wristpos[6], wristpos[7])
        pinkyF(wristpos[8], wristpos[9])
    elif k*3 < i <k*4:
        palmP([final_palmPosition[0], final_palmPosition[1], final_palmPosition[2]],
              p.getQuaternionFromEuler([final_orientation[0], final_orientation[1], final_orientation[2]]))
    elif i > k*4:
        thumb(wristpos[1], wristpos[0])
        indexF(wristpos[3], wristpos[2])
        midF(wristpos[5], wristpos[4])
        ringF(wristpos[7], 0)
        pinkyF(wristpos[9], 0)
    p.stepSimulation()
print("disconnected")







