#add parent dir to find package. Only needed for source code build, pip install doesn't need it.
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)
import pybullet as p
import math

#from pybullet_envs.bullet.kukaGymEnv import KukaGymEnv
from sawyerEnv import sawyerEnv
import time

def main():

  # replace the values of following 3 variables to recall the last configurations
  # the values are the 3 output of the last run of this program
  handInitial =  [0.22758715119487366, 0.9757903273209749, 0.24161750372834237, 0.22710371244591762, 0.976509480725665, 0.18225672671637788, 0.5776262116231128, 0.5856521577290977, 0.21449415241902586, 0.573393395852755, 0.5911037708666577, 0.1700040673710855, 0.9352621390539142, 0.26659978467036205, 0.1700028233065044, 0.9354455405240106, 0.26708167250766834, 0.16999999999999985, 0.9059983743861906, 0.25177778770414255, 0.1699999881540996, 0.9058464398347609, 0.25171720259452235, 0.16999998371672256, 1.5697207347951239, 0.3404695043046988, 0.34043465669512]
  orientation =  [1.2892775535583496, 2.827588395276342, 1.2237756252288818]
  # palmPosition = [1.04,-0.03,0.38]
  palmPosition = [0.95, 0,0.2]

  environment = sawyerEnv(renders=True, isDiscrete=False, maxSteps=10000000, palmPosition = palmPosition, orientation = orientation)
  readings = [0] * 35
  motorsIds = []
 
  # dv = 0.01
  dv = 0.001
  motorsIds.append(environment._p.addUserDebugParameter("posX", -dv, dv, 0))
  motorsIds.append(environment._p.addUserDebugParameter("posY", -dv, dv, 0))
  dv_z_min = -0.01
  dv_z_max = 0.01
  motorsIds.append(environment._p.addUserDebugParameter("posZ", dv_z_min, dv_z_max, 0))

  # orientation of the palm 
  motorsIds.append(environment._p.addUserDebugParameter("orienX", -math.pi, math.pi, 0))
  motorsIds.append(environment._p.addUserDebugParameter("orienY", -math.pi, math.pi, 0))
  motorsIds.append(environment._p.addUserDebugParameter("orienZ", -math.pi, math.pi, 0))

  #low [0.17 - 1.57], mid [0.34, 1.5]
  motorsIds.append(environment._p.addUserDebugParameter("thumbLow", 0.85, 1.57, handInitial[24]))
  motorsIds.append(environment._p.addUserDebugParameter("thumbMid", 0.34, 1.5, handInitial[25]))
  #[0.17 - 1.57]
  motorsIds.append(environment._p.addUserDebugParameter("indexLow", 0.17, 1.57, handInitial[18]))
  motorsIds.append(environment._p.addUserDebugParameter("indexMid", 0.17, 1.57, handInitial[19]))
  motorsIds.append(environment._p.addUserDebugParameter("middleLow", 0.17, 1.57, handInitial[12]))
  motorsIds.append(environment._p.addUserDebugParameter("middleMid", 0.17, 1.57, handInitial[13]))
  motorsIds.append(environment._p.addUserDebugParameter("ringLow", 0.17, 1.57, handInitial[6]))
  motorsIds.append(environment._p.addUserDebugParameter("ringMid", 0.17, 1.57, handInitial[7]))
  motorsIds.append(environment._p.addUserDebugParameter("pinkyLow", 0.17, 1.57, handInitial[0]))
  motorsIds.append(environment._p.addUserDebugParameter("pinkyMid", 0.17, 1.57, handInitial[1]))
  done = False
  action = []
  while (not done):

    action = []
    for motorId in motorsIds:
      action.append(environment._p.readUserDebugParameter(motorId))

    # print (action)
    # break
    #state, reward, done, info = environment.step2(action)
    state, reward, info = environment.step2(action)
    #environment.step2(action)
    #done = True
    #obs = environment.getExtendedObservation()
    handReading = environment.handReading()
    orientation1 = environment.o()
    palmPosition1 = environment.p()
    palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact = environment.info()
    qKey = ord('q')
    keys = p.getKeyboardEvents()
    if qKey in keys and keys[qKey]&p.KEY_WAS_TRIGGERED:

      break;

  print("==============================================================================") 
  print("handReading = ", handReading) 
  print("==============================================================================") 
  print("orientation = ", orientation1) 
  print("==============================================================================") 
  print("palmPosition = ", palmPosition1) 
  print("==============================================================================") 
  print("==============================================================================")
  print("==============================================================================") 
  print("==============================================================================")  
  print("PalmCOntact:")
  for x in palmContact:
    print(x)
  print("==============================================================================") 
  print("thumbContact:")
  for x in thumbContact:
    print(x)
  print("==============================================================================") 
  print("indexContact:")
  for x in indexContact:
    print(x)
  print("==============================================================================") 
  print("midContact:")
  for x in midContact:
    print(x)
  print("==============================================================================") 
  print("ringContact:")
  for x in ringContact:
    print(x)
  print("==============================================================================") 
  print("pinkyContact:")
  for x in pinkyContact:
    print(x)

	


if __name__ == "__main__":
  main()


