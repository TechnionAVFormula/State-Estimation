import math
import numpy as np

## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')



"""
Function takes orders cones by distance to car
"""

# Get proper Enum:
YELLOW 		= messages.perception.Yellow
BLUE   		= messages.perception.Blue
ORANGE_BIG  = messages.perception.OrangeBig
ORANGE_SMALL= messages.perception.OrangeSmall


class OrderByDis:

	def __init__(self, cones, carState):
		self.carState = carState
		self.cones = cones


	def takeDistance(self, elem):
		return math.sqrt(math.pow(elem.x-self.carState.x,2) + math.pow(elem.y-self.carState.y,2))
	def orderByDis(self):

		nblue = 0
		nyellow = 0
		for cone in self.cones:
			if cone.type == BLUE:
				nblue += 1
			elif cone.type == YELLOW:
				nyellow += 1

		bluePoints = np.empty([nblue, 2], dtype=(object))
		yellowPoints = np.empty([nyellow, 2], dtype=(object))

		nblue = 0
		nyellow = 0
		for cone in self.cones:
			if cone.type == BLUE:
				bluePoints[nblue][0] = cone
				bluePoints[nblue][1] = math.sqrt(math.pow(cone.position.x-self.carState.position.x,2) + math.pow(cone.position.y-self.carState.position.y,2))
				nblue += 1
			elif cone.type == YELLOW:
				yellowPoints[nyellow][0] = cone
				yellowPoints[nyellow][1] = math.sqrt(math.pow(cone.position.x-self.carState.position.x,2) + math.pow(cone.position.y-self.carState.position.y,2))
				nyellow += 1
			
		returnBlue = bluePoints[bluePoints[:,1].argsort()]
		returnYellow = yellowPoints[yellowPoints[:,1].argsort()]
		
		return returnBlue[:,0] , returnYellow[:,0]

