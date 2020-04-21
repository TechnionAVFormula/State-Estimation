import math

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
		bluePoints = []
		yellowPoints = []
		otherPoints = []
		for cone in self.cones:
			if cone.type == BLUE:
				bluePoints.append(cone)
			elif cone.type == YELLOW:
				yellowPoints.append(cone)
			else:
				otherPoints.append(cone)
			
		bluePoints.sort(key=self.takeDistance)
		yellowPoints.sort(key=self.takeDistance)
		
		return bluePoints , yellowPoints

