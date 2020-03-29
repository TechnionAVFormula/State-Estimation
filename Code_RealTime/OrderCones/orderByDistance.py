import math
from pyFormulaClientNoNvidia import messages
import sys
# sys.path.append("..")
# from class_defs.StateEst_CarState import CarState

"""
Function takes orders cones by distance to car
"""

# Get proper Enum:
YELLOW = messages.perception.Yellow
BLUE = messages.perception.Blue
ORANGE = messages.perception.Orange


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

