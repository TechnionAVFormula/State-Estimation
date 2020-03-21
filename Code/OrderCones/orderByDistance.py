import math

"""
Function takes orders cones by distance to car
"""

YELLOW = 0
BLUE = 1
ORANGE = 2

def takeDistance(elem):
	return elem.r

def orderByDis(cones, carPos, carDir):

	print("I'm inside orderByDis::")
	return

	bluePoints = []
	yellowPoints = []
	for cone in cones:
		if cone.type == BLUE:
			bluePoints.append(cone)
		elif cone.type == YELLOW:
			yellowPoints.append(cone)
		
	bluePoints.sort(key=takeDistance)
	yellowPoints.sort(key=takeDistance)

	count = 0
	for point in bluePoints:
		point.order = count
		count += 1
	count = 0
	for point in yellowPoints:
		point.order = count
		count += 1
		
	newCones = []
	newCones.extend(yellowPoints)
	newCones.extend(bluePoints)
		
	return newCones