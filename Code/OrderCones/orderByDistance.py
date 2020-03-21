import math

"""
Function takes orders cones by distance to car
"""

BLUE = 0
YELLOW = 1
TYPE = 2

def takeDistance(elem):
	return elem.order

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
	
	for point in bluePoints:
		point.order = math.sqrt((carPos.x - point.x)**2 + (carPos.y - point.y)**2)
	for point in yellowPoints:
		point.order = math.sqrt((carPos.x - point.x)**2 + (carPos.y - point.y)**2)
		
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