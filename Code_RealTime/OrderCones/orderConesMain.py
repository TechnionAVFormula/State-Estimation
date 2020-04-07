from .orderByDistance import OrderByDis
from .OrderByDeluanay import orderByDeluanay




def orderCones(coneMap, carState  ):
	
	
	#to order cones by distance
	'''
    orderClass = OrderByDis(coneMap , carState)
    bluePoints , yellowPoints = orderClass.orderByDis()
	'''
	
	#to order cones by delaunay
	bluePoints , yellowPoints, blueLost, yellowLost = orderByDeluanay(coneMap, carState)

    return bluePoints , yellowPoints
    

