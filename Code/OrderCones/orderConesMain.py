from .orderByDistance import OrderByDis
from Code import StateMain

def orderCones(cones, carState  ):
    print("I'm inside orderCones()::")
    
    orderClass = OrderByDis(cones , carState)
    bluePoints , yellowPoints = orderClass.orderByDis()

    return bluePoints , yellowPoints
    

