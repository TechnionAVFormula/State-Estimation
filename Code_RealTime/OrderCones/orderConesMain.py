from .orderByDistance import OrderByDis




def orderCones(coneMap, carState  ):
    print("I'm inside orderConesMain()::")
    
    orderClass = OrderByDis(coneMap , carState)
    bluePoints , yellowPoints = orderClass.orderByDis()

    return bluePoints , yellowPoints
    

