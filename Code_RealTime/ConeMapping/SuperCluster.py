import math

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

## classes and enums from our utilities:
from StateEst_Utils.config import CONFIG, IS_DEBUG_MODE  #?
from StateEst_Utils.ConeType import ConeType

MISS_DETECTION_THRESHOLD = 5 #max itrations the super cluster lives without detection
CERTAINTY_DETECTION_THRESHOLD  = -5 # num itrations the super cluster lives

class SuperCluster():

    availableID = 1

    def __init__(self,Type,Weight,x,y,x_deviation,y_deviation):
        self.x = x
        self.y = y
        self.x_deviation = x_deviation
        self.y_deviation = y_deviation
        self.Weight = Weight
        self.Type = Type # color
        self._age = 0 #number of iterations the super cluster live
        self._is_eternal = False
        # Keep track on this cone's ID:
        self.id  = SuperCluster.availableID
        SuperCluster.increment_availableID()

    @staticmethod
    def increment_availableID():
        SuperCluster.availableID = SuperCluster.availableID + 1


    
    def combine(self, other): 
        """combine Combined existing cluster with a new one.

        The one that has been called "combine" lives and the other dies

        Args:
            other ([type]): [description]
        """
        if self.getType() != self.getType(): # We shouldn't get here. the Types shouldn't be the same
            raise Exception("combine")

        SumWeights = other.Weight+self.Weight
        # Weigthed avarage. The bigger cluster has more weight on decodong the new location:
        self.x=(other.Weight*other.x+self.Weight*self.x)/SumWeights
        self.y=(other.Weight*other.y+self.Weight*self.y)/SumWeights
        self.Weight=SumWeights
        SumAges = other._age+self._age    
        self._age=SumAges

    def markSeen(self):
        self._becomeYounger()

    def markNotSeen(self):
        if self._is_eternal:
            pass    
        else:
            self._becomeOlder()

    def _becomeOlder(self): #if we didnt apply combine 
        self._age=self._age+1


    def _becomeYounger(self): #if we apply combine
        self._age=self._age-1 
        if self._age <= CERTAINTY_DETECTION_THRESHOLD:
            self._is_eternal = True
    
    def checkDead(self):
        if self._age >= MISS_DETECTION_THRESHOLD: 
            return True
        else:
            return False

    def getX(self):
        return self.x  
    def getY(self):        
        return self.y
    def getXDeviation(self):
        return self.x_deviation
    def getYDeviation(self):
        return self.y_deviation
    def getWeight(self):
        return self.Weight
    def getType(self):
        return self.Type
    def getAge(self):
        return self._age
    def getId(self):
        return self.id    
    
if __name__ == "__main__":
    a = SuperCluster(ConeType.YELLOW, 10, 3, 4)
    b = SuperCluster(ConeType.YELLOW, 20, 3, 5)
    a.combine(b)
    print(a)
    