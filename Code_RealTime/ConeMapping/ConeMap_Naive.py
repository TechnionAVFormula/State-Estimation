import numpy as np
import math

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

## classes and enums from our utilities:
from StateEst_Utils.config import COMULATIVE_CONE_MAP
from StateEst_Utils.ConeType import ConeType

# base class:
from .ConeMap_Base import ConeMap_Base


class ConeMap_Naive(ConeMap_Base):
    def __init__(self):
        super().__init__()
        self._epsilon = 2 # meters of error around cone for a new cone
        self._got_cones = False

    def insert_new_points(  self  , cone_array   ):
        ## Avoid adding more cones if that's what the user chose.
        if (not COMULATIVE_CONE_MAP) and (self._got_cones):
            return        
        self._got_cones = True
        ## Add the cones:
        for cone in cone_array:
            if not self._cone_exist(cone):
                self._cone_samples = np.append( self._cone_samples , cone )


    def _cone_exist(self, new_cone):
        epsilon_sqrd = self._epsilon**2
        x_new = new_cone.position.x
        y_new = new_cone.position.y

        for old_cone in self._cone_samples:
            x_old = old_cone.position.x
            y_old = old_cone.position.y
            seperation_sqrd =  math.pow(   x_new - x_old  ,2) + math.pow(    y_new - y_old   ,2)
            if seperation_sqrd < epsilon_sqrd:
                return True #exist
        #if we came here, no existing cone match one of the existing cones
        return False #not exist

    ## In base-class
    # def get_all_samples( self ):
    #     return self._cone_samples

    def get_all_samples( self ):
        return self._cone_samples

    def get_real_cones( self ):
        return self.get_all_samples()            

def main():
    cone_map = ConeMap_Base()
    print(cone_map)


if __name__ == "__main__":
    out = main()