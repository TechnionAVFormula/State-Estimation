import numpy as np
import math

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))

# Clusttering method
from .ClusteringOptics import ClusteringOptics
# base class:
from .ConeMap_Base import ConeMap_Base


## classes and enums from our utilities:
from StateEst_Utils.config import CONFIG, IS_DEBUG_MODE 
from StateEst_Utils.ConeType import ConeType


# Get proper Enum:
YELLOW 		 = ConeType.YELLOW.value #messages.perception.Yellow
BLUE 		 = ConeType.BLUE.value #messages.perception.Blue
ORANGE_BIG   = ConeType.ORANGE_BIG.value #messages.perception.OrangeBig
ORANGE_SMALL = ConeType.ORANGE_SMALL.value 
UNKNOWN      = ConeType.UNKNOWN.value



class ConeMap_CumulativeClustering(ConeMap_Base):
    def __init__(self):
        # Hypre Params:
        self.filter_freq = 10
        self.real_cone_threshold = 4


        # dynamic propertise:
        self._cone_samples = np.array( [] ) #accumaltive ssamples before clusstering
        self._real_cones   = np.array( [] ) # real cones after clusttering
        self._call_counter = 0


    '''Sampling new cones'''
    def insert_new_points(  self  , cone_array   ):
        
        self._call_counter = self._call_counter + 1
        self._cone_samples = np.append( self._cone_samples , cone_array )


        # Not every insert causes filttering.
        # we need to get a certein amount of samples - denoted by self.filter_freq:
        if ( self._call_counter >=  self.filter_freq  ):
            print(f"ConeMap::Filtering cones.  _call_counter={self._call_counter}")
            self._call_counter = 0 

            yellow_cones , blue_cones , orange_cones = self.__prepare_cones4clusttering()
            blue_clust    = ClusteringOptics(blue_cones  ,self.real_cone_threshold)
            yellow_clust  = ClusteringOptics(yellow_cones,self.real_cone_threshold)
            orange_clust  = ClusteringOptics(orange_cones,self.real_cone_threshold)

            #do svm here

    def __prepare_cones4clusttering(self):
        yellow_cones = np.array([])
        blue_cones   = np.array([])
        orange_cones = np.array([])
        for cone in self._cone_samples:
            element = [cone.position.x , cone.position.y]

            if cone.type == YELLOW :
                if len(yellow_cones) == 0:
                   yellow_cones=np.append(yellow_cones, element)
                else:
                    yellow_cones=np.vstack((yellow_cones, element))

            if cone.type == BLUE :
                if len(blue_cones) == 0:
                   blue_cones=np.append(blue_cones, element)
                else:
                    blue_cones=np.vstack((blue_cones, element))

            if cone.type == ORANGE_BIG:
                if len(orange_cones) == 0:
                   orange_cones=np.append(orange_cones, element)
                else:
                    orange_cones=np.vstack((orange_cones, element))

            if cone.type == UNKNOWN:
                print(f"StateEstimation:ConeMap_CumulativeClustering: Got an unkown cone type")
                
            '''!!! Add ORANGE_SMALL !!!'''

        return yellow_cones, blue_cones, orange_cones


    ## Also in base-class
    def get_all_samples( self ):
        return self._cone_samples

    def get_real_cones( self ):
        return self._real_cones
