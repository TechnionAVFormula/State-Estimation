import numpy as np
import math


## for relative path
import os 
import sys
current_dir_name = os.path.dirname(__file__)
relative_dir_name = os.path.join(current_dir_name, '..')
sys.path.append(relative_dir_name)

from .ClusteringOptics import ClusteringOptics
from class_defs.Cone import Cone

## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')

# Get proper Enum:
YELLOW = messages.perception.Yellow
BLUE = messages.perception.Blue
ORANGE = messages.perception.Orange

class ConeMap():
    def __init__(self):
        # Hypre Params:
        self.filter_freq = 10
        self.real_cone_threshold = 4


        # dynamic propertise:
        self._cone_samples = np.array( []   ,   dtype=Cone) #accumaltive ssamples before clusstering
        self._real_cones   = np.array( []   ,   dtype=Cone) # real cones after clusttering
        self._call_counter = 0


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
            element = [cone.x , cone.y]

            if cone.type==YELLOW :
                if len(yellow_cones) == 0:
                   yellow_cones=np.append(yellow_cones, element) 
                else:
                    yellow_cones=np.vstack((yellow_cones, element))

            if cone.type==BLUE :
                if len(blue_cones) == 0:
                   blue_cones=np.append(blue_cones, element) 
                else:
                    blue_cones=np.vstack((blue_cones, element))

            if cone.type==ORANGE:
                if len(orange_cones) == 0:
                   orange_cones=np.append(orange_cones, element) 
                else:
                    orange_cones=np.vstack((orange_cones, element))  

        return yellow_cones, blue_cones, orange_cones 


    def get_all_samples( self ):
        return self._cone_samples

    def get_real_cones( self ):
        return self._real_cones
    
