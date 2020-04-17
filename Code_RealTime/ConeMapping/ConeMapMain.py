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
        self.filter_freq = 2
        self.real_cone_threshold = 50


        # dynamic propertise:
        self._cone_map = np.array( []   ,   dtype=Cone)
        self._call_counter = 0


    def insert_new_points(  self  , cone_array   ):
        self._call_counter = self._call_counter + 1 
        for cone in cone_array:
            self._cone_map = np.append( self._cone_map , cone )

        if ( self._call_counter >=  self.filter_freq  ):
            print(f"ConeMap::Filtering cones.  _call_counter={self._call_counter}")
            self._call_counter = 0
            
            yellow_cones , blue_cones , orange_cones = self.prepare_cones4cluster()
            yellow_clust  = ClusteringOptics(yellow_cones)
            blue_clust    = ClusteringOptics(blue_cones)
            orange_clust  = ClusteringOptics(orange_cones)

            #do svm here

    def prepare_cones4cluster(self):
        yellow_cones = np.array([])
        blue_cones   = np.array([])
        orange_cones = np.array([])
        for cone in self._cone_map:
            element = [cone.x , cone.y]
            if cone.type==YELLOW :
                yellow_cones=np.append(yellow_cones, element)
            if cone.type==BLUE :
                blue_cones=np.append(blue_cones, element)
            if cone.type==ORANGE :
                orange_cones=np.append(orange_cones, element)
        return yellow_cones, blue_cones, orange_cones 


        



    def get_all_cones( self ):
        return self._cone_map
        '''
        all_cones = np.array( [] , dtype=Cone)
        for cone in self._cone_map:
            temp_element = np.array( cone   ,   dtype=Cone)
            all_cones = np.append( real_cones , temp_element )             
        return all_cones
        '''

    def get_real_cones( self ):
        real_cones = np.array( [] , dtype=Cone)
        for cone in self._cone_map:
            if cone.value > self.real_cone_threshold:
                temp_element = np.array( cone   ,   dtype=Cone)
                real_cones = np.append( real_cones , temp_element )
        return real_cones


    
