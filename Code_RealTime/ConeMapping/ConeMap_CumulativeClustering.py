import numpy as np
import math


## for relative path
import os
import sys
from pathlib import Path
current_path  = os.path.dirname(__file__)
current_path = Path(current_path)
relative_path = current_path.parent
sys.path.append(str(relative_path))

from .ClusteringOptics import ClusteringOptics
from .ConeMap_Base import ConeMap_Base


## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
    # from pyFormulaClient.messages.state_est import StateCone
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
    # from pyFormulaClientNoNvidia.messages.state_est import StateCone
else:
    raise NameError('User Should Choose Configuration from config.py')


# Get proper Enum:
YELLOW      = messages.perception.Yellow
BLUE        = messages.perception.Blue
ORANGE_BIG  = messages.perception.OrangeBig
ORANGE_SMALL= messages.perception.Orange


class ConeMap_CumulativeClustering(ConeMap_Base):
    def __init__(self):
        # Hypre Params:
        self.filter_freq = 10
        self.real_cone_threshold = 4


        # dynamic propertise:
        self._cone_samples = np.array( [] ) #accumaltive ssamples before clusstering
        self._real_cones   = np.array( [] ) # real cones after clusttering
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

            if cone.type==ORANGE_BIG:
                if len(orange_cones) == 0:
                   orange_cones=np.append(orange_cones, element)
                else:
                    orange_cones=np.vstack((orange_cones, element))

            '''!!! Add ORANGE_SMALL !!!'''

        return yellow_cones, blue_cones, orange_cones


    ## Also in base-class
    def get_all_samples( self ):
        return self._cone_samples

    def get_real_cones( self ):
        return self._real_cones
