import numpy as np
import math


## for relative path
import os 
import sys
current_dir_name = os.path.dirname(__file__)
relative_dir_name = os.path.join(current_dir_name, '..')
sys.path.append(relative_dir_name)


from class_defs.Cone import Cone

## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
else:
    raise NameError('User Should Choose Configuration from config.py')


class ConeMap():
    def __init__(self):
        # Hypre Params:
        self.filter_freq = 100
        self.real_cone_threshold = 50


        # dynamic propertise:
        self._cone_map = np.array( []   ,   dtype=Cone)
        self._call_counter = 0


    def insert_new_points(  self  , cone_array   ):
        self._call_counter = self._call_counter + 1 

        if ( self._call_counter >=  self.filter_freq  ):
            print(f"ConeMap::Filtering cones.  _call_counter={self._call_counter}")
            self._call_counter = 0
            pass
            #do svm here


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


    
