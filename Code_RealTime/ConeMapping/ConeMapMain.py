from class_defs.Cone import Cone
import numpy as np
import math


## import depanding on running state / configuration state:
from ..config import CONFIG , ConfigEnum , IS_DEBUG_MODE

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

        if ( self._call_counter %  self.filter_freq  == 0 ):
            pass
            #do svm here


    def get_real_cones( self ):
        real_cones = np.array( []   ,   dtype=Cone)

        for cone in self._cone_map:

            if cone.value > self.real_cone_threshold:
                temp_element = np.array( cone   ,   dtype=Cone)
                real_cones = np.append( real_cones , temp_element )


        return real_cones




    
