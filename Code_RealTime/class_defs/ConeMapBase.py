import numpy as np

class ConeMapBase:

    def __init__(self):
        self._cone_samples = np.array( [] ) #accumaltive ssamples before clusstering

    def insert_new_points(  self  , cone_array   ):
        self._cone_samples = np.append( self._cone_samples , cone_array )

    def get_all_samples( self ):
        return self._cone_samples
