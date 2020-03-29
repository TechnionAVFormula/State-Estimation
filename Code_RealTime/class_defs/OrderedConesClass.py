import numpy as np
from pyFormulaClientNoNvidia import messages
from class_defs.Cone import Cone

class OrderedCones:
    yellow_cones =  np.array([] , dtype=Cone)
    blue_cones =    np.array([] , dtype=Cone)
    orange_cones =  np.array([] , dtype=Cone)