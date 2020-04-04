import numpy as np
from config import CONFIG, ConfigEnum
from class_defs.Cone import Cone

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages

class OrderedCones:
    yellow_cones =  np.array([] , dtype=Cone)
    blue_cones =    np.array([] , dtype=Cone)
    orange_cones =  np.array([] , dtype=Cone)