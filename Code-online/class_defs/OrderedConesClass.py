import numpy as np
from pyFormulaClientNoNvidia import messages

class OrderedCones:
    yellow_cones = np.array( [] , dtype= messages.state_est.StateCone )
    blue_cones =   np.array( [] , dtype= messages.state_est.StateCone )
    orange_cones = np.array( [] , dtype= messages.state_est.StateCone )