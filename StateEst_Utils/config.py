from .ConfigEnum import ConfigEnum

## Choose method of running the whole State Estimation Module
# CONFIG = ConfigEnum.COGNATA_SIMULATION
CONFIG = ConfigEnum.LOCAL_TEST 
# CONFIG = ConfigEnum.REAL_TIME

## Choose message input and output
# IN_MESSAGE_FILE = 'Messages/input_test.messages'     #hand-made messages
IN_MESSAGE_FILE = 'Messages/fromSimulation.messages'   #messages from "driveSim" - Matlab simulation
OUT_MESSAGE_FILE = 'Messages/state.messages'


## Choose if to print in debug mode:
IS_DEBUG_MODE = True
IS_TIME_CODE_WITH_TIMER = False

# Cone map:
IS_CONE_MAP_WITH_CLUSTERING = False
COMULATIVE_CONE_MAP = True    # False - will shut dwon the option to sample more cones. Only the first cone message will count

## Real time visualizations:
SHOW_REALTIME_DASHBOARD = False
