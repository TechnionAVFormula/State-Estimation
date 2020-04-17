from config import CONFIG, ConfigEnum

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages

class Cone:
    cone_id = -1
    #relative to vehicle:
    r       = -1.1
    theta   = -1.2
    #absolute coordinatees in space:
    x       = -1.3
    y       = -1.4
    #type of cone:
    type    = messages.perception.Blue 
