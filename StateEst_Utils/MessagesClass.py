
from .ConfigEnum import ConfigEnum
from .config import CONFIG 


if (CONFIG == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
    from pyFormulaClient.MessageDeque import NoFormulaMessages
elif CONFIG == ConfigEnum.LOCAL_TEST:
    from pyFormulaClientNoNvidia import messages
    from pyFormulaClientNoNvidia.MessageDeque import NoFormulaMessages
else:
    raise NameError("User Should Choose Configuration from config.py")


messages = messages
NoFormulaMessages = NoFormulaMessages