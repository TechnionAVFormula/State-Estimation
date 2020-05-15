from .MessagesClass import messages
from enum import Enum

class ConeType(Enum):
    UNKNOWN     = messages.perception.UnknownType  
    YELLOW      = messages.perception.Yellow
    BLUE        = messages.perception.Blue
    ORANGE_BIG  = messages.perception.OrangeBig
    ORANGE_SMALL= messages.perception.Orange
