from matplotlib import pyplot as plt
import math
import numpy as np

## for relative path
import os 
import sys
from pathlib import Path
current_dir_name = os.path.dirname(__file__)
current_dir_path = Path(current_dir_name)
relative_dir_path= current_dir_path.parent
sys.path.append(str(relative_dir_path))


## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
    # from pyFormulaClient.messages.state_est import StateCone
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
    # from pyFormulaClientNoNvidia.messages.state_est import StateCone
else:
    raise NameError('User Should Choose Configuration from config.py')

UNKOWN_TYPE = messages.perception.ConeType.UnknownType
YELLOW      = messages.perception.ConeType.Yellow
BLUE        = messages.perception.ConeType.Blue
ORANGE_BIG  = messages.perception.ConeType.OrangeBig 
ORANGE_SMALL= messages.perception.ConeType.OrangeSmall 


IS_GRID_ON = False


class CompPlot():
    def __init__(self):
        self.fig , self.ax = plt.subplots()
        self._intialize_figure()
        self._refresh_timer = 0.00001
        self._trace_epsilon = 3
        self._car_icon = None
        self._car_arrow = None
        self._car_trace = np.array([])

        self.refresh()
        

    def refresh(self):
        plt.pause(self._refresh_timer)    

    def _intialize_figure(self):
        self.ax.set_xlabel('xNorth')
        self.ax.set_ylabel('yEast')
        self.ax.set_title('State Estimation Map')
        self.ax.set_xlim(-50 , 50)
        self.ax.set_ylim(-50 , 50)
        if IS_GRID_ON:
            self.ax.grid()

    def plot_cones(self , cone_array):
        max_x = -math.inf
        min_x = math.inf
        max_y = -math.inf
        min_y = math.inf

        for cone in cone_array:
            x = cone["x"]
            y = cone["y"]
            cone_type = cone["type"]
            if cone_type == BLUE:
                plt.scatter(x,y , c='blue' )
            if cone_type == YELLOW:
                plt.scatter(x,y , c='gold' )

            #update limits:
            if x > max_x:
                max_x = x
            if x < min_x:
                min_x = x
            if y > max_y:
                max_y = y
            if y < min_y:
                min_y = y
        
        delta = 10
        self.ax.set_xlim(min_x - delta , max_x + delta)
        self.ax.set_ylim(min_y - delta , max_y + delta)
        self.refresh()
    

    def check_exist_in_trace(self , x_new, y_new):
        clear_radius_sqrd = self._trace_epsilon**2
        for position in self._car_trace:
            x_exist = position["x"]
            y_exist = position["y"] 
            distance_sqrd = (x_exist - x_new)**2 +  (y_exist - y_new)**2
            if distance_sqrd < clear_radius_sqrd:
                return True 
        return False    

    def update_car_state(self , car_turth):
        x = car_turth["x"]
        y = car_turth["y"]
        theta = car_turth["theta"]
        
        arrow_length = 5
        dx = arrow_length*math.cos(theta)
        dy = arrow_length*math.sin(theta)


        #Plot car trace:
        if not self.check_exist_in_trace(x,y) :
            plt.scatter(x,y , c='black' , alpha=0.3 , marker='.')
            position = {"x": x , "y": y}
            self._car_trace = np.append( self._car_trace , position  )

        #Plot car icon:
        if self._car_icon == None:  #create:
            self._car_icon  = plt.scatter(x,y , c='red')
            self._car_arrow = plt.arrow(x,y,dx,dy , color='black' , width=0.2 )
        else:  #update/replace:
            self._car_icon.set_offsets([x,y])
            self._car_arrow.remove()
            self._car_arrow = plt.arrow(x,y,dx,dy , color='black' , width=0.2)

        self.refresh()