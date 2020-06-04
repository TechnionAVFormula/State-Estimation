import plotly
import plotly.graph_objects as pgo
import plotly.figure_factory as pff
import codecs

import math

## for relative Imports
import sys, os, pathlib
currentPath = pathlib.Path(os.path.dirname(__file__))
relativePath = currentPath.parent.parent
sys.path.append(str(relativePath))


## classes and enums from our utilities:
from StateEst_Utils.config import CONFIG, IS_DEBUG_MODE 
from StateEst_Utils.ConeType import ConeType
from StateEst_Utils.MessagesClass import messages

# Get proper Enum:
YELLOW 		 = ConeType.YELLOW #messages.perception.Yellow
BLUE 		 = ConeType.BLUE #messages.perception.Blue
ORANGE_BIG   = ConeType.ORANGE_BIG #messages.perception.OrangeBig
ORANGE_SMALL = ConeType.ORANGE_SMALL 
UNKNOWN      = ConeType.UNKNOWN


is_first_call = True
IS_Xnorth_Yeast = True


def send_StateEst_DashBoard_with_GroundTruth(msg ,CarTruth): 
    data = messages.state_est.FormulaState()
    msg.data.Unpack(data)
    time_in_milisec = msg.header.timestamp.ToMilliseconds()
    
    plotly_state(data , time=time_in_milisec , CarTruth=CarTruth)


def send_StateEst_DashBoard_msg(msg):
    data = messages.state_est.FormulaState()
    msg.data.Unpack(data)
    time_in_milisec = msg.header.timestamp.ToMilliseconds()

    plotly_state(data , time=time_in_milisec)


def get_update(path_str):
    with open(path_str) as f:
        data = json.load(f)
    plotly_state(data)


def set_fig_appearance(fig):
    if IS_Xnorth_Yeast:
        fig.update_layout(
        title='State Estimation Dash-Board',
        xaxis_title='yEast [m]',
        yaxis_title='xNorth [m]'
        )
    else:
        fig.update_layout(
        title='State Estimation Dash-Board',
        xaxis_title='xNorth [m]',
        yaxis_title='yEast [m]'
        )
    return fig

def cones_to_x_y_arrays(cone_array):
    x_array = []
    y_array = []
    for cone in cone_array:
        x_array.append(cone.position.x)
        y_array.append(cone.position.y)
    return x_array , y_array


def plotly_state(data , time=0 , **kargs):


    if 'CarTruth' in kargs:
        is_with_car_truth = True
        CarTruth = kargs['CarTruth']
        ## Parse GroundTruth"
        true_x = CarTruth["x"]
        true_y = CarTruth["y"]
        true_theta = CarTruth["theta"]
    else:
        is_with_car_truth = False


    ## Parse Data:
    right_cones = data.right_bound_cones
    left_cones  = data.left_bound_cones
    distance2finish = data.distance_to_finish
    car_x = data.current_state.position.x
    car_y = data.current_state.position.y
    car_theta =  data.current_state.theta
    #error in estimation:
    car_pos_deviation_x = data.current_state.position_deviation.x
    car_pos_deviation_y = data.current_state.position_deviation.y


    x_arr_yellow , y_arr_yellow = cones_to_x_y_arrays(right_cones)
    color_arr_yellow = ['yellow']  * len(x_arr_yellow)
    x_arr_blue , y_arr_blue = cones_to_x_y_arrays(left_cones)
    color_arr_blue = ['blue'] * len(x_arr_blue)

    if len(x_arr_yellow)==0:
        print(f"DashBoard::StateEst::no cones")

    if is_with_car_truth:
        x_arr = x_arr_yellow     + x_arr_blue     + [car_x]  + [true_x]
        y_arr = y_arr_yellow     + y_arr_blue     + [car_y]  + [true_y]
        c_arr = color_arr_yellow + color_arr_blue + ['red']  + ['orange']
    else:
        x_arr = x_arr_yellow      + x_arr_blue     + [car_x] 
        y_arr = y_arr_yellow      + y_arr_blue     + [car_y] 
        c_arr = color_arr_yellow  + color_arr_blue + ['red']  

    ## Show theta as arrow:
    # fig = pff.create_quiver(car_x, car_y , math.cos(car_theta) , math.sin(car_theta)) 

    ## Plot:
    if IS_Xnorth_Yeast:
        scatter = pgo.Scatter( x=y_arr , y=x_arr ,  mode='markers' ,
            marker=dict( color=c_arr , size=20)
        )
    else:
        scatter = pgo.Scatter( x=x_arr , y=y_arr ,  mode='markers' ,
            marker=dict( color=c_arr , size=20)
        )
        # color='rgb(140,140,0)'



    fig = pgo.Figure(data=scatter)
    fig = set_fig_appearance(fig)

    global is_first_call
    if is_first_call:
        fig.write_html('State Estimation Dash-Board.html' , auto_open=True)
        is_first_call = False
    else:
        fig.write_html('State Estimation Dash-Board.html' , auto_open=False)


def plotly_test():
    x = [1 , 3]
    y = [2 , 2]
    scatter = pgo.Scatter(x=x , y=y , mode='markers')
    fig = pgo.Figure(data=scatter)
    fig.write_html('State Estimation Dash-Board.html' , auto_open=True )

    x = [2 , 2]
    y = [3 , 5]
    scatter = pgo.Scatter(x=x , y=y , mode='markers')
    fig = pgo.Figure(data=scatter)
    fig.write_html('State Estimation Dash-Board.html' , auto_open=False )


if __name__ == "__main__":
    plotly_test()
