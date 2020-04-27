import plotly.graph_objects as pgo
import plotly
import codecs

## for relative import:
import os 
import sys
from pathlib import Path
current_dir = os.path.dirname(__file__)
relative_dir = Path(current_dir)
relative_dir = relative_dir.parent
sys.path.append(str(relative_dir))    #add to sys search path


## import depanding on running state / configuration state:
from config import CONFIG , ConfigEnum , IS_DEBUG_MODE

if (CONFIG  == ConfigEnum.REAL_TIME) or (CONFIG == ConfigEnum.COGNATA_SIMULATION):
    from pyFormulaClient import messages
    from pyFormulaClient.MessageDeque import NoFormulaMessages
elif ( CONFIG == ConfigEnum.LOCAL_TEST):
    from pyFormulaClientNoNvidia import messages
    from pyFormulaClientNoNvidia.MessageDeque import NoFormulaMessages
else:
    raise NameError('User Should Choose Configuration from config.py')



def get_update(path_str):
    with open(path_str) as f:
        data = json.load(f)
    plotly_state(data)


def set_fig_appearance(fig):
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


def plotly_state(data):

    

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
        return

    x_arr = x_arr_yellow + x_arr_blue
    y_arr = y_arr_yellow + y_arr_blue
    c_arr = color_arr_yellow + color_arr_blue


    
    ## Plot:
    scatter = pgo.Scatter( x=x_arr , y=y_arr ,  mode='markers' ,
        marker=dict( color=c_arr , size=20)
    )
    # color='rgb(140,140,0)'



    fig = pgo.Figure(data=scatter)
    fig = set_fig_appearance(fig)

    if IS_FIRST_CALL:
        fig.write_html('State Estimation Dash-Board.html' , auto_open=True)
        IS_FIRST_CALL = False
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
  