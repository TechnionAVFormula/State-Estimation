import plotly.graph_objects as pgo

def plotly_test():
    x = [2 , 2]
    y = [3 , 5]
    scatter = pgo.Scatter(x=x , y=y , mode='markers')
    fig = pgo.Figure(data=scatter)
    fig.write_html('first_figure.html', auto_open=True)


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
    fig.write_html('State Estimation Dash-Board.html' , auto_open=True)


if __name__ == "__main__":
    plotly_test()

    