import can
def get_wheel_velocity(wheel):
    wheel_number = (wheel=='FrontRight')+(wheel=='FrontLeft')*2+(wheel=='RearRight')*3+(wheel=='RearLeft')*4
    wheel_request = can.Message(arbitration_id=0xABCDE, data=[wheel_number])
    ......
    return wheel.data

def get_steering_angle()
    steering_request = can.Message(arbitration_id=0xABCDE, data=[wheel_number])
    .....
    return steering.data

def get_acceleration(axis)
    axis_number = (axis == 'x')+(axis == 'y')*2+(axis == 'z')*3
    axis_request = can.Message(arbitration_id=0xABCDE, data=[wheel_number])
    ....
    return axis

def send_throttle(thorttlegain)
    thorttle = can.Message(arbitration_id=0xABCDE, data=[thorttlegain])
    .....
    return 


##send velocity motec