



message measureables
{
    Vector3 linear_accelerion = 1
    Vector3 linear_velocity = 10
    Vector3 rotational_velocity = 2 
    double steering_angle = 3   // radians. right positive
    double wheel_velocity_rear_left = 4
    double wheel_velocity_rear_right = 5
    double wheel_velocity_front_left = 6
    double wheel_velocity_front_right = 7
    double throttle_position = 8
    /*  */
    double orientation = 9
}


message IMU
{
    measureables Measurments = 1
    measureables Ground_Truth = 2
}