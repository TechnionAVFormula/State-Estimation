// messages.sensors.IMUSensors
message IMUSensors
{
    Vector3D linear_accelerion = 1
    Vector3D linear_velocity = 2
    Vector3D rotational_velocity = 3 
    double steering_angle = 4   // radians. right positive
    double wheel_velocity_rear_left = 5
    double wheel_velocity_rear_right = 6
    double wheel_velocity_front_left = 7
    double wheel_velocity_front_right = 8
    double throttle_position = 9
    double orientation = 10
}

// messages.sensors.GroundTruth 
message GroundTruth
{
    Vector3d                        Position = 1
    messages.sensors.IMUSensors     IMU = 2
    messages.perception.ConeMap     ConeMap = 3
}