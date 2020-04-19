classdef rawSensorsData < handle
    
    %{ Here  measurements are raw.  without translation  ! }%
    properties 
%         velocity_up_right;
        WheelSpeedFrontLeft;
        WheelSpeedRearRight;
        WheelSpeedRearLeft;
        accelerate_forward;
        accelerate_side;
        steering_angle;  %angel between cars' wheels and main_frame's axis as sensors said,
        throttle;
        measured_theta;
        
        time;
        
        GPS_longtitude;
        GPS_latitude;
    end
    
end%classdef