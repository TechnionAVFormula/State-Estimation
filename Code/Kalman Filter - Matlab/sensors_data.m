classdef sensorsData < handle
    
    properties 
        throttle;       %value from -1 to 1;
        steering_angle;  % [deg]
    end
    
    methods 
        function obj = sensorsData(throttle, steering_angle)
          obj.throttle = throttle;
          obj.steering_angle = steering_angle;
        end
    end
    
end%class
