classdef sensorsData < handle
    
    properties 
        velocity_up_right;
        velocity_down_right;
        velocity_up_leftt;
        velocity_down_left;
        accelerate_forward;
        accelerate_side;
        steering_angle;  %angel between cars' wheels and main_frame's axis
        throttle;
        measured_theta;
        
        x_pos;
        y_pos;
    end
    
    methods
        
        function measuredState =    measuredState(obj)
            measuredState = carState() ;
            measuredState.x_pos = obj.x_pos;
            measuredState.y_pos = obj.y_pos;
            measuredState.velocity = ( obj.velocity_down_left + obj.velocity_down_right + obj.velocity_up_leftt + obj.velocity_up_right ) /4; 
            measuredState.theta= obj.measured_theta;
        end
       function MeanVelocity =    m_velocity(obj)
           MeanVelocity =  (velocity_down_right+velocity_up_leftt+velocity_down_left)/3;

        end
        
  
    end%methods
    
end%classdef