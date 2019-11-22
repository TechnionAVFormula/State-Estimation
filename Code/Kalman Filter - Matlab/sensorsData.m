classdef sensorsData < handle
    
    properties 
        velocity_up_right;
        velocity_down_right;
        velocity_up_leftt;
        velocity_down_left;
        accelerate_forward;
        accelerate_side;
        psi;  %angel between cars' wheels and main_frame's axis
        throttle;
        measured_theta;
        
        
        x_pos;
        y_pos;
    end
    
    methods
        function measured_state_vector = measured_state(obj)%z
            measured_state_vector = zeros(1,2)  ;
            measured_state_vector(1) = obj.x_pos;
            measured_state_vector(2) = obj.y_pos;
            measured_state_vector(3) = ( obj.velocity_down_left + obj.velocity_down_right + obj.velocity_up_leftt + obj.velocity_up_right ) /4; 
            measured_state_vector(4) = obj.measured_theta;
        end
    end%methods
    
end%classdef