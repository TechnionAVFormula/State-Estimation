classdef sensorsData < handle
    
    %{ Here  measurements are raw.  without translation  ! }%
    properties 
%         velocity_up_right;
        WheelSpeedFrontLeft;
        WheelSpeedRearRight;
        WheelSpeedRearLeft;
        accelerate_forward;
        accelerate_side;
        steering_angle;  %angel between cars' wheels and main_frame's axis
        throttle;
        measured_theta;
        
        x_pos;
        y_pos;
    end
    
    
    %{ After calling methods,  properties must be translated:  }%
    methods
        
%         function measuredState =    measuredState(obj)
%             measuredState = carState() ;
%             measuredState.x_pos = obj.x_pos;
%             measuredState.y_pos = obj.y_pos;
%             measuredState.velocity = ( obj.velocity_down_left + obj.velocity_down_right + obj.velocity_up_leftt + obj.velocity_up_right ) /4; 
%             measuredState.theta= obj.measured_theta;
%         end
        
         function MeanVelocity =    m_velocity(obj)
           velocityFrontLeft = translate 
           MeanVelocity =  (obj.WheelSpeedFrontLeft + obj.WheelSpeedRearLeft+ obj.WheelSpeedRearRight) / 3;
        end
  
    end%methods
    
end%classdef