classdef carState < handle
    
    properties 
        x_pos;
        y_pos;
        velocity;
        theta;
        
        
        %{ Remember to Update:  }%
        num_values = 4; 
    end
    
    methods

        function obj = carState()
            %c'tor:
            obj.x_pos = 0;
            obj.y_pos = 0;
            obj.velocity = 0;
            obj.theta = 0;
        end%creation function
        
        function stateVector  =  vector(obj)
            stateVector = zeros(1,4);
            stateVector(1) = obj.x_pos;
            stateVector(2) = obj.y_pos;
            stateVector(3) = obj.velocity;
            stateVector(4) = obj.theta;
        end
       
        
    end%methods
    
end%classdef