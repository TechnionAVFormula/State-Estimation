classdef carState < handle
    
    properties 
        x_pos;
        y_pos;
        velocity;
        theta;
        psi;
        
        %{ Remember to Update:  }%
        numValues;
    end
    
    methods

        function obj = carState()
            %c'tor:
            obj.x_pos = 0;
            obj.y_pos = 0;
            obj.velocity = 0;
            obj.theta = 0;
            obj.psi = 0;
            
             obj.numValues = 5; 
        end%creation function
        
        function stateVector  =  vector(obj)
            stateVector = zeros(obj.numValues , 1);
            stateVector(1) = obj.x_pos;
            stateVector(2) = obj.y_pos;
            stateVector(3) = obj.velocity;
            stateVector(4) = obj.theta;
            stateVector(5) = obj.psi;
        end
       
        
    end%methods
    
end%classdef