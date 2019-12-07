classdef carState < handle
    
    properties 
        x_pos;
        y_pos;
        velocity;
        theta;

        
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

            
             obj.numValues = 4; 
        end%creation function
        
        function stateVector  =  vector(obj)
            stateVector = zeros(obj.numValues , 1);
            stateVector(1) = obj.x_pos;
            stateVector(2) = obj.y_pos;
            stateVector(3) = obj.velocity;
            stateVector(4) = obj.theta;
        end
        
        function disp(obj)
            disp(join(["car state:"]));
            disp(join(["x=",obj.x_pos]));
            disp(join(["y=",obj.y_pos]));
            disp(join(["v=",obj.velocity]));
            disp(join(["theta=",obj.theta]));
        end
        
    end%methods
    
end%classdef