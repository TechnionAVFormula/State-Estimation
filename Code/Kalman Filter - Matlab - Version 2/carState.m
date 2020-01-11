classdef carState < handle
    
    properties 
        x;
        y;
        Vx;
        Vy;
        theta;

        %{ Remember to Update:  }%
        numValues;
    end
    
    methods

        function obj = carState()
            %c'tor:
            obj.x = 0;
            obj.y= 0;
            obj.Vx= 0;
            obj.Vy =0;
            obj.theta = 0;

             obj.numValues = 5; 
        end%creation function
        
        function set_from_elements(obj , x, y , Vx, Vy , theta )
            %c'tor:
            obj.x= x;
            obj.y= y;
            obj.Vx= Vx;
            obj.Vy= Vy;
            obj.theta = theta;
        end%creation function
        
        function set_from_vector(obj , vector)
            %c'tor:
            obj.x = vector(1);
            obj.y = vector(2);
            obj.Vx = vector(3);
            obj.Vy = vector(4);
            obj.theta = vector(5);
        end%creation function
        
        
        function stateVector  =  vector(obj)
            stateVector = zeros(obj.numValues , 1);
            stateVector(1) = obj.x;
            stateVector(2) = obj.y;
            stateVector(3) = obj.Vx;
            stateVector(4) = obj.Vy;
            stateVector(5) = obj.theta;
        end
        
        function disp(obj)
            disp(join(["car state:"]));
            disp(join(["x=",obj.x]));
            disp(join(["y=",obj.y]));
            disp(join(["Vx=",obj.Vx]));
            disp(join(["Vy=",obj.Vy]));
            disp(join(["theta=",obj.theta]));
        end
        
        
    end%methods
    
end%classdef