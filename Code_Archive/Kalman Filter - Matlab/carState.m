classdef carState < handle
    
    properties 
        x;
        y;
        v;
        theta;

        
        %{ Remember to Update:  }%
        numValues;
    end
    
    methods

        function obj = carState()
            %c'tor:
            obj.x = 0;
            obj.y= 0;
            obj.v= 0;
            obj.theta = 0;

            
             obj.numValues = 4; 
        end%creation function
        
        function set_from_elements(obj , x, y , v , theta )
            %c'tor:
            obj.x= x;
            obj.y= y;
            obj.v= v;
            obj.theta = theta;
        end%creation function
        
        function set_from_vector(obj , vector)
            %c'tor:
            obj.x= vector(1);
            obj.y= vector(2);
            obj.v= vector(3);
            obj.theta = vector(4);
        end%creation function
        
        
        function stateVector  =  vector(obj)
            stateVector = zeros(obj.numValues , 1);
            stateVector(1) = obj.x;
            stateVector(2) = obj.y;
            stateVector(3) = obj.v;
            stateVector(4) = obj.theta;
        end
        
        function disp(obj)
            disp(join(["car state:"]));
            disp(join(["x=",obj.x]));
            disp(join(["y=",obj.y]));
            disp(join(["v=",obj.v]));
            disp(join(["theta=",obj.theta]));
        end
        
        function obj = plus(obj  ,  add)
            obj.x = obj.x + add;
            obj.y = obj.y + add;
            obj.v = obj.v + add;
            obj.theta = obj.theta + add;
        end
        
        
        function obj = imag(obj  )
            obj.x = imag(obj.x) ;
            obj.y = imag(obj.y);
            obj.v = imag(obj.v);
            obj.theta = imag(obj.theta);
        end
        
        function obj = mrdivide(obj  ,  dividant)
            obj.x = obj.x / dividant;
            obj.y = obj.y / dividant;
            obj.v = obj.v / dividant;
            obj.theta = obj.theta / dividant;
        end
        
    end%methods
    
end%classdef