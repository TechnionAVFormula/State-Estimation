%{Set a number for a wheel position }%
classdef wheelPosition < handle
    
    methods(Static)
        
        function num = RearLeft()
            num = 1;
        end
        function num = RearRight()
            num = 2;
        end
        function num = FrontLeft()
            num = 3;
        end
        function num = FrontRight()
            warning('wheelPosition:  FrontRight wheel  sensor is currently dead');
            num = 4;
        end
              
    end%static methods
    
end%class def 