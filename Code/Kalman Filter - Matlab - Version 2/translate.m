classdef translate 
    
    methods(Static) 
        
        %{By: Oren }%
        %{Translate Steering reading from stupid sensor to actual angle: }%
        function angle = steering2angle(reading)  
            Tie_rod_step  = reading /1000; %from [mm] to [m]
            Tie_rod_wheel_pose = 0.046; % meters from the center of the wheel
            angle = rad2deg( tan(Tie_rod_step./Tie_rod_wheel_pose) );
        end
        
        
        %{   From wheel of  --not always--  km/h  to velocity of  m/s  }%
        function velocity = wheelSpeed2velocity(wheelPositionNum , WheelSpeed )
            
            km_per_hour_to_meter_per_second = (10/36) ;
            rear_wheel_coef = (15/8)  ;
            
            %{  turning cases into actual numbers (Decided in wheelPosition class )}%
            FronLeft     = wheelPosition.FrontLeft;
            FrontRight  = wheelPosition.FrontRight;
            RearLeft      = wheelPosition.RearLeft;
            RearRight    = wheelPosition.RearRight;
            
            %{  Cases:  }%
            switch wheelPositionNum
                case FrontRight
                    warning('wheelSpeed2velocity:  FrontRight wheel  sensor is currently dead');
                    velocity   = zeros(size(WheelSpeed)) ;
                case FronLeft
                    velocity =   WheelSpeed .* km_per_hour_to_meter_per_second ;
                case RearRight
                    velocity  = WheelSpeed .* km_per_hour_to_meter_per_second .* rear_wheel_coef ;
                case RearLeft
                    velocity   =  WheelSpeed .* km_per_hour_to_meter_per_second .* rear_wheel_coef ;
                otherwise
                    warning('wheelSpeed2velocity: Not a valid Wheel Name\String ');
                    velocity   = zeros(size(WheelSpeed)) ;
                    
            end%swirch
        end %function 
        
        
        
        function [x_north , y_east] = gpsDeg2meters(GPSLatitudeDeg, GPSLongitudeDeg)
            href = 0;
            psio = 90;
            
            %{ Static location for the first call of this function }%
            persistent static_lla_location
            if ( isempty(static_lla_location) )
                static_lla_location = [ GPSLatitudeDeg  , GPSLongitudeDeg    , 0];
            end
            
            %{ Compute new location llo_location  in comparison to the first location (static_lla_location)  }%
            llo_location =  [GPSLatitudeDeg , GPSLongitudeDeg   ];
            
            try
                flatearth_pos = lla2flat(static_lla_location, llo_location, psio, href);
            catch
                warning('gpsDeg2meters:   problem with input [GPSLatitudeDeg , GPSLongitudeDeg]')
            end
            
            x_north = flatearth_pos(1);
            y_east    = flatearth_pos(2);
            
        end %function

        
    end%Static methods
    
end%class