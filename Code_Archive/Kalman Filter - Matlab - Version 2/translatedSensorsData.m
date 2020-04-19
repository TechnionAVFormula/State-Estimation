classdef translatedSensorsData < handle
    
    %{ Here  measurements are Translated :}%
    properties 
        
        mean_velocty
        steering_angle;  %angel between cars' wheels and main_frame's axis  [deg]
        x_north;
        y_east;
        
        is_newGPS;
        
        time=0;
        
        %not yet exists:
        accelerate_forward;
        accelerate_side;
        theta;
        
    end
    
    methods 
  
        function obj = translatedSensorsData()
        % Build an empty translatedSensorsData struct 
        % FrontRight is dead   
            obj.mean_velocty = 0;
            obj.steering_angle =0;
            obj.x_north = 0;   
            obj.y_east    = 0;
            obj.is_newGPS = true;
            obj.time = 0;
        end
  
        function obj1 = copy_constructor(obj1 , obj2)
        % Build an empty translatedSensorsData struct 
        % FrontRight is dead   
            obj1.mean_velocty = obj2.mean_velocty;
            obj1.steering_angle =obj2.steering_angle;
            obj1.x_north = obj2.x_north ;
            obj1.y_east    = obj2.y_east ;
            obj1.is_newGPS = obj2.is_newGPS;
            obj1.time = obj2.time;
        end
        
        function obj = set_from_rawSensorsData (obj , rawSensorsData )         
        % Build a translatedSensorsData struct from a rawSensorsData struct.
        % Uses  translations from "translate" class.
            
             margin4newGPS_m = 0.01 ; %[meter] 
             
             % FrontRight is dead   
             FrontLeft = translate.wheelSpeed2velocity(wheelPosition.FrontLeft  , rawSensorsData.WheelSpeedFrontLeft ) ;
             RearLeft = translate.wheelSpeed2velocity(wheelPosition.RearLeft  , rawSensorsData.WheelSpeedRearLeft ) ;
             RearRight = translate.wheelSpeed2velocity(wheelPosition.RearRight  , rawSensorsData.WheelSpeedRearRight ) ;
             obj.mean_velocty = (FrontLeft +RearLeft + RearRight)/3 ;
             
             obj.steering_angle = translate.steering2angle( rawSensorsData.steering_angle );
         
         
             [new_x_north , new_y_east ] =  translate.gpsDeg2meters( rawSensorsData.GPS_latitude , rawSensorsData.GPS_longtitude );
             
             if ( abs(  new_x_north - obj.x_north )<margin4newGPS_m )  && ( abs(new_y_east - obj.y_east)<margin4newGPS_m   )
                 obj.is_newGPS = false;
             else
                 obj.is_newGPS = true;
             end
        
             obj.x_north = new_x_north;
             obj.y_east = new_y_east;
             obj.time = rawSensorsData.time;
             
        end %set_from_rawSensorsData
        
    end % methods
end%classdef
