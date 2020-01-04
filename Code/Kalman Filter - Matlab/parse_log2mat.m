%  outStruct = parse_log2mat(fullpath)
%%
clear all; clc;
tic;
folder = 'C:\Users\NGBig\OneDrive - Technion\State Estimation Drive\logs';
file  = 'log_Endurance_StateEstimation_01.csv';
fullpath = join([folder , filesep,  file   ] )    ;
%%
table = readtable(fullpath , 'HeaderLines' ,  14  )  ;
%
start_of_data = 4;
length_data = size(table , 1) - start_of_data;
outStruct = struct();
%
km_per_hour_to_meter_per_second = (10/36) ;
rear_wheel_coef = (15/8)  ;

for i = 1 : length_data
    %time:
    outStruct.time(i,1) = str2double(  table.Time{i + start_of_data - 1 } )  ;
    
    %Convertrs from km/h  to  m/s:
    outStruct.WheelSpeedRearLeft(i,1)   =  str2double(  table.WheelSpeedRearLeft{i + start_of_data - 1 } ) *km_per_hour_to_meter_per_second *rear_wheel_coef ;
    outStruct.WheelSpeedRearRight(i,1) =  str2double(  table.WheelSpeedRearRight{i + start_of_data - 1 } ) *km_per_hour_to_meter_per_second *rear_wheel_coef ;
    outStruct.WheelSpeedFrontLeft(i,1)  =  str2double(  table.WheelSpeedFrontLeft{i + start_of_data - 1 } ) *km_per_hour_to_meter_per_second ;
    outStruct.WheelSpeedFrontRight(i,1) =  str2double(  table.WheelSpeedFrontRight{i + start_of_data - 1 } )  *km_per_hour_to_meter_per_second ; 
   
    outStruct.SteeringAngle(i,1)                =  str2double(  table.SteeringAngle{i + start_of_data - 1 } )   ;
    
    %GPS reading in G forces
    outStruct.VehicleGPSAccelerationLateral(i,1) =  str2double(  table.VehicleGPSAccelerationLateral{i + start_of_data - 1 } )  ;         
    outStruct.VehicleGPSAccelerationLongitudinal(i,1) =  str2double(  table.VehicleGPSAccelerationLongitudinal{i + start_of_data - 1 } )  ;  
    
    
    %GPS reading in deg
    outStruct.VehicleGPSPositionLatitude_deg(i,1)    =  str2double(  table.VehicleGPSPositionLatitude{i + start_of_data - 1 } )  ;         
    outStruct.VehicleGPSPositionLongitude_deg(i,1) =  str2double(  table.VehicleGPSPositionLongitude{i + start_of_data - 1 } )  ;       
    %convert to meters
    if i == 1
        outStruct.GPSPositionX(i,1)    = 0;
        outStruct.GPSPositionY(i,1)    = 0;
    else
        latlon0 =  [outStruct.VehicleGPSPositionLatitude_deg(1)  ,  outStruct.VehicleGPSPositionLongitude_deg(1)];
        %compute Latitued (longtitude const): 
        latlon1 =  [outStruct.VehicleGPSPositionLatitude_deg(i)   ,   outStruct.VehicleGPSPositionLongitude_deg(1)   ];
        [Latitued_dis_km , ~] = distance_lat_lon_km( latlon0 , latlon1  ) ;
        %compute Longtitudee (latitued const): 
        latlon1 =  [outStruct.VehicleGPSPositionLatitude_deg(1)  ,  outStruct.VehicleGPSPositionLongitude_deg(i)];
        [Longtitudee_dis_km , ~] = distance_lat_lon_km( latlon0 , latlon1  ) ;
        %convery to meters
        outStruct.GPSPositionLatitude_meters(i,1)    = Latitued_dis_km/1000;
        outStruct.GPSPositionLongitude_meters(i,1) = Longtitudee_dis_km/1000;
    end
    
    
    if (  mod(i,100) == 0 )
        disp( i );
    end
    
end % for i

outStruct.delta_time = outStruct.time(3) - outStruct.time(2);
outStruct.length_data = length_data;
toc;
disp(outStruct);


