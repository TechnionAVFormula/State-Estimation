%  outStruct = parse_log2mat(fullpath)
%%
clear all; clc;
tic;
folder = 'C:\Users\NGBig\OneDrive - Technion\State Estimation Drive\logs';
file  = 'log_Endurance_StateEstimation_01.csv';
fullpath = join([folder , filesep,  file   ] )    ;

format long
%%
line_of_titles = 14;
table = readtable(fullpath , 'HeaderLines' ,  line_of_titles )  ;
%
start_of_data = 4;
length_data = size(table , 1) - start_of_data + 1;
outStruct = struct();
outStruct.length_data  = length_data;
%
km_per_hour_to_meter_per_second = (10/36) ;
rear_wheel_coef = (15/8)  ;

i_limit = length_data ; 
% i_limit = 500 ; 
%%
for i = 1 : i_limit
    %time:
    tableIndex = i + start_of_data - 1;
    outStruct.time(i,1) =            str2double(  table.Time{tableIndex} )             ;
    
    %Convertrs from km/h  to  m/s:
    outStruct.WheelSpeedRearLeft(i,1)    =  str2double(  table.WheelSpeedRearLeft{ tableIndex  } ) *km_per_hour_to_meter_per_second *rear_wheel_coef ;
    outStruct.WheelSpeedRearRight(i,1)  =  str2double(  table.WheelSpeedRearRight{ tableIndex  } ) *km_per_hour_to_meter_per_second *rear_wheel_coef ;
    outStruct.WheelSpeedFrontLeft(i,1)   =  str2double(  table.WheelSpeedFrontLeft{ tableIndex  } ) *km_per_hour_to_meter_per_second ;
    
    % Broken
    %     outStruct.WheelSpeedFrontRight(i,1) =  str2double(  table.WheelSpeedFrontRight{ tableIndex  } )  *km_per_hour_to_meter_per_second ; 
   
    outStruct.SteeringAngle(i,1)                =  str2double(  table.SteeringAngle{tableIndex} )   ;
    
    %GPS reading in G forces
    outStruct.VehicleGPSAccelerationLateral(i,1) =  str2double(  table.VehicleGPSAccelerationLateral{tableIndex } )  ;         
    outStruct.VehicleGPSAccelerationLongitudinal(i,1) =  str2double(  table.VehicleGPSAccelerationLongitudinal{tableIndex } )  ;  
    
    
    %GPS reading in deg
    outStruct.VehicleGPSPositionLongitude_deg(i,1) =  str2double(  table.VehicleGPSPositionLongitude{tableIndex} )  ;       
    outStruct.VehicleGPSPositionLatitude_deg(i,1)    =  str2double(  table.VehicleGPSPositionLatitude{tableIndex} )  ;         
    
    %convert to meters
    if i == 1
        outStruct.xGPS(i,1)    = 0;
        outStruct.yGPS(i,1)    = 0;
        
        %for distance measurement:
        llo =  [outStruct.VehicleGPSPositionLatitude_deg(i,1)  , outStruct.VehicleGPSPositionLongitude_deg(i,1)   ];
        href = 0;
        psio = 90;
    else
        lla = [outStruct.VehicleGPSPositionLatitude_deg(i,1)  , outStruct.VehicleGPSPositionLongitude_deg(i,1)     , 0];
        
        try
            flatearth_pos = lla2flat(lla, llo, psio, href);
        catch
            outStruct.xGPS(i,1)    = outStruct.xGPS(i-1,1);
            outStruct.yGPS(i,1)    = outStruct.yGPS(i-1,1);
            outStruct.length_data = i;
            break
        end
        outStruct.xGPS(i,1)    = flatearth_pos(1);
        outStruct.yGPS(i,1)    = flatearth_pos(2);
    end
    
    
    % Display Progress
    if (  mod(i,100) == 0 )
        disp( i );
    end
    
end % for i

outStruct.delta_time = outStruct.time(3) - outStruct.time(2);
toc;
disp(outStruct);


