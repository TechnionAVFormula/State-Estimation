function outTable = parse_log2table(fullpath)
%%
%for test:
%{
clear all; clc;
% folder = 'C:\Users\NGBig\OneDrive - Technion\State Estimation Drive\logs';
% file  = 'log_Endurance_StateEstimation_01.csv';
[file , folder ] = uigetfile('*');
fullpath = join([folder , filesep,  file   ] )    ;
%}

format long
line_of_titles = 14;
inTable = readtable(fullpath , 'HeaderLines' ,  line_of_titles )  ;
%
outTable = table ;   %empty table
inTable(1:3 , :) =[];
%
outTable.SteeringAngle                           = str2double( inTable.SteeringAngle) ;
outTable.Time                                           = str2double( inTable.Time);
outTable.VehicleGPSPositionLatitude    = str2double( inTable.VehicleGPSPositionLatitude);
outTable.VehicleGPSPositionLongitude = str2double( inTable.VehicleGPSPositionLongitude);
outTable.WheelSpeedFrontLeft               = str2double( inTable.WheelSpeedFrontLeft );
outTable.WheelSpeedFrontRight             = str2double( inTable.WheelSpeedFrontRight);
outTable.WheelSpeedRearLeft                 = str2double( inTable.WheelSpeedRearLeft);
outTable.WheelSpeedRearRight               = str2double( inTable.WheelSpeedRearRight) ;

end