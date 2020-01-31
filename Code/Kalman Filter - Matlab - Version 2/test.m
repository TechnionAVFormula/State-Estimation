close all ; clear all; clc;
format long

%for real run:
% log = ask_user_for_log_file();
%for test:
temp = struct2cell( load('C:\Users\NGBig\Documents\GitHub\State-Estimation\Code\Kalman Filter - Matlab - Version 2\logs\Log01.mat'))  ;
log = temp{1};       clear temp;


control = struct( 'is_plotPosition' , true , ...
                            'is_collectMetaData'  , true , ...
                            'is_debug'   ,  false  ...
                            );

annimation = struct(  ...
                           'is_nothUp' ,  false , ...
                           'is_plotTheta' , true , ....
                           'pause_time'  ,  0.1  ...
                           );
                        
log_start_index = 1;                        


%% Initial Vals:
crntState  =  set_initial_carState();
[P,R,Q]  = initial_cov_mats();

x = crntState.vector();
rawSensorsData_updated    =  rawSensorsData() ;
translatedSensorsData_old   = translatedSensorsData();
translatedSensorsData_new = translatedSensorsData();

hmeas= @(x)   x;
GPStheta = 0 ;

%%
if control.is_plotPosition
    [fig1 , plot_struct] =plot_init(annimation);
end

length_data  = size(log , 1) ;
for log_line_indx  = log_start_index  :   length_data
    
    %{Get data from logs.  and understand it .   Not in real life }
    
    [rawSensorsData_updated , is_legit] = get_crnt_sensorsData_from_log(log , log_line_indx , rawSensorsData_updated );
    if ~is_legit
        error("log:  data not legit");
        break
    end
    
    %{Practical in Real Life: }%
    translatedSensorsData_old.copy_constructor(   translatedSensorsData_new       );
    translatedSensorsData_new.set_from_rawSensorsData(rawSensorsData_updated) ;
    delta_t = translatedSensorsData_new.time - translatedSensorsData_old.time;

    
    %{ Kalman Filter Stuff } %
    
    [z_vector , GPStheta ] = update_zVector_from_sensorsData(translatedSensorsData_old, translatedSensorsData_new, GPStheta);
    fstateVectpr =  @(stateVector)     fstate_from_carDynamics(stateVector , translatedSensorsData_new ,  delta_t) ;
    
%{  Estimate with Kalman Filter or Estimate only with carDynamics:  }
    [x,P]=ekf(fstateVectpr,x,P,hmeas  ,   z_vector  ,   Q   , R) ;
%    
%     if translatedSensorsData_new.is_newGPS
%         %[x,P]=ekf(fstate,x,P,hmeas,z,Q,R)
%         [x,P]=ekf(fstateVectpr,x,P,hmeas  ,   z_vector  ,   Q   , R) ;
%     else
%         x= fstateVectpr(x);
%     end
    
    %{Plot Stuff}
    if control.is_plotPosition
        plot_struct =  plot_location(fig1, plot_struct , x ,translatedSensorsData_new ,annimation  ) ;
    end
    
end % for loop
%%

function  out =    fstate_from_carDynamics(stateVector , translatedSensorsData ,  delta_t)
    
crntState = carState();
crntState.set_from_vector(stateVector);
    
newState = dynamic_model(crntState , translatedSensorsData ,  delta_t);
out = newState.vector();

end

function [new_sensorsData , is_legit] = get_crnt_sensorsData_from_log(log , log_line_indx , given_sensorsData )
new_sensorsData=given_sensorsData;

%{Get crnt log line and check if legit: }
crntLine  =log(log_line_indx , :);
if any(isnan(table2array(crntLine)))   % find if at least 1 element is NaN
    is_legit = false;
else
     is_legit = true;
end % if

%{parse log into sensors data: }
%FrontRight is dead
new_sensorsData.WheelSpeedFrontLeft = crntLine.WheelSpeedFrontLeft;
new_sensorsData.WheelSpeedRearLeft = crntLine.WheelSpeedRearLeft;
new_sensorsData.WheelSpeedRearRight = crntLine.WheelSpeedRearRight;

new_sensorsData.steering_angle = crntLine.SteeringAngle;

new_sensorsData.GPS_latitude      = crntLine.VehicleGPSPositionLatitude;
new_sensorsData.GPS_longtitude = crntLine.VehicleGPSPositionLongitude;

new_sensorsData.time  =  crntLine.Time ; 

% new_sensorsData.accelerate_forward  = !WorkInProgress;
% new_sensorsData.accelerate_side         = !WorkInProgress;



end %function

function log =  ask_user_for_log_file()

message_prompt='Choose prepared-log as a .mat file  or a raw-log as a .csv file';
disp(message_prompt);
[name ,path] = uigetfile('*' , message_prompt);
fullPath = [path , filesep , name ];
[~,~,ext] = fileparts(fullPath) ;

if strcmp(ext , '.csv')
    disp('Parsing log from .csv file. This may take some time.');
    tic;
    log = parse_log2mat(fullPath);
    toc;
elseif strcmp(ext , '.mat')
    disp('Taking a prepared log file');
    temp = struct2cell( load(fullPath))  ;
    log = temp{1};
else
    error('ask_user_for_log_file:  invalid file');
end


end%func

function [z_vector , GPStheta_new ] = update_zVector_from_sensorsData(translatedSensorsData1,translatedSensorsData2, GPStheta_old)
%     translatedSensorsData2  is more recent than translatedSensorsData1

    z_vector =zeros(5,1);
    x1 = translatedSensorsData1.x_north;
    y1 = translatedSensorsData1.y_east;
    x2 = translatedSensorsData2.x_north;
    y2 = translatedSensorsData2.y_east;
        
    if translatedSensorsData2.is_newGPS
        GPStheta_new = atan2( y2-y1 , x2-x1 );
    else
        GPStheta_new =GPStheta_old;
    end
    
    z_vector(1) = x2;
    z_vector(2) = y2;
    z_vector(3) = translatedSensorsData1.mean_velocty * cos( GPStheta_new);  % Vx
    z_vector(4) = translatedSensorsData1.mean_velocty * sin( GPStheta_new);  % Vy
    z_vector(5) = GPStheta_new;  % Vy

end%func

function plot_struct =  plot_location(fig1, plot_struct , x_vector  ,translatedSensorsData2 , annimation)

figure(fig1);
hold on

%{ Estimate:  }%
plot(x_vector(1) , x_vector(2)   , '.b');
drawnow
% GPS:
if (translatedSensorsData2.is_newGPS)
    hold on
    if annimation.is_nothUp
        error('is_nothUp: not supported');
    else
        plot(  translatedSensorsData2.x_north  ,  translatedSensorsData2.y_east, '*r'    )
        drawnow
    end
end

if annimation.pause_time
    pause(annimation.pause_time);
end

if annimation.is_plotTheta
    theta = x_vector(5);
    arrow_scale = 2;
    x = x_vector(1);
    y = x_vector(2);
    u = cos((theta));
    v = sin((theta));
    if  isempty(fieldnames(plot_struct))
        plot_struct.thetaArrow = quiver(x,y,u,v );
        plot_struct.thetaArrow.Color = 'Black';
%         plot_struct.thetaArrow.LineWidth = 1;
    else
         delete(plot_struct.thetaArrow);
        plot_struct.thetaArrow = quiver(x,y,u,v );
        plot_struct.thetaArrow.Color = 'Black';
        plot_struct.thetaArrow.AutoScale;
        plot_struct.thetaArrow.AutoScaleFactor = arrow_scale;
%         plot_struct.thetaArrow.LineWidth = 1;
    end
end

end

function [fig_out , plot_struct] = plot_init(annimation)

    %For Beautiful Graphs:
    set(groot, 'defaultAxesXGrid', 'on'); % x grid
    set(groot, 'defaultAxesYGrid', 'on'); % y grid
    set(groot, 'defaultAxesBox', 'on'); % plot in a box

    %for toolbars in figures:
    set(groot,'defaultFigureCreateFcn',@(fig,~)addToolbarExplorationButtons(fig))
    set(groot,'defaultAxesCreateFcn',@(ax,~)set(ax.Toolbar,'Visible','off'))

    
    
    if annimation.is_nothUp
        error('is_nothUp: not yet supported')
    else
        fig_out = figure(1);
        xlabel("x_{north}")
        ylabel("y_{east}")
        plot_struct = struct;
    end

end