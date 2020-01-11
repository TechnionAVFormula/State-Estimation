close all ; clear all; clc;
format long
log = ask_user_for_log_file();
%% Initial Vals:
crntState  =  set_initial_carState();
[P,R,Q]  = initial_cov_mats();

x = crntState.vector();
sensorsData = sensorsData() ;
hmeas= @(x)   x;
GPStheta = 0 ;

%%
fig1 = figure(1);
for k  = 1  :    Log01.length_data - 1

sensorsData = update_sensorsDate_from_log(log , k , sensorsData);
delta_t = log.delta_time;

fstateVectpr =  @(vector)     fstate_from_carDynamics(stateVector , sensorsData ,  delta_t) ;

z_vector = update_zVector_from_log(log , k, GPStheta);


%[x,P]=ekf(fstate,x,P,hmeas,z,Q,R)
[x,P]=ekf(fstateVectpr,x,P,hmeas  ,   z_vector  ,   Q   , R) ;

figure(fig1)


if k== 1151
   disp(join(['k=', num2str(k)]))
end
if (abs(x(1))  >  1000)
    disp(join(['k=', num2str(k)]))
    disp(x); 
end

% Estimate:
hold on
plot(x(1) , x(2)   , '.b');
% GPS:
if ( Log01.xGPS(k) ~=  Log01.xGPS(k-1)  )
    hold on
    plot(  Log01.xGPS(k)   ,  Log01.yGPS(k) , '*r'    )
    drawnow
end


end % for k
%%


function  out =    fstate_from_carDynamics(stateVector , sensorsData ,  delta_t)
    
crntState = carState();
crntState.set_from_vector(stateVector);
    
newState = dynamic_model(crntState , sensorsData ,  delta_t);
out = newState.vector();

end

function sensorsData = update_sensorsDate_from_log(log , k , sensorsData)

%FrontRight is dead
sensorsData.WheelSpeedFrontLeft = log.WheelSpeedFrontLeft(k);
sensorsData.WheelSpeedRearLeft = log.WheelSpeedRearLeft(k);
sensorsData.WheelSpeedRearRight = log.WheelSpeedRearRight(k);

sensorsData.steering_angle = steering_reading2angle(  log.SteeringAngle(k)  ) ;

end

function GPStheta = estimate_theta_from_GPS(Log01 , k , previousTheta)
theta = rad2deg( tan(  (Log01.yGPS(k+1) - Log01.yGPS(k) ) /(Log01.xGPS(k+1) - Log01.xGPS(k) )   ) )  ;
if ( isnan(theta) ) 
    GPStheta = previousTheta;
else
    GPStheta = theta;
end
end

function log =  ask_user_for_log_file()

[name ,path] = uigetfile('*' , 'Choose prepared-log as a .mat file  or a raw-log as a .csv file');
fullPath = [path , filesep , name ];
[~,~,ext] = fileparts(fullPath) ;

if strcmp(ext , '.csv')
    tic;
    log = parse_log2mat(fullPath);
    toc;
elseif strcmp(ext , '.mat')
    temp = struct2cell( load(fullPath))  ;
    log = temp{1};
else
    error('ask_user_for_log_file:  invalid file');
end


end%func

function z_vector = update_zVector_from_log(log , k , GPStheta)

GPStheta = estimate_theta_from_GPS(log , k , GPStheta) ;
% tempState.set_from_elements(log.xGPS(k) , log.yGPS(k), log.WheelSpeedFrontLeft(k) , GPStheta ) ;
                %       x                             y                       Vx                   Vy         Theta
z_vector =    [ log.xGPS(k)  ;  log.yGPS(k) ] ;
end