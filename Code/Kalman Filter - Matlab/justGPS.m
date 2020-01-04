close all ; clear all; clc;
load 'Log01'
format long

%%
figure();
k=1;
plot(  Log01.xGPS(k)   ,  Log01.yGPS(k) , '*r'    )
for k  = 100   :    Log01.length_data - 100

if ( Log01.xGPS(k) ~=  Log01.xGPS(k-1)  )
    hold on
    plot(  Log01.xGPS(k)   ,  Log01.yGPS(k) , '*r'    )
    drawnow
end

end % for k
%%
function  out =    fstate_from_carDynamics(vector , sensorsData ,  delta_t)
    
crntState = carState();
crntState.set_from_vector(vector);
    
newState = dynamic_model(crntState , sensorsData ,  delta_t);
out = newState.vector();

end

function sensorsData = updateSensorsData(Log01 , k , sensorsData)

%FrontRight is dead
sensorsData.WheelSpeedFrontLeft = Log01.WheelSpeedFrontLeft(k);
sensorsData.WheelSpeedRearLeft = Log01.WheelSpeedRearLeft(k);
sensorsData.WheelSpeedRearRight = Log01.WheelSpeedRearRight(k);

sensorsData.steering_angle = Log01.SteeringAngle(k);

end

function GPStheta = estimate_theta_from_GPS(Log01 , k , previousTheta)
theta = rad2deg( tan(  (Log01.yGPS(k+1) - Log01.yGPS(k) ) /(Log01.xGPS(k+1) - Log01.xGPS(k) )   ) )  ;
if ( isnan(theta) ) 
    GPStheta = previousTheta;
else
    GPStheta = theta;
end
end