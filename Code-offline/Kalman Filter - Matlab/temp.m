close all ; clear all; clc;
load 'Log01'
format long
T = [];
%
crntState  =  carState() ;
i=1;
crntState.set_from_elements(Log01.xGPS(i) , Log01.yGPS(i), Log01.WheelSpeedFrontLeft(i) , 20);
x = crntState.vector();

R  =   [ [    0.9   , 0      ,   0      F ,  0  ]  ;  ... 
            [     0     ,  0.9  ,   0       ,  0  ]  ; ...
            [     0     ,    0   ,  0.2     ,  0  ]  ; ...
            [     0      ,   0   ,   0       ,  5  ]    ]  ;
        
Q  = [   [    0.2      , 0   ,   0       ,  0  ]  ;  ... 
            [     0     ,  0.2   ,   0       ,  0  ]  ; ...
            [     0     ,  0     ,   0.2       ,  0  ]  ; ...
            [     0     ,  0     ,   0          ,  1  ]    ] ;      
        
 P = eye(4);       
 
sensorsData = sensorsData() ;
  
hmeas= @(x)   x;
GPStheta = 0 ;

%%
fig1 = figure(1);
for k  = 100 :    Log01.length_data - 1

sensorsData = updateSensorsData(Log01 , k , sensorsData);
delta_t = Log01.delta_time;

fstate =  @(vector)     fstate_from_carDynamics(vector , sensorsData ,  delta_t) ;

z         = carState( );

GPStheta = estimate_theta_from_GPS(Log01 , k , GPStheta) ;

z.set_from_elements(Log01.xGPS(k) , Log01.yGPS(k), Log01.WheelSpeedFrontLeft(k) , GPStheta ) ;
z= z.vector;

%[x,P]=ekf(fstate,x,P,hmeas,z,Q,R)
[x,P]=ekf(fstate,x,P,hmeas  ,   z  ,   Q   , R) ;

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

sensorsData.steering_angle = steering_reading2angle(  Log01.SteeringAngle(k)  ) ;

end

function GPStheta = estimate_theta_from_GPS(Log01 , k , previousTheta)
theta = rad2deg( tan(  (Log01.yGPS(k+1) - Log01.yGPS(k) ) /(Log01.xGPS(k+1) - Log01.xGPS(k) )   ) )  ;
if ( isnan(theta) ) 
    GPStheta = previousTheta;
else
    GPStheta = theta;
end
end