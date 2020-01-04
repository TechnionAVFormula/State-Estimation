close all ; clear all; clc;
load 'Log01'
Log01.length_data  = length(Log01.WheelSpeedFrontLeft) ;
%
x  =  carState() ;
i=1;
x.set_from_elements(Log01.GPSPositionLatitude_meters(i) , Log01.GPSPositionLongitude_meters(i), Log01.WheelSpeedFrontLeft(i) , 20);
x = x.vector;


R  =(  [ [    0.1   , 0      ,   0       ,  0  ]  ;  ... 
            [     0     ,  0.1  ,   0       ,  0  ]  ; ...
            [     0     ,    0   ,  1.4     ,  0  ]  ; ...
            [     0      ,   0   ,   0       ,  2  ]    ]  ).^2 ;
        
Q  = [   [    0      , 0      ,   0       ,  0  ]  ;  ... 
            [     0     ,  0     ,   0       ,  0  ]  ; ...
            [     0     ,    0   ,   0       ,  0  ]  ; ...
            [     0      ,   0   ,   0       ,  0  ]    ] ;      
        
 P = eye(4);       
 
 sensorsData = sensorsData() ;
sensorsData.steering_angle = 12;
 
delta_t =0.2;
fstate =  @(vector)     fstate_from_carDynamics(vector , sensorsData ,  delta_t) ;  
hmeas= @(carState)   carState;

%%
fig1 = figure(1);
for k  = 1 : Log01.length_data

z         = carState( );
z.set_from_elements(Log01.GPSPositionLatitude_meters(k) , Log01.GPSPositionLongitude_meters(k), Log01.WheelSpeedFrontLeft(k) , 20);
z= z.vector;

%[x,P]=ekf(fstate,x,P,hmeas,z,Q,R)
[x,P]=ekf(fstate,x,P,hmeas  ,   z  ,   Q   , R) ;

figure(fig1)
hold on
plot(x(1) , x(2)   , '.b');
hold on
plot(Log01.GPSPositionLatitude_meters(k)   ,  Log01.GPSPositionLongitude_meters(k) , '.r'    )

end % for k
%%
function  out =    fstate_from_carDynamics(vector , sensorsData ,  delta_t)
    carState1 = dynamic_model(vector , sensorsData ,  delta_t);
    out = carState1.vector;
    return
end