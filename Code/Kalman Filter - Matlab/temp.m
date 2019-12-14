clear all; clc;

x  =  carState() ;
x.set_from_elements( 10 , 0 , 2 , 20);
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
z         = carState();
z.set_from_elements(15 , 3  , 4 , 20);
z= z.vector;
%%

% [x,P]=ekf(fstate,x,P,hmeas,z,Q,R)
[x,P]=ekf(fstate,x,P,hmeas  ,   z  ,   Q   , R) ;


%%
function  out =    fstate_from_carDynamics(vector , sensorsData ,  delta_t)
    carState1 = dynamic_model(vector , sensorsData ,  delta_t);
    out = carState1.vector;
    return
end