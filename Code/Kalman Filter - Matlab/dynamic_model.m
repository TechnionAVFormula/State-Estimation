function newState = dynamic_model(crntState   , sensorsData , delta_t)
%%
sensorsData = sensorsData() ;
%
carModel = struct( 'm' , 1250 , ...  %mass  [Kg]
                                'P' , 100000 , ... %Peak Engine Power [W]
                                'A' , 1 , ... surface area [m^2]
                                'Cd' , 0.3 , ...  %Drag coefficients [-]
                                'L' , 2.5  ... WheelBase length [m]
                                );
%
                                 
newState = carState();

fake_acceleration_factor = 1/80;

%%
newState.x_pos =  delta_t* crntState.velocity *cos( deg2rad(crntState.theta)  )   + crntState.x_pos ;
newState.y_pos =  delta_t* crntState.velocity *sin( deg2rad(crntState.theta)  )   + crntState.y_pos ;
%newState.velocity = (delta_t/carModel.m )*(carModel.P *control_input.throttle/crntState.velocity -  carModel.A*carModel.Cd * crntState.velocity*crntState.velocity)  ; 
newState.velocity = (delta_t/carModel.m )* carModel.P*fake_acceleration_factor   +   crntState.velocity ;
newState.theta =  rad2deg(atan(        delta_t*crntState.velocity* tan(deg2rad(sensorsData.steering_angle))  /carModel.L    ))  + crntState.theta ;
newState.theta = fix_angle(newState.theta);

crntState=newState ;
disp(crntState);

end%function

%% ============================== sub-functions ============================== %%

function theta =  fix_angle(theta)

if (theta >= 360)
    theta = theta - 360;
elseif (theta < 0)
    theta = theta + 360;
end
   
end