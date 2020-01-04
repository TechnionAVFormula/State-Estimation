function newState = dynamic_model(crntState , sensorsData , delta_t)
%%
carModel = struct('L',1530);                         
newState = carState();


%%
newState.v= sensorsData.m_velocity() ;
newState.theta =  delta_t*crntState.v*tan(deg2rad(sensorsData.steering_angle))/carModel.L   + crntState.theta ;
newState.x =  delta_t* crntState.v *cos( deg2rad(crntState.theta))   + crntState.x ;
newState.y=  delta_t*crntState.v*sin(deg2rad(crntState.theta))   + crntState.y ;
%newState.velocity = (delta_t/carModel.m )*(carModel.P *control_input.throttle/crntState.velocity -  carModel.A*carModel.Cd * crntState.velocity*crntState.velocity)  ; 



end%function

%% ============================== sub-functions ============================== %%

function theta =  fix_angle(theta)

if (theta >= 360)
    theta = theta - 360;
elseif (theta < 0)
    theta = theta + 360;
end
   
end