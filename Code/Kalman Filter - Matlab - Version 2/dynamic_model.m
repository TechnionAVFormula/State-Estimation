function newState = dynamic_model(crntState , translatedSensorsData2 , delta_t)
%%
carModel = struct('L',1530);                         
newState = carState();


%%
newState.theta = delta_t*translatedSensorsData2.mean_velocty*tan(deg2rad(translatedSensorsData2.steering_angle))/carModel.L   + crntState.theta ;
newState.theta = fix_angle(newState.theta );

newState.x_north =  delta_t*translatedSensorsData2.mean_velocty*cos( deg2rad(crntState.theta))   + crntState.x_north ;
newState.y_east   =  delta_t* translatedSensorsData2.mean_velocty*sin(deg2rad(crntState.theta))   + crntState.y_east ;

newState.Vx = delta_t*0  + crntState.Vx;   %times  0 because we don;t have acceleration for now....
newState.Vy = delta_t*0  + crntState.Vx;   %times  0 because we don;t have acceleration for now.... 
end%function

%% ============================== sub-functions ============================== %%

function theta =  fix_angle(theta)

if (theta >= 360)
    theta = theta - 360;
elseif (theta < 0)
    theta = theta + 360;
end
   
end