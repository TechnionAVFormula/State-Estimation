function newState = dynamic_model(crntState , translatedSensorsData , delta_t)
%%
carModel = struct('L',1.530);                         
newState = carState();

%%
newState.theta = delta_t*translatedSensorsData.mean_velocty*tan((translatedSensorsData.steering_angle))/carModel.L   + crntState.theta ;
newState.theta = fix_angle(newState.theta );

newState.x_north =  delta_t*translatedSensorsData.mean_velocty*cos( (crntState.theta))   + crntState.x_north ;
newState.y_east   =  delta_t* translatedSensorsData.mean_velocty*sin((crntState.theta))   + crntState.y_east ;

newState.Vx = delta_t*0  + crntState.Vx;   %times  0 because we don;t have acceleration for now....
newState.Vy = delta_t*0  + crntState.Vx;   %times  0 because we don;t have acceleration for now.... 
end%function

%% ============================== sub-functions ============================== %%

function theta =  fix_angle(theta)
    while theta > 2*pi
        theta = theta - 2*pi;
    end
    
    while theta < 0 
        theta  = theta + 2*pi;
    end
end