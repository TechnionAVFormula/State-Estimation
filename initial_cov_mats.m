function [P,R,Q]  = initial_cov_mats()
                %x    %y      %Vx  %Vy %Theta
R  =   [ [    0.9   , 0      ,   0       ,  0         , 0]  ;  ...  
            [     0     ,  0.9  ,   0       ,  0         , 0]  ; ...
            [     0     ,    0   ,  0.1555     ,  0         , 0]  ; ...
            [     0      ,   0   ,   0       ,  0.1555         , 0]  ; ...
            [     0      ,   0   ,   0       ,  0         , 0.22]   ]  ;
        %measurement noise of x,y are currently sheker ve'cazav because I didn't find
        %the error of the GPS
        %theta measurement noise given in degrees
        %I uesd the error given in the files plus the statistical measurment error
Q  =   [ [    0.02   , 0      ,   0       ,  0         , 0]  ;  ...  
            [     0     ,  0.02  ,   0       ,  0         , 0]  ; ...
            [     0     ,    0   ,  0.23     ,  0           , 0]  ; ...
            [     0      ,   0   ,   0       ,  0.23        , 0]  ; ...
            [     0      ,   0   ,   0       ,  0           , 0.33]   ]  ;   
        %same here for x,y. Nir please be proud of me 
        %process noise calculated weighting the filter's update step and dynamic model
 P =[0.124536262397879,1.68586023371090e-07,0,0,-2.04133926343496e-08; ... 
     1.68586023371084e-07,0.124583808868131,0,0,-0.00353120114306259;0,0,0.118614066163451,0.0186140661634507,0; ...
     0,0,0.0186140661634507,0.118614066163451,0;-2.04133926343496e-08,-0.00353120114306259,0,0,2.65329138921966] ;
 
end