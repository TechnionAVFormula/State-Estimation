function angle = steering_reading2angle(reading)

Tie_rod_step  = reading /1000; %from [mm] to [m]
Tie_rod_wheel_pose = 0.046; % meters from the center of the wheel
angle = tan(Tie_rod_step/Tie_rod_wheel_pose);

end