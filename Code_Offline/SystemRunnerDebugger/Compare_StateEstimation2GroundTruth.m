%% Ground Truth:

N = length(GroundTruth)

gt = struct();
gt.time = zeros(1,N);
gt.delta = zeros(1,N);
gt.speed = zeros(1,N);
gt.theta = zeros(1,N);
gt.x = zeros(1,N);
gt.y = zeros(1,N);

for i = 1 : N
    gt.time(i) = GroundTruth{i}.time_in_milisec;
    gt.delta(i) = GroundTruth{i}.delta;
    gt.speed(i) = GroundTruth{i}.speed;
    gt.theta(i) = GroundTruth{i}.theta;
    gt.x(i) = GroundTruth{i}.x;
    gt.y(i) = GroundTruth{i}.y;
    
end

%% State Estimation:
M = length(StateEstimation)

state = struct();
state.x = zeros(1,M);
state.y = zeros(1,M);
state.theta = zeros(1,M);

for j = 1 : M
    state.x(j) = StateEstimation{j}.x;
    state.y(j) = StateEstimation{j}.y;
    state.theta(j) = StateEstimation{j}.theta;
end
%%
 
% Position:
figure()
plot( gt.y , gt.x );
hold on
plot(state.y  , state.x );
grid on
grid minor
xlabel('yEast')
ylabel('xNorth')
legend('Ground Truth' , 'Staet Estimation')
title('Position Estimation Vs Truth')

% Theta:


figure()

subplot(2,1,1)
plot( gt.theta  );
grid on
grid minor
xlabel('yEast')
ylabel('xNorth')
title('Theta - GroundTruth')
theta_state =   pi/2 - state.theta;

subplot(2,1,2)
plot( theta_state  );
grid on
grid minor
xlabel('yEast')
ylabel('xNorth')
title('Theta - StateEstimation')