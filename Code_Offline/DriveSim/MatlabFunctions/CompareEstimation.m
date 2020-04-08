xGT=simout.Ground_Truth.x.Data;
yGT=simout.Ground_Truth.y.Data;
xIMU=simout.IMUIntegration.x.Data;
yIMU=simout.IMUIntegration.y.Data;
xEst=simout.StateEstimation.x.Data;
yEst=simout.StateEstimation.y.Data;
%% Create graphic objects
Fig = figure(...
    'NumberTitle',        'off',...
    'Color',             [1,1,1]);

%Create Scene Axes
SceneAxes=axes('parent',Fig);
hold(SceneAxes,'on'); grid(SceneAxes,'on');
SceneAxes.DataAspectRatio=[1,1,1];

plot(SceneAxes,xGT,yGT,'linewidth',2,'color','b');
% plot(SceneAxes,xIMU,yIMU,'linewidth',2,'color','g');
plot(SceneAxes,xEst,yEst,'linewidth',2,'color','r');
legend(SceneAxes,'Ground Truth','IMU','Estimation');