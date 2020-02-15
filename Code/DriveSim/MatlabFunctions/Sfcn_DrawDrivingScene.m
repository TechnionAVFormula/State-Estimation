function Sfcn_DrawDrivingScene(block)
setup(block);
end
function setup(block) %runs at t=0 i/o definitions
block.SetSimViewingDevice(true);

%dialog parameters
block.NumDialogPrms = 3;
block.DialogPrmsTunable = {'Nontunable','Nontunable','Nontunable'}; %can change during simulation
%[ControlTs,L]

%register number of ports
block.NumInputPorts = 3;
block.NumOutputPorts = 2;

%setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;

%Register the properties of the input ports
%Enable
block.InputPort(1).Complexity     ='Real';
block.InputPort(1).DataTypeId     =-1;
block.InputPort(1).Dimensions     =1;
block.InputPort(1).SamplingMode   ='Sample';
%State
block.InputPort(2).Complexity     ='Real';
block.InputPort(2).DataTypeId     =-1;
block.InputPort(2).Dimensions     =5;
block.InputPort(2).SamplingMode   ='Sample';
%Time
block.InputPort(3).Complexity     ='Real';
block.InputPort(3).DataTypeId     =-1;
block.InputPort(3).Dimensions     =1;
block.InputPort(3).SamplingMode   ='Sample';

%Register the properties of the output ports
%keys pressed
block.OutputPort(1).Dimensions   = 6; %logical [down,up,right,left,space,left control]
block.OutputPort(1).SamplingMode = 'Sample';
block.OutputPort(1).DatatypeID   = 0;
%trigger
block.OutputPort(2).Dimensions   = 1;
block.OutputPort(2).SamplingMode = 'Sample';
block.OutputPort(2).DatatypeID   = 0;

%Register sample time
ControlTs=block.DialogPrm(1).Data;
block.SampleTimes = [ControlTs 0]; %[discrete time, offset]

%specify block simStateCompliace
block.SimStateCompliance = 'HasNoSimState';

%register functions
block.RegBlockMethod('InitializeConditions',    @InitializeConditions);
block.RegBlockMethod('Start',                   @Start);
block.RegBlockMethod('Terminate',               @Terminate);
block.RegBlockMethod('Outputs',                 @Outputs);
block.RegBlockMethod('CheckParameters',         @CheckPrms);
block.RegBlockMethod('ProcessParameters',       @ProcessPrms);
end
function Start(block) %runs on t=0
%Check for valid key inputs
NET.addAssembly('PresentationCore');
akey = System.Windows.Input.Key.A;  %use any key to get the enum type
keys = System.Enum.GetValues(akey.GetType);  %get all members of enumeration
% keynames = cell(System.Enum.GetNames(akey.GetType))';
iskeyvalid = true(keys.Length, 1);
iskeydown = false(keys.Length, 1);
for keyidx = 1:keys.Length
   try
       iskeydown(keyidx) = System.Windows.Input.Keyboard.IsKeyDown(keys(keyidx));
   catch
       iskeyvalid(keyidx) = false;
   end
end

%% Update User Data
UserData=get(gcbh,'UserData');

UserData.keys=keys;
UserData.iskeyvalid=iskeyvalid;
UserData.SteeringWheelGearRatio=3;

set(gcbh,'UserData',UserData);
end 
function ProcessPrms(block) %runs on every dt (Wasnt checked!)
  block.AutoUpdateRuntimePrms;
end
function InitializeConditions(block) %runs on t=0 and when susbystem is enabled
Enable=block.InputPort(1).Data(1);
if ~Enable, return, end

%check if figute exists and valid. if not - reset it
UserData=get(gcbh,'UserData');
if ~isfield(UserData,'Fig') %first time simulation is activated
     SetupFigAndUserData(block);
elseif ~ishghandle(UserData.Fig) %figure was deleted
    SetupFigAndUserData(block);
else %figure exists, just clear it and start a new
    SetupFigAndUserData(block,UserData.Fig); %reset figure
end
end
function Outputs(block) %runs on every dt
UserData=get(gcbh,'UserData'); %UserData is now a struct of handles and is NOT connected to BlockHandle
if ~ishghandle(UserData.Fig)
     UserData=SetupFigAndUserData(block); %set figure to a new start
end
%General inputs
L=block.DialogPrm(2).Data;
Vmax=block.DialogPrm(3).Data;
x=block.InputPort(2).Data(1);
y=block.InputPort(2).Data(2);
v=block.InputPort(2).Data(3);
theta=block.InputPort(2).Data(4);
delta=block.InputPort(2).Data(5);

%---------update Scene Axes
Rcar=makehgtform('zrotate',theta);
Tcar=makehgtform('translate',[x,y,0]);
%update front wheels
Tfrontwheels2center=makehgtform('translate',[-L/3,0,0]);
RSteeringWheel=makehgtform('zrotate',UserData.SteeringWheelGearRatio*delta);
UserData.hFrontWheelsTransform.Matrix=Tcar*Rcar*inv(Tfrontwheels2center)*RSteeringWheel*Tfrontwheels2center;
%update entire car
UserData.hCarTransform.Matrix=Tcar*Rcar;
addpoints(UserData.hPastLine,x,y);

%check which keys are down
keys=UserData.keys;
iskeyvalid=UserData.iskeyvalid;
iskeydown(iskeyvalid) = arrayfun(@(keyidx) System.Windows.Input.Keyboard.IsKeyDown(keys(keyidx)), find(iskeyvalid));

%if escape is pressed - close simulation
if iskeydown(18) %escape
    PushbuttonCallback;
end

keyspressed=iskeydown([31,33,30,32,23,126]); %check keynames
%31 - down arrow - deccelerate
%33 - up arrow - accelerate
%30 - turn right
%32 - turn left
%23 - space
%126 - left control

%outputs - keystrkes
if any(keyspressed)
    block.OutputPort(1).Data=double(keyspressed);
    block.OutputPort(2).Data=1;
    
    %Updated Limfactor according to vehicle velocity
    if keyspressed(1)||keyspressed(2)
        TargetAccelerationSign=-keyspressed(1)+keyspressed(2);
        TargetVelocity=v+(5/3.6)*TargetAccelerationSign; %5 is what is added on each keypress - see slx
        UserData.LimFactor=10*(TargetVelocity/Vmax)+2;
        set(gcbh,'UserData',UserData); %updates the whole of UserData... unfournate
    end
else
    block.OutputPort(1).Data=[0,0,0,0,0,0];
    block.OutputPort(2).Data=0;
end

%updated SceneAxes limits
xlim(UserData.SceneAxes,UserData.LimFactor*L*[-1,1]+x);
ylim(UserData.SceneAxes,UserData.LimFactor*L*[-1,1]+y);

%--------Update auxiliary axes

%Update time text
Time=block.InputPort(3).Data(1);
UserData.hTimeTxt.String=sprintf('Time %g[s]',Time);

%Update steering wheel orientation and text
UserData.hSteeringWheelTransform.Matrix=RSteeringWheel;
UserData.hSteeringAngleTxt.String=sprintf('Steering Angle %.2g[deg]',round(delta*180/pi,1));

%Update odometer orientation and text
UserData.hOdometerArrow.UData=cos(pi-v/Vmax*pi);
UserData.hOdometerArrow.VData=sin(pi-v/Vmax*pi);
UserData.hOdometerTxt.String=sprintf('Velocity %.3g[km/h]',round(v*3.6,2));

drawnow limitrate
end
function Terminate(block)
UserData=get(gcbh,'UserData');
close(UserData.Fig,'WindowStyle','Normal');
end
%% Auxiliary functions
function UserData=SetupFigAndUserData(block,varargin)
if nargin<2 %figure was not provided in input
    %Create figure
    FigName='Drive Sim';
    Fig = figure(...
        'Name',              FigName,...
        'NumberTitle',        'off',...
        'IntegerHandle',     'off',...
        'Color',             [1,1,1],...
        'MenuBar',           'figure',...
        'ToolBar',           'auto',...
        'HandleVisibility',   'callback',...
        'Resize',            'on',...
        'visible',           'on',...
        'units',            'normalized');
    
    set(Fig,'WindowStyle','Modal');
    
    %Create Scene Axes
    L=block.DialogPrm(2).Data;
    SceneAxes=axes(...
        'parent',       Fig,...
        'position',     [0.2,0.1,0.9,0.8],... %[Left,Buttom,Height,Width]
        'XLim',          2*L*[-1,1],...
        'YLim',          2*L*[-1,1]);
    hold(SceneAxes,'on'); grid(SceneAxes,'on'); axis(SceneAxes,'manual')
    SceneAxes.DataAspectRatio=[1,1,1];
    %Create TimeAxes,SteeringWheelAxes and OdometerAxes
    TimeAxes=axes(...
        'Parent',       Fig,...
        'position',     [0,0.7,0.2,0.2],... %[Left,Buttom,Height,Width]
        'NextPlot',     'add',...
        'Xtick',        [],...
        'Ytick',        [],...
        'Xcolor',       'none',...
        'Ycolor',       'none',...
        'XLim',          [-2,2],...
        'YLim',          [-2,2]);
    SteeringWheelAxes=axes(...
        'Parent',       Fig,...
        'position',     [0,0.45,0.2,0.2],... %[Left,Buttom,Height,Width]
        'NextPlot',     'add',...
        'Xtick',        [],...
        'Ytick',        [],...
        'Xcolor',       'none',...
        'Ycolor',       'none',...
        'XLim',          [-2,2],...
        'YLim',          [-2,2]);
    OdometerAxes=axes(...
        'Parent',       Fig,...
        'position',     [0,0.2,0.2,0.2],... %[Left,Buttom,Height,Width]
        'NextPlot',     'add',...
        'Xtick',        [],...
        'Ytick',        [],...
        'Xcolor',       'none',...
        'Ycolor',       'none',...
        'XLim',          [-2,2],...
        'YLim',          [-2,2]);
    
    %Create PushButton to stop simulation
    uicontrol(...
        'Parent',           Fig,...
        'Style',            'pushbutton',...
        'units',            'normalized',...
        'Position',         [0.05 0.1 0.2 0.1],...
        'String',           'Stop Simulation',...
        'Backgroundcolor',  [0.3010, 0.7450, 0.9330],...
        'Callback',     {@PushbuttonCallback});
    
else %figure was provided in input
    Fig=varargin{1};
    hAxes=findobj(Fig,'type','axes');
    cla(hAxes);
end

%Obtain data
x=block.InputPort(2).Data(1);
y=block.InputPort(2).Data(2);
v=block.InputPort(2).Data(3);
theta=block.InputPort(2).Data(4);
delta=block.InputPort(2).Data(5);
L=block.DialogPrm(2).Data; 

%-------------Initalize Drawing

%SceneAxes 
hCarTransform=hgtransform(SceneAxes);
hFrontWheelsTransform=hgtransform(SceneAxes);
DrawCar(hCarTransform,hFrontWheelsTransform,L/2,L);
hPastLine=animatedline(SceneAxes,0,0,'linewidth',2,'color','r');

%TimeAxes
DrawClock(TimeAxes);
hTimeTxt=text(TimeAxes,-1,2,'');

%SteeringWheelAxes
hSteeringWheelTransform=hgtransform(SteeringWheelAxes);
DrawSteeringWheel(hSteeringWheelTransform);
hSteeringAngleTxt=text(SteeringWheelAxes,-1,2,'');

%OdometerAxes
hOdometer=DrawOdometer(OdometerAxes);
hOdometerArrow=hOdometer(2);
hOdometerTxt=text(OdometerAxes,-1,2,'');
%% Storing handles to "figure" and block "UserData"
UserData=get(gcbh,'UserData');

%Update
UserData.Fig = Fig;
UserData.SceneAxes = SceneAxes;
UserData.hCarTransform = hCarTransform;
UserData.hFrontWheelsTransform = hFrontWheelsTransform;
UserData.hPastLine=hPastLine;
UserData.hTimeTxt = hTimeTxt;
UserData.LimFactor=2;
UserData.hSteeringWheelTransform = hSteeringWheelTransform;
UserData.hSteeringAngleTxt = hSteeringAngleTxt;
UserData.hOdometerArrow = hOdometerArrow;
UserData.hOdometerTxt = hOdometerTxt;

%Store in both figure and block
set(gcbh,'UserData',UserData);
end
function hCar=DrawCar(hCarTransform,hFrontWheelsTransform,W,L)
        purple=[0.5,0,0.5];
        yellow=[1,0.8,0];
        brown=[0.6350, 0.0780, 0.1840];
        t=linspace(0,2*pi,7);
        t=t(1:end-1);
        r=W/6; %lights raidus
        
        %            ---------
        %------------------|O
        %|                 |
        %|                 |   Main axis is the 'X' axis
        %|                 |
        %------------------|O
        %            ---------
        
        
        %Draw carBody
        x_body=L/2*[-1, -1, 1, 1];
        y_body=W/2*[1, -1, -1, 1];
        hCarBody=patch('Parent',hCarTransform,'XData',x_body,'YData',y_body,'facecolor',purple);
        
        %Draw front right Wheel
        x_frw=L/4*[-1, -1, 1, 1]+L/3;
        y_frw=W/4*[1, -1, -1, 1]-1.5*(W/2);
        hFrontRightWheel=patch('Parent',hFrontWheelsTransform,'XData',x_frw,'YData',y_frw,'facecolor',brown);
        
        %Draw front left wheel
        x_flw=L/4*[-1, -1, 1, 1]+L/3;
        y_flw=W/4*[1, -1, -1, 1]+1.5*(W/2);
        hFrontLeftWheel=patch('Parent',hFrontWheelsTransform,'XData',x_flw,'YData',y_flw,'facecolor',brown);
        
        %Draw rear right Wheel
        x_rrw=L/4*[-1, -1, 1, 1]-L/3;
        y_rrw=W/4*[1, -1, -1, 1]-1.5*(W/2);
        hRearRightWheel=patch('Parent',hCarTransform,'XData',x_rrw,'YData',y_rrw,'facecolor',brown);
        
        %Draw rear left wheel
        x_rlw=L/4*[-1, -1, 1, 1]-L/3;
        y_rlw=W/4*[1, -1, -1, 1]+1.5*(W/2);
        hRearLeftWheel=patch('Parent',hCarTransform,'XData',x_rlw,'YData',y_rlw,'facecolor',brown);
        
        %Draw right light
        x_rl=r*cos(t)+L/2;
        y_rl=r*sin(t)+W/4;
        hRightLight=patch('Parent',hCarTransform,'XData',x_rl,'YData',y_rl,'facecolor',yellow);
        
        %Draw left light
        x_ll=r*cos(t)+L/2;
        y_ll=r*sin(t)-W/4;
        hLeftLight=patch('Parent',hCarTransform,'XData',x_ll,'YData',y_ll,'facecolor',yellow);
        
        hCar=[hCarBody,hRearRightWheel,hRearLeftWheel,hFrontRightWheel,hFrontLeftWheel,hRightLight,hLeftLight];
end
function hClock=DrawClock(Parent)
t=linspace(0,2*pi,16);
hCircle=plot(cos(t),sin(t),'linewidth',2,'color','k','Parent',Parent);
hSmallLine=plot([0,0.5*cos(pi/3)],[0,0.5*sin(pi/3)],'linewidth',2,'color','k','Parent',Parent);
hBigLine=plot([0,cos(pi/2)],[0,sin(pi/2)],'linewidth',2,'color','k','Parent',Parent);
hClock=[hCircle,hSmallLine,hBigLine];
end
function hSteeringWheel=DrawSteeringWheel(Parent)
t=linspace(0,3*pi,16);
hCircle=plot(cos(t),sin(t),'linewidth',4,'color','k','Parent',Parent);
hTopLine=plot([0,0],[0,1],'linewidth',4,'color','k','Parent',Parent);
hLeftLine=plot([0,cos(-pi/5)],[0,sin(-pi/5)],'linewidth',4,'color','k','Parent',Parent);
hRightLine=plot([0,cos(-pi+pi/5)],[0,sin(-pi+pi/5)],'linewidth',4,'color','k','Parent',Parent);
hSteeringWheel=[hCircle,hTopLine,hLeftLine,hRightLine];
end
function hOdometer=DrawOdometer(Parent)
t=linspace(0,pi,16);
hArc=plot(cos(t),sin(t),'linewidth',2,'color','k','Parent',Parent);
hArrow=quiver(0,0,-1,0,'linewidth',3,'color','r','autoscale','off','Parent',Parent);
hOdometer=[hArc,hArrow];
end
function PushbuttonCallback(obj,eventdata,handle)
set_param(bdroot(gcs),'SimulationCommand', 'stop');
end
%% Unused fcns
function CheckPrms(block)
  %can check validity of parameters here
end