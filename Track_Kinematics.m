function varargout = Track_Kinematics(varargin)
% TRACK_KINEMATICS MATLAB code for Track_Kinematics.fig
%      TRACK_KINEMATICS, by itself, creates a new TRACK_KINEMATICS or raises the existing
%      singleton*.
%
%      H = TRACK_KINEMATICS returns the handle to a new TRACK_KINEMATICS or the handle to
%      the existing singleton*.
%
%      TRACK_KINEMATICS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACK_KINEMATICS.M with the given input arguments.
%
%      TRACK_KINEMATICS('Property','Value',...) creates a new TRACK_KINEMATICS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Track_Kinematics_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Track_Kinematics_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, guidata, GUIHANDLES

% Edit the above text to modify the response to help Track_Kinematics

% Last Modified by GUIDE v2.5 24-Aug-2019 17:27:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Track_Kinematics_OpeningFcn, ...
    'gui_OutputFcn',  @Track_Kinematics_OutputFcn, ...
    'gui_LayoutFcn',  [] , ...
    'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
function Track_Kinematics_OpeningFcn(hObject, eventData, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventData  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user Kinematics (see guidata)
% varargin   command line arguments to Track_Kinematics (see VARARGIN)

% Choose default command line output for Track_Kinematics
handles.output = hObject;

%update Ax and introduce mousedown callback to it. Initalize Fig UserData
%for plotedit toolbar
Ax=handles.Ax;
grid(Ax,'on'); hold(Ax,'on'); axis(Ax,'manual');
xlabel(Ax,'x_1'); ylabel(Ax,'x_2');

%define GuiStruct and input it into handles
handles.GuiStruct.Kinematics=struct();
handles.GuiStruct.Nodes=struct();

guidata(hObject, handles); %save handles in figure
function varargout = Track_Kinematics_OutputFcn(hObject, eventData, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventData  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user Kinematics (see guidata)

% Get default command line output from handles structure
varargout{1} = handles.output;
%% Callbacks
function CalculateKinematicsPush_Callback(hObject, eventData, handles)
handles=guidata(hObject); %Obtain updated handles

%Obtain Nodes from Axes.
%note: flip-up-down as graphic objects are stacked 
Ax=handles.Ax;
Color=[0,0,0];
NodesGraphicObj=flipud(findobj(Ax,'UserData',Color)); %find nodes by their input color to UserData. 
NodesPosition=cell2mat(arrayfun(@(x) [x.XData,x.YData],NodesGraphicObj,'un',0)); %obtain nodes [x1,x2]
NodeAmnt=size(NodesPosition,1);

%Obtain Time and interpolation points amount from GUI
T=str2num(handles.TimeEdit.String);
if isempty(T), errordlg('Time input is invalid'), return, end
N=str2num(handles.IntrpAmntEdit.String);
if isempty(N) || mod(N,1)~=0, errordlg('Interpolation point amount is invalid'), return, end
time=linspace(0,T,N);

%Calculate kinematics assuming constant dt between nodes
dt=T/(NodeAmnt-1);
timeNodes=0:dt:T;

IntrpMethod=handles.IntrpMethodPopUp.String{handles.IntrpMethodPopUp.Value};
switch IntrpMethod %create piecewise-polynomial class
    case 'spline'
        pp=spline(timeNodes,NodesPosition');
    case 'pchip'
        pp=pchip(timeNodes,NodesPosition');
end
dpp=fnder(pp,1); %velocity pp
ddpp=fnder(pp,2); %acceleration pp

r=ppval(pp,time)';
v=ppval(dpp,time)'; 
a=ppval(ddpp,time)';

theta=atan2(r(:,2),r(:,1)); %orientation
speed=vecnorm(v,2,2); %mx1 absolute of velocity
t=v./speed; %tangent vectors mx2

%Calculate normal direction - diretion equation approach
%from Ruven Segev Dynamics book page 19
%a=dds*t+ds^2/rho*n
%n=(dr x (ddr x dr))/(speed *|ddr x dr|)
V=[v,zeros(N,1)]; %increase dimensions to allow for cross
A=[a,zeros(N,1)]; %increase dimensions to allow for cross
nominator=cross(V,cross(A,V,2),2);
denominator=speed.*vecnorm(cross(A,V,2),2,2);
n=nominator./denominator;
n=n(:,1:2); %reduce dimensions to mx2

rho=speed.^2./dot(a,n,2); %a_n=v^2/rho
omega=speed./rho; %omega*r=speed

Kinematics=handles.GuiStruct.Kinematics; %Obtain Kinematics struct to input too
Kinematics.Time=time'; %mx1 time stamps
Kinematics.Position=r; %mx1 position values [x1,x2]
Kinematics.Velocity=v; %mx2 velocity values [dx1,dx2]
Kinematics.Acceleration=a; %mx2 acceleraion values [ddx1,ddx2]
Kinematics.Theta=theta; %mx1 radians of orientation [atan2(x2,x1)
Kinematics.Omega=omega; %mx1 radial speed
Kinematics.NormalAcceleration=dot(a,n,2); %mx1
Kinematics.TangentAcceleration=dot(a,t,2); %mx1 
Kinematics.Tangent=t; %mx2 tangent unit vectors
Kinematics.Normal=n; %mx2 normal unit vectors
Kinematics.Rho=rho; %mx1 curvature radius

Nodes.Time=timeNodes';
Nodes.Position=NodesPosition;

%plot posotion on track
plot(Ax,r(:,1),r(:,2),'linewidth',2)

%Input Kinematics  and Nodes into into GuiStruct
handles.GuiStruct.Nodes=Nodes;
handles.GuiStruct.Kinematics=Kinematics;
guidata(hObject, handles); %save handles in figure
%mouse down callback
function Mdown(Ax,event,handles)
handles=guidata(Ax); %Obtain updated handles

if handles.PlaceNodesToggle.Value %Place cones
    Color=[0,0,0];
    P=Ax.CurrentPoint; %returns 2x3 matrix of [x,y,z;x,y,z] in [front;back] format
    scatter(Ax,P(1,1),P(1,2),40,Color,'filled','linew',1,...
        'markeredgecolor','k','UserData',Color);
end

guidata(Ax,handles); %save handles in figure
%simple callbacks
function PlaceNodesToggle_Callback(hObject, eventData, handles)
if hObject.Value == 1
    set(handles.Ax,'ButtonDownFcn',{@Mdown,handles})
    handles.Fig.Pointer='hand';
else
    set(handles.Ax,'ButtonDownFcn','')
    handles.Fig.Pointer='arrow';
end
function ClearPush_Callback(hObject, eventData, handles)
handles=guidata(hObject); %Obtain updated handles
cla(handles.Ax);
handles.GuiStruct.Kinematics=struct();
handles.GuiStruct.Nodes=struct();
guidata(hObject, handles); %save handles in figure
function LoadTrackPush_Callback(hObject, eventData, handles)
handles=guidata(hObject); %Obtain updated handles

%Initalize
Ax=handles.Ax;
Uinput=inputdlg('Please enter variable name','',1);
if isempty(Uinput), return, end
VarName=Uinput{1};
r=evalin('base',VarName); %must be a mx2 of [x1,x2] position
if size(r,2)~=2, errordlg('Loaded Variable of shape [x1,x2]'), return, end

%Clear axes
ClearPush_Callback(handles.ClearPush,[],handles)

%Draw cones and store in handles
axis(Ax,'auto'); %turn Ax limits to auto for drawing
%Draw nodes
Color=[0,0,0];
h=helpdlg('please hold, computing');
for k=1:size(r,1)
    scatter(Ax,r(k,1),r(k,2),40,Color,'filled','linew',1,...
        'markeredgecolor','k','UserData',Color);
end
delete(h);
AutoLimsCheckBox_Callback(handles.AutoLimsCheckBox,eventData,handles) %return Ax lims to what user decided

guidata(hObject, handles); %save handles in figure
function Export2WSPush_Callback(hObject, eventData, handles)
handles=guidata(hObject); %Obtain updated handles
Uinput=inputdlg({'Kinematics Variable Name','Nodes Variable Name'},'',1,{'Kinematics','Nodes'});
if isempty(Uinput), return, end
KinematicsName=Uinput{1}; assignin('base',KinematicsName,handles.GuiStruct.Kinematics);
NodesName=Uinput{2}; assignin('base',NodesName,handles.GuiStruct.Nodes);
%deadsimple callbacks
function PointerToggleToolBar_OnCallback(hObject, eventData, handles)
plotedit(handles.Fig,'on');
function PointerToggleToolBar_OffCallback(hObject, eventData, handles)
plotedit(handles.Fig,'off');
function AutoLimsCheckBox_Callback(hObject, eventData, handles)
Ax=handles.Ax;

if hObject.Value
    axis(Ax,'auto');
else
    axis(Ax,'manual');
end
function AxLimEdit_Callback(hObject, eventData, handles)
Ax=handles.Ax;
lims=str2num(hObject.String);
if isempty(lims), errordlg('Please input a correct limit input','A-lon'); return, end
if length(lims)~=4,  errordlg('Please input a correct limit input','A-lon'); return, end

Ax.XLim=lims(1:2); Ax.YLim=lims(3:4);
%% empty GUI functions
function AxLimEdit_CreateFcn(hObject, eventData, handles)
% hObject    handle to AxLimEdit (see GCBO)
% eventData  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function TimeEdit_CreateFcn(hObject, eventData, handles)
% hObject    handle to TimeEdit (see GCBO)
% eventData  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function IntrpAmntEdit_CreateFcn(hObject, eventData, handles)
% hObject    handle to IntrpAmntEdit (see GCBO)
% eventData  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function TimeEdit_Callback(hObject, eventData, handles)
function IntrpAmntEdit_Callback(hObject, eventData, handles)
function IntrpMethodPopUp_Callback(hObject, eventdata, handles)
function IntrpMethodPopUp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to IntrpMethodPopUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end