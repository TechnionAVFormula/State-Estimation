function varargout = SpinSim(varargin)
% SPINSIM MATLAB code for SpinSim.fig
%      SPINSIM, by itself, creates a new SPINSIM or raises the existing
%      singleton*.
%
%      H = SPINSIM returns the handle to a new SPINSIM or the handle to
%      the existing singleton*.
%
%      SPINSIM('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SPINSIM.M with the given input arguments.
%
%      SPINSIM('Property','Value',...) creates a new SPINSIM or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SpinSim_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SpinSim_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SpinSim

% Last Modified by GUIDE v2.5 20-Sep-2019 18:19:45

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @SpinSim_OpeningFcn, ...
    'gui_OutputFcn',  @SpinSim_OutputFcn, ...
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
function SpinSim_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SpinSim (see VARARGIN)

% Choose default command line output for SpinSim
handles.output = hObject;

%introduce default data
GameSettings.dt=0.001; %[s]
GameSettings.XLim=[0,100]; %[m]
GameSettings.YLim=[0,100]; %[m]
GameSettings.TrackWidth=5; %[m]
GameSettings.TrackInterpolationMultiplier=2;
handles.GameSettings=GameSettings;
handles.RaceTrack=RaceTrack; %empty handle class
handles.CarGeometrey=[];

% Update handles structure
guidata(hObject, handles);
function varargout = SpinSim_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

%% Game Callbacks
function BuildTrackToggle_Callback(hObject, eventdata, handles)
handles=guidata(hObject); %Obtain updated handles
GameSettings=handles.GameSettings;

if hObject.Value == 1 %build track is pressed
    set(handles.Ax,'ButtonDownFcn',{@Mdown,handles})
    handles.Fig.Pointer='hand';
    
    delete(handles.RaceTrack);
    handles.RaceTrack=RaceTrack; %reset the handle
    handles.RaceTrack.TrackWidth=GameSettings.TrackWidth;
    
    Ax=handles.Ax;
    Ax.XLim=GameSettings.XLim;
    Ax.YLim=GameSettings.YLim;
    cla(Ax);
    grid(Ax,'on'); hold(Ax,'on'); daspect(Ax,[1,1,1]);
else
    set(handles.Ax,'ButtonDownFcn','')
    handles.Fig.Pointer='arrow';
    
    cla(handles.Ax);
    N=GameSettings.TrackInterpolationMultiplier*size(handles.RaceTrack.MidNodes,1);
    handles.RaceTrack.PchipInterpolate(N)
    handles.RaceTrack.PlotTrack(handles.Ax);
end
guidata(hObject, handles);
function PlaceCarPush_Callback(hObject, eventdata, handles)

handles=guidata(hObject); %Obtain updated handles

RT=handles.RaceTrack;
m=size(RT.MidNodes,1);
if m<2, error('Track must be built before a car is placed'); end

if ~isempty(handles.CarGeometrey) %car was drawn before
    if isvalid(handles.CarGeometrey)
        delete(handles.CarGeometrey.GraphicHandles)
    end
end

CG=RT.MidNodes(1,:);
t=RT.MidIntrp(2,:)-RT.MidIntrp(1,:);
Theta=atan2(t(2),t(1));
Width=handles.GameSettings.TrackWidth/2;
Length=Width*2;
handles.CarGeometrey=CarGeometrey(CG,Theta,Width,Length);
handles.CarGeometrey.DrawCar(handles.Ax);

%save updated handles in figure
guidata(hObject, handles);
function GameSettingsPush_Callback(hObject, eventdata, handles)
handles=guidata(hObject); %Obtain updated handles

%Obtain user input
prompt={'time step [s]',...
    'x axes limits [m]',...
    'y axes limits [m]',...
    'Track width [m]',...
    'Track Interpolation Multiplier'};
definputs=cellfun(@num2str,struct2cell(handles.GameSettings),'un',0);
Uinput=inputdlg(prompt,'Game Settings',1,definputs);
if isempty(Uinput), return, end

%Set Game Setings struct
GameSettings.dt=str2num(Uinput{1});
GameSettings.XLim=str2num(Uinput{2});
GameSettings.YLim=str2num(Uinput{3});
GameSettings.TrackWidth=str2num(Uinput{4});
GameSettings.TrackInterpolationMultiplier=str2num(Uinput{5});
%set Game Settings in handles struct
handles.GameSettings=GameSettings;

%Set axes properties by settings
handles.Ax.XLim=GameSettings.XLim;
handles.Ax.YLim=GameSettings.YLim;

%redraw interpolated track and car
handles.RaceTrack.TrackWidth=GameSettings.TrackWidth;
m=size(handles.RaceTrack.MidNodes,1);
if m>1
    cla(handles.Ax);
    N=GameSettings.TrackInterpolationMultiplier*m;
    handles.RaceTrack.PchipInterpolate(N)
    handles.RaceTrack.PlotTrack(handles.Ax);
    
    if ~isempty(handles.CarGeometrey) %car is placed on track
    Width=handles.GameSettings.TrackWidth/2;
    Length=Width*2;
    handles.CarGeometrey.Width=Width;
    handles.CarGeometrey.Length=Length;
    delete(handles.CarGeometrey.GraphicHandles);
    handles.CarGeometrey.DrawCar(handles.Ax);
    end
end

%save updated handles in figure
guidata(hObject, handles);
function GOpush_Callback(hObject, eventdata, handles)
handles=guidata(hObject); %Obtain updated handles

MidIntrp=handles.RaceTrack.MidIntrp;
CarGeo=handles.CarGeometrey;
for ii=2:size(MidIntrp,1)
    delete(CarGeo.GraphicHandles);
    CarGeo.CG=MidIntrp(ii,:);
    t=MidIntrp(ii,:)-MidIntrp(ii-1,:);
    Theta=atan2(t(2),t(1));
    CarGeo.Theta=Theta;
    CarGeo.DrawCar(handles.Ax);
    pause(0.1);
end

function Mdown(Ax,event,handles)
handles=guidata(Ax); %Obtain updated handles
if handles.BuildTrackToggle.Value %Place cones
    P=Ax.CurrentPoint; %returns 2x3 matrix of [x,y,z;x,y,z] in [front;back] format
    P=[P(1,1),P(1,2)];
    RT=handles.RaceTrack;
    RT.AddNode(P);
    h=handles.RaceTrack.PlotSection_Linear(handles.Ax);
    if size(handles.RaceTrack.MidNodes,1)==1 %if first node
        set(h,'ButtonDownFcn',{@CloseLoopCallback,handles});
    end
end
function CloseLoopCallback(PlotHandle,event,handles)
RT=handles.RaceTrack;
RT.LoopClose;
RT.PlotSection_Linear(handles.Ax);
plot(handles.Ax,RT.MidNodes(1,1),RT.MidNodes(1,2),'color',[1,0,0],...
    'marker','o','markersize',10);
function LoadTrackMenu_Callback(hObject, eventdata, handles)
filter={'*.mat'};
[file,path,Ind]=uigetfile(filter);
FileFullName=[path,file];
S=load(FileFullName); %S is a struct with fieldnames being the names of the variables in mat
cellS=struct2cell(S); %cell array containig struct fields (loses field names)
handles.RaceTrack=cellS{1}; %extract the racetrack variable

cla(handles.Ax);
N=handles.GameSettings.TrackInterpolationMultiplier*size(handles.RaceTrack.MidNodes,1);
handles.RaceTrack.PchipInterpolate(N)
handles.RaceTrack.PlotTrack(handles.Ax);

guidata(hObject, handles);
if Ind==0, return, end %user cancelled
function SaveTrackMenu_Callback(hObject, eventdata, handles)
handles=guidata(hObject); %Obtain updated handles
filter={'*.mat'};
[file,path]=uiputfile(filter); %file=[name,ext]
if ~ischar(file), return, end %user cancelled operation in ui
[~,name]=fileparts(file);
S.(name)=handles.RaceTrack; %create struct with field [name chosen by user]
save([path,file],'-struct','S') %save .mat file with variable named [name chosen by user]

%% Simulation Callbacks
function PlaySimulationPush_Callback(hObject, eventdata, handles)
%% functions

%% general callbacks
function PointerToggle_OffCallback(hObject, eventdata, handles)
% hObject    handle to PointerToggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
plotedit(handles.Fig,'off');
function PointerToggle_OnCallback(hObject, eventdata, handles)
% hObject    handle to PointerToggle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
plotedit(handles.Fig,'on');

%% Unimportant Callbacks
function FileMenu_Callback(hObject, eventdata, handles)
function GameMenu_Callback(hObject, eventdata, handles)
function KalmanFilterMethodPopUp_Callback(hObject, eventdata, handles)
function KalmanFilterMethodPopUp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to KalmanFilterMethodPopUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function VehicleModelPopUp_Callback(hObject, eventdata, handles)
function VehicleModelPopUp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to VehicleModelPopUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
function ControllerPopUp_Callback(hObject, eventdata, handles)
function ControllerPopUp_CreateFcn(hObject, eventdata, handles)
% hObject    handle to ControllerPopUp (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
