function varargout = TrackBuilder(varargin)
% TRACKBUILDER MATLAB code for TrackBuilder.fig
%      TRACKBUILDER, by itself, creates a new TRACKBUILDER or raises the existing
%      singleton*.
%
%      H = TRACKBUILDER returns the handle to a new TRACKBUILDER or the handle to
%      the existing singleton*.
%
%      TRACKBUILDER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in TRACKBUILDER.M with the given input arguments.
%
%      TRACKBUILDER('Property','Value',...) creates a new TRACKBUILDER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before TrackBuilder_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to TrackBuilder_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help TrackBuilder

% Last Modified by GUIDE v2.5 06-Apr-2020 16:11:56

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @TrackBuilder_OpeningFcn, ...
    'gui_OutputFcn',  @TrackBuilder_OutputFcn, ...
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
function TrackBuilder_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to TrackBuilder (see VARARGIN)

% Choose default command line output for TrackBuilder
handles.output = hObject;

%introduce default data
TrackSettings.XLim=[-300,300]; %[m]
TrackSettings.YLim=[-300,300]; %[m]
TrackSettings.TrackWidth=10; %[m]
TrackSettings.TrackInterpolationMultiplier=4;
handles.TrackSettings=TrackSettings;
handles.RaceTrack=RaceTrack; %empty handle class
handles.CarGeometrey=[];

%initailize axes
Ax=handles.Ax;
Ax.XLim=TrackSettings.XLim;
Ax.YLim=TrackSettings.YLim;
grid(Ax,'on'); hold(Ax,'on'); daspect(Ax,[1,1,1]);

% Update handles structure
guidata(hObject, handles);
function varargout = TrackBuilder_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
%% Callbacks
function BuildTrackToggle_Callback(hObject, eventdata, handles)
handles=guidata(hObject); %Obtain updated handles
TrackSettings=handles.TrackSettings;

if hObject.Value == 1 %build track is pressed
    set(handles.Ax,'ButtonDownFcn',{@Mdown,handles})
    handles.Fig.Pointer='hand';
    
    delete(handles.RaceTrack);
    handles.RaceTrack=RaceTrack; %reset the handle
    handles.RaceTrack.TrackWidth=TrackSettings.TrackWidth;
    
    Ax=handles.Ax;
    Ax.XLim=TrackSettings.XLim;
    Ax.YLim=TrackSettings.YLim;
    cla(Ax);
    grid(Ax,'on'); hold(Ax,'on'); daspect(Ax,[1,1,1]);
else
    set(handles.Ax,'ButtonDownFcn','')
    handles.Fig.Pointer='arrow';
    
    cla(handles.Ax);
    
    m=size(handles.RaceTrack.MidNodes,1);
    if m<2, return, end
    
    N=m*TrackSettings.TrackInterpolationMultiplier;
    handles.RaceTrack.PchipInterpolate(N)
    handles.RaceTrack.PlotTrack(handles.Ax);
end
guidata(hObject, handles);
function TrackSettingsPush_Callback(hObject, eventdata, handles)
handles=guidata(hObject); %Obtain updated handles

%Obtain user input
prompt={
    'x axes limits [m]',...
    'y axes limits [m]',...
    'Track width [m]',...
    'Track Interpolation Multiplier'};
definputs=cellfun(@num2str,struct2cell(handles.TrackSettings),'un',0);
Uinput=inputdlg(prompt,'Game Settings',1,definputs);
if isempty(Uinput), return, end

%Set Game Setings struct
TrackSettings.XLim=str2num(Uinput{1});
TrackSettings.YLim=str2num(Uinput{2});
TrackSettings.TrackWidth=str2num(Uinput{3});
TrackSettings.TrackInterpolationMultiplier=str2num(Uinput{4});
%set Game Settings in handles struct
handles.TrackSettings=TrackSettings;

%Set axes properties by settings
handles.Ax.XLim=TrackSettings.XLim;
handles.Ax.YLim=TrackSettings.YLim;

%redraw interpolated track and car
handles.RaceTrack.TrackWidth=TrackSettings.TrackWidth;
m=size(handles.RaceTrack.MidNodes,1);
if m>1
    cla(handles.Ax);
    N=TrackSettings.TrackInterpolationMultiplier*m;
    handles.RaceTrack.PchipInterpolate(N)
    handles.RaceTrack.PlotTrack(handles.Ax);
    
    if ~isempty(handles.CarGeometrey) %car is placed on track
    Width=handles.TrackSettings.TrackWidth/2;
    Length=Width*2;
    handles.CarGeometrey.Width=Width;
    handles.CarGeometrey.Length=Length;
    delete(handles.CarGeometrey.GraphicHandles);
    handles.CarGeometrey.DrawCar(handles.Ax);
    end
end

%save updated handles in figure
guidata(hObject, handles);
function SavePush_Callback(hObject, eventdata, handles)
handles=guidata(hObject); %Obtain updated handles
filter={'*.mat'};
[file,path]=uiputfile(filter); %file=[name,ext]
if ~ischar(file), return, end %user cancelled operation in ui
[~,name]=fileparts(file);
S.(name)=handles.RaceTrack; %create struct with field [name chosen by user]
save([path,file],'-struct','S') %save .mat file with variable named [name chosen by user]
function LoadPush_Callback(hObject, eventdata, handles)
filter={'*.mat'};
[file,path,Ind]=uigetfile(filter);
FileFullName=[path,file];
S=load(FileFullName); %S is a struct with fieldnames being the names of the variables in mat
cellS=struct2cell(S); %cell array containig struct fields (loses field names)
handles.RaceTrack=cellS{1}; %extract the racetrack variable

cla(handles.Ax);
N=handles.TrackSettings.TrackInterpolationMultiplier*size(handles.RaceTrack.MidNodes,1);
handles.RaceTrack.PchipInterpolate(N)
handles.RaceTrack.PlotTrack(handles.Ax);

guidata(hObject, handles);
if Ind==0, return, end %user cancelled
function WorkspacePush_Callback(hObject, eventdata, handles)
m=size(handles.RaceTrack.MidNodes,1);
if m<2, return, end
assignin('base','LeftCones',handles.RaceTrack.LeftCones);
assignin('base','RightCones',handles.RaceTrack.RightCones);

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
