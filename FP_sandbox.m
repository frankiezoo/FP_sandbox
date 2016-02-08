function varargout = FP_sandbox(varargin)
%FP_SANDBOX M-file for FP_sandbox.fig
%      FP_SANDBOX, by itself, creates a new FP_SANDBOX or raises the existing
%      singleton*.
%
%      H = FP_SANDBOX returns the handle to a new FP_SANDBOX or the handle to
%      the existing singleton*.
%
%      FP_SANDBOX('Property','Value',...) creates a new FP_SANDBOX using the
%      given property value pairs. Unrecognized properties are passed via
%      varargin to FP_sandbox_OpeningFcn.  This calling syntax produces a
%      warning when there is an existing singleton*.
%
%      FP_SANDBOX('CALLBACK') and FP_SANDBOX('CALLBACK',hObject,...) call the
%      local function named CALLBACK in FP_SANDBOX.M with the given input
%      arguments.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help FP_sandbox

% Last Modified by GUIDE v2.5 25-Oct-2015 14:37:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @FP_sandbox_OpeningFcn, ...
                   'gui_OutputFcn',  @FP_sandbox_OutputFcn, ...
                   'gui_LayoutFcn',  [], ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
   gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before FP_sandbox is made visible.
function FP_sandbox_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   unrecognized PropertyName/PropertyValue pairs from the
%            command line (see VARARGIN)

% Choose default command line output for FP_sandbox
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes FP_sandbox wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = FP_sandbox_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function FC_pos_x_Callback(hObject, eventdata, handles)
% hObject    handle to FC_pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FC_pos_x as text
%        str2double(get(hObject,'String')) returns contents of FC_pos_x as a double
pos_FC_x=str2double(get(hObject,'String'));
assignin('base', 'pos_FC_x', pos_FC_x)

% --- Executes during object creation, after setting all properties.
function FC_pos_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FC_pos_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FC_pos_y_Callback(hObject, eventdata, handles)
% hObject    handle to FC_pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FC_pos_y as text
%        str2double(get(hObject,'String')) returns contents of FC_pos_y as a double
pos_FC_y=str2double(get(hObject,'String'));
assignin('base', 'pos_FC_y', pos_FC_y)

% --- Executes during object creation, after setting all properties.
function FC_pos_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FC_pos_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FC_pos_z_Callback(hObject, eventdata, handles)
% hObject    handle to FC_pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FC_pos_z as text
%        str2double(get(hObject,'String')) returns contents of FC_pos_z as a double
pos_FC_z=str2double(get(hObject,'String'));
assignin('base', 'pos_FC_z', pos_FC_z)

% --- Executes during object creation, after setting all properties.
function FC_pos_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FC_pos_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FC_mu_x_Callback(hObject, eventdata, handles)
% hObject    handle to FC_mu_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FC_mu_x as text
%        str2double(get(hObject,'String')) returns contents of FC_mu_x as a double
mu_FC_x=str2double(get(hObject,'String'));
assignin('base', 'mu_FC_x', mu_FC_x)

% --- Executes during object creation, after setting all properties.
function FC_mu_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FC_mu_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FC_mu_y_Callback(hObject, eventdata, handles)
% hObject    handle to FC_mu_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FC_mu_y as text
%        str2double(get(hObject,'String')) returns contents of FC_mu_y as a double
mu_FC_y=str2double(get(hObject,'String'));
assignin('base', 'mu_FC_y', mu_FC_y)

% --- Executes during object creation, after setting all properties.
function FC_mu_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FC_mu_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function FC_mu_z_Callback(hObject, eventdata, handles)
% hObject    handle to FC_mu_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of FC_mu_z as text
%        str2double(get(hObject,'String')) returns contents of FC_mu_z as a double
mu_FC_z=str2double(get(hObject,'String'));
assignin('base', 'mu_FC_z', mu_FC_z)

% --- Executes during object creation, after setting all properties.
function FC_mu_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to FC_mu_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mag_pos0_x_Callback(hObject, eventdata, handles)
% hObject    handle to Mag_pos0_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mag_pos0_x as text
%        str2double(get(hObject,'String')) returns contents of Mag_pos0_x as a double
pos0_x=str2double(get(hObject,'String'));
assignin('base', 'pos0_x', pos0_x)

% --- Executes during object creation, after setting all properties.
function Mag_pos0_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mag_pos0_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mag_pos0_y_Callback(hObject, eventdata, handles)
% hObject    handle to Mag_pos0_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mag_pos0_y as text
%        str2double(get(hObject,'String')) returns contents of Mag_pos0_y as a double
pos0_y=str2double(get(hObject,'String'));
assignin('base', 'pos0_y', pos0_y)

% --- Executes during object creation, after setting all properties.
function Mag_pos0_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mag_pos0_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mag_pos0_z_Callback(hObject, eventdata, handles)
% hObject    handle to Mag_pos0_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mag_pos0_z as text
%        str2double(get(hObject,'String')) returns contents of Mag_pos0_z as a double
pos0_z=str2double(get(hObject,'String'));
assignin('base', 'pos0_z', pos0_z)

% --- Executes during object creation, after setting all properties.
function Mag_pos0_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mag_pos0_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mag_mu0_x_Callback(hObject, eventdata, handles)
% hObject    handle to Mag_mu0_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mag_mu0_x as text
%        str2double(get(hObject,'String')) returns contents of Mag_mu0_x as a double
mu0_x=str2double(get(hObject,'String'));
assignin('base', 'mu0_x', mu0_x)

% --- Executes during object creation, after setting all properties.
function Mag_mu0_x_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mag_mu0_x (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mag_mu0_y_Callback(hObject, eventdata, handles)
% hObject    handle to Mag_mu0_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mag_mu0_y as text
%        str2double(get(hObject,'String')) returns contents of Mag_mu0_y as a double
mu0_y=str2double(get(hObject,'String'));
assignin('base', 'mu0_y', mu0_y)

% --- Executes during object creation, after setting all properties.
function Mag_mu0_y_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mag_mu0_y (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Mag_mu0_z_Callback(hObject, eventdata, handles)
% hObject    handle to Mag_mu0_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Mag_mu0_z as text
%        str2double(get(hObject,'String')) returns contents of Mag_mu0_z as a double
mu0_z=str2double(get(hObject,'String'));
assignin('base', 'mu0_z', mu0_z)

% --- Executes during object creation, after setting all properties.
function Mag_mu0_z_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Mag_mu0_z (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function B_mag_Callback(hObject, eventdata, handles)
% hObject    handle to B_mag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of B_mag as text
%        str2double(get(hObject,'String')) returns contents of B_mag as a double
B_mag=str2double(get(hObject,'String'));
assignin('base', 'B_mag', B_mag)

% --- Executes during object creation, after setting all properties.
function B_mag_CreateFcn(hObject, eventdata, handles)
% hObject    handle to B_mag (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function damping_Callback(hObject, eventdata, handles)
% hObject    handle to damping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of damping as text
%        str2double(get(hObject,'String')) returns contents of damping as a double
damping=str2double(get(hObject,'String'));
assignin('base', 'damping', damping)

% --- Executes during object creation, after setting all properties.
function damping_CreateFcn(hObject, eventdata, handles)
% hObject    handle to damping (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function sim_time_Callback(hObject, eventdata, handles)
% hObject    handle to sim_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sim_time as text
%        str2double(get(hObject,'String')) returns contents of sim_time as a double
sim_time=str2double(get(hObject,'String'));
assignin('base', 'sim_time', sim_time)

% --- Executes during object creation, after setting all properties.
function sim_time_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sim_time (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Set_Parameters.
function Set_Parameters_Callback(hObject, eventdata, handles)
% hObject    handle to Set_Parameters (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
B_mag = evalin('base', 'B_mag');
mu_0=4*pi*10^-7; %N/A^2
damping = evalin('base', 'damping');
sim_time = evalin('base', 'sim_time');

mu0_x = evalin('base', 'mu0_x');
mu0_y = evalin('base', 'mu0_y');
mu0_z = evalin('base', 'mu0_z');
mu_mag_0=[mu0_x; mu0_y; mu0_z];
assignin('base', 'mu_mag_0', mu_mag_0)

pos0_x = evalin('base', 'pos0_x');
pos0_y = evalin('base', 'pos0_y');
pos0_z = evalin('base', 'pos0_z');
r_mag_0=[pos0_x; pos0_y; pos0_z];
assignin('base', 'r_mag_0', r_mag_0)

mu_FC_x = evalin('base', 'mu_FC_x');
mu_FC_y = evalin('base', 'mu_FC_y');
mu_FC_z = evalin('base', 'mu_FC_z');
mu_FC=[mu_FC_x; mu_FC_y; mu_FC_z];
assignin('base', 'mu_FC', mu_FC)

pos_FC_x = evalin('base', 'pos_FC_x');
pos_FC_y = evalin('base', 'pos_FC_y');
pos_FC_z = evalin('base', 'pos_FC_z');
r_FC=[pos_FC_x; pos_FC_y; pos_FC_z];
assignin('base', 'r_FC', r_FC)

sim('single_magnet_single_superconductor.slx')

assignin('base','r_m', r_m)
assignin('base','mu_m', mu_m)
assignin('base','r_f', r_f)
assignin('base','mu_f', mu_f)
assignin('base','r_mag', r_mag)
assignin('base','mu_mag', mu_mag)

assignin('base','F_m', F_m)
assignin('base','tau_m', tau_m)
assignin('base','F_f', F_f)
assignin('base','tau_f', tau_f)
assignin('base','U_f', U_f)
assignin('base','U_m', U_m)
assignin('base','TE', TE)



% --- Executes on button press in Plot_Images.
function Plot_Images_Callback(hObject, eventdata, handles)
% hObject    handle to Plot_Images (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.images);
r_mag_0 = evalin('base', 'r_mag_0');
r_m0 = evalin('base','r_m.data(1,:)');
r_f0 = evalin('base','r_f.data(1,:)');

mu_mag_0 = evalin('base','mu_mag_0');
mu_m0 = evalin('base','mu_m.data(1,:)');
mu_f0 = evalin('base','mu_f.data(1,:)');

mu0_x = [r_mag_0(1) r_mag_0(1)+0.01*mu_mag_0(1)/norm(mu_mag_0)];
mu0_y = [r_mag_0(2) r_mag_0(2)+0.01*mu_mag_0(2)/norm(mu_mag_0)];
mu0_z = [r_mag_0(3) r_mag_0(3)+0.01*mu_mag_0(3)/norm(mu_mag_0)];
mu_m0_x = [r_m0(1) r_m0(1)+0.01*mu_m0(1)/norm(mu_m0)];
mu_m0_y = [r_m0(2) r_m0(2)+0.01*mu_m0(2)/norm(mu_m0)];
mu_m0_z = [r_m0(3) r_m0(3)+0.01*mu_m0(3)/norm(mu_m0)];
mu_f0_x = [r_f0(1) r_f0(1)+0.01*mu_f0(1)/norm(mu_f0)];
mu_f0_y = [r_f0(2) r_f0(2)+0.01*mu_f0(2)/norm(mu_f0)];
mu_f0_z = [r_f0(3) r_f0(3)+0.01*mu_f0(3)/norm(mu_f0)];

scatter3(r_mag_0(1),r_mag_0(2),r_mag_0(3),300,[1 0 0],'filled')
hold on
scatter3(r_m0(1),r_m0(2),r_m0(3),300,[0 1 0],'filled')
scatter3(r_f0(1),r_f0(2),r_f0(3),300,[0 1 1],'filled')
line([0 .02],[0 0],[0 0],'LineWidth',2,'Color',[1 0 1])
line([0 0],[0 .02],[0 0],'LineWidth',2,'Color',[0 0 1])
line([0 0],[0 0],[0 .02],'LineWidth',2,'Color',[0 0 0])
line(mu0_x,mu0_y,mu0_z,'LineWidth',2,'Color',[1 0 0])
line(mu_m0_x,mu_m0_y,mu_m0_z,'LineWidth',2,'Color',[0 1 0])
line(mu_f0_x,mu_f0_y,mu_f0_z,'LineWidth',2,'Color',[0 1 1])
[X,Y] = meshgrid(-.1:.005:.1);
M=mesh(X,Y,zeros(length(X),length(Y)));
set(M,'facealpha',0)
set(M,'edgecolor',[.7 .7 .7])
axis(.5*[-.1 .1 -.1 .1 -.1 .1])
view(5,10);
h=legend('magnet','mobile image','frozen image','x','y','z','Location','southwest');
legend boxoff
psn=get(h,'Position');
psn(3:4)=0.50*psn(3:4);
set(h,'position',psn);
hold off


% --- Executes on button press in Plot_Ftau.
function Plot_Ftau_Callback(hObject, eventdata, handles)
% hObject    handle to Plot_Ftau (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
axes(handles.forces);

r_mag_0 = evalin('base', 'r_mag_0');
r_m0 = evalin('base','r_m.data(1,:)');
r_f0 = evalin('base','r_f.data(1,:)');

F_m0 = evalin('base','F_m.data(1,:)');
F_f0 = evalin('base','F_f.data(1,:)');

tau_m0 = evalin('base','tau_m.data(1,:)');
tau_f0 = evalin('base','tau_f.data(1,:)');

F_tot0=F_f0+F_m0;
tau_tot0=tau_f0+tau_m0;

scatter3(r_mag_0(1),r_mag_0(2),r_mag_0(3),100,[1 0 0],'filled')

F_m0_x = [r_mag_0(1) r_mag_0(1)+0.02*F_m0(1)/norm(F_m0)];
F_m0_y = [r_mag_0(2) r_mag_0(2)+0.02*F_m0(2)/norm(F_m0)];
F_m0_z = [r_mag_0(3) r_mag_0(3)+0.02*F_m0(3)/norm(F_m0)];
tau_m0_x = [r_mag_0(1) r_mag_0(1)+0.02*tau_m0(1)/norm(tau_m0)];
tau_m0_y = [r_mag_0(2) r_mag_0(2)+0.02*tau_m0(2)/norm(tau_m0)];
tau_m0_z = [r_mag_0(3) r_mag_0(3)+0.02*tau_m0(3)/norm(tau_m0)];
F_f0_x = [r_mag_0(1) r_mag_0(1)+0.02*F_f0(1)/norm(F_f0)];
F_f0_y = [r_mag_0(2) r_mag_0(2)+0.02*F_f0(2)/norm(F_f0)];
F_f0_z = [r_mag_0(3) r_mag_0(3)+0.02*F_f0(3)/norm(F_f0)];
tau_f0_x = [r_mag_0(1) r_mag_0(1)+0.02*tau_f0(1)/norm(tau_f0)];
tau_f0_y = [r_mag_0(2) r_mag_0(2)+0.02*tau_f0(2)/norm(tau_f0)];
tau_f0_z = [r_mag_0(3) r_mag_0(3)+0.02*tau_f0(3)/norm(tau_f0)];
F_tot0_x = [r_mag_0(1) r_mag_0(1)+0.02*F_tot0(1)/norm(F_tot0)];
F_tot0_y = [r_mag_0(2) r_mag_0(2)+0.02*F_tot0(2)/norm(F_tot0)];
F_tot0_z = [r_mag_0(3) r_mag_0(3)+0.02*F_tot0(3)/norm(F_tot0)];
tau_tot0_x = [r_mag_0(1) r_mag_0(1)+0.02*tau_tot0(1)/norm(tau_tot0)];
tau_tot0_y = [r_mag_0(2) r_mag_0(2)+0.02*tau_tot0(2)/norm(tau_tot0)];
tau_tot0_z = [r_mag_0(3) r_mag_0(3)+0.02*tau_tot0(3)/norm(tau_tot0)];

hold on
scatter3(r_m0(1),r_m0(2),r_m0(3),100,[0 1 0],'filled')
scatter3(r_f0(1),r_f0(2),r_f0(3),100,[0 1 1],'filled')
line(F_m0_x,F_m0_y,F_m0_z,'LineWidth',2,'Color',[0 1 0])
line(F_f0_x,F_f0_y,F_f0_z,'LineWidth',2,'Color',[0 1 1])
line(F_tot0_x,F_tot0_y,F_tot0_z,'LineWidth',2,'Color',[1 0 0])
title('forces on magnet')
axis(.5*[-.1 .1 -.1 .1 -.1 .1])
view(5,10);
hold off

axes(handles.torques);
scatter3(r_mag_0(1),r_mag_0(2),r_mag_0(3),100,[1 0 0],'filled')
hold on
scatter3(r_m0(1),r_m0(2),r_m0(3),100,[0 1 0],'filled')
scatter3(r_f0(1),r_f0(2),r_f0(3),100,[0 1 1],'filled')
line(tau_m0_x,tau_m0_y,tau_m0_z,'LineWidth',2,'Color',[0 1 0])
line(tau_f0_x,tau_f0_y,tau_f0_z,'LineWidth',2,'Color',[0 1 1])
line(tau_tot0_x,tau_tot0_y,tau_tot0_z,'LineWidth',2,'Color',[1 0 0])
title('torques on magnet')
axis(.5*[-.1 .1 -.1 .1 -.1 .1])
view(5,10);
hold off

% --- Executes on button press in Dynamics_Simulation.
function Dynamics_Simulation_Callback(hObject, eventdata, handles)
% hObject    handle to Dynamics_Simulation (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
r_mag = evalin('base','r_mag');
r_m = evalin('base','r_m');
r_f = evalin('base','r_f');

mu_mag = evalin('base','mu_mag');
mu_m = evalin('base','mu_m');
mu_f = evalin('base','mu_f');

F_m = evalin('base','F_m');
F_f = evalin('base','F_f');

tau_m = evalin('base','tau_m');
tau_f = evalin('base','tau_f');

mu_x = zeros(size(r_mag.data,1),2);
mu_y = zeros(size(r_mag.data,1),2);
mu_z = zeros(size(r_mag.data,1),2);
mu_m_x = zeros(size(r_mag.data,1),2);
mu_m_y = zeros(size(r_mag.data,1),2);
mu_m_z = zeros(size(r_mag.data,1),2);
mu_f_x = zeros(size(r_mag.data,1),2);
mu_f_y = zeros(size(r_mag.data,1),2);
mu_f_z = zeros(size(r_mag.data,1),2);
F_m_x = zeros(size(r_mag.data,1),2);
F_m_y = zeros(size(r_mag.data,1),2);
F_m_z = zeros(size(r_mag.data,1),2);
tau_m_x = zeros(size(r_mag.data,1),2);
tau_m_y = zeros(size(r_mag.data,1),2);
tau_m_z = zeros(size(r_mag.data,1),2);
F_f_x = zeros(size(r_mag.data,1),2);
F_f_y = zeros(size(r_mag.data,1),2);
F_f_z = zeros(size(r_mag.data,1),2);
tau_f_x = zeros(size(r_mag.data,1),2);
tau_f_y = zeros(size(r_mag.data,1),2);
tau_f_z = zeros(size(r_mag.data,1),2);
F_tot_x = zeros(size(r_mag.data,1),2);
F_tot_y = zeros(size(r_mag.data,1),2);
F_tot_z = zeros(size(r_mag.data,1),2);
tau_tot_x = zeros(size(r_mag.data,1),2);
tau_tot_y = zeros(size(r_mag.data,1),2);
tau_tot_z = zeros(size(r_mag.data,1),2);

F_tot=F_f.data+F_m.data;
tau_tot=tau_f.data+tau_m.data;

for i = 1:size(r_mag.data,1)
    mu_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.01*mu_mag.data(i,1)/norm(mu_mag.data(i,:))];
    mu_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.01*mu_mag.data(i,2)/norm(mu_mag.data(i,:))];
    mu_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.01*mu_mag.data(i,3)/norm(mu_mag.data(i,:))];
    mu_m_x(i,:) = [r_m.data(i,1) r_m.data(i,1)+0.01*mu_m.data(i,1)/norm(mu_m.data(i,:))];
    mu_m_y(i,:) = [r_m.data(i,2) r_m.data(i,2)+0.01*mu_m.data(i,2)/norm(mu_m.data(i,:))];
    mu_m_z(i,:) = [r_m.data(i,3) r_m.data(i,3)+0.01*mu_m.data(i,3)/norm(mu_m.data(i,:))];
    mu_f_x(i,:) = [r_f.data(i,1) r_f.data(i,1)+0.01*mu_f.data(i,1)/norm(mu_f.data(i,:))];
    mu_f_y(i,:) = [r_f.data(i,2) r_f.data(i,2)+0.01*mu_f.data(i,2)/norm(mu_f.data(i,:))];
    mu_f_z(i,:) = [r_f.data(i,3) r_f.data(i,3)+0.01*mu_f.data(i,3)/norm(mu_f.data(i,:))];
    F_m_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*F_m.data(i,1)/norm(F_m.data(i,:))];
    F_m_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*F_m.data(i,2)/norm(F_m.data(i,:))];
    F_m_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*F_m.data(i,3)/norm(F_m.data(i,:))];
    tau_m_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*tau_m.data(i,1)/norm(tau_m.data(i,:))];
    tau_m_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*tau_m.data(i,2)/norm(tau_m.data(i,:))];
    tau_m_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*tau_m.data(i,3)/norm(tau_m.data(i,:))];
    F_f_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*F_f.data(i,1)/norm(F_f.data(i,:))];
    F_f_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*F_f.data(i,2)/norm(F_f.data(i,:))];
    F_f_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*F_f.data(i,3)/norm(F_f.data(i,:))];
    tau_f_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*tau_f.data(i,1)/norm(tau_f.data(i,:))];
    tau_f_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*tau_f.data(i,2)/norm(tau_f.data(i,:))];
    tau_f_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*tau_f.data(i,3)/norm(tau_f.data(i,:))];
    F_tot_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*F_tot(i,1)/norm(F_tot(i,:))];
    F_tot_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*F_tot(i,2)/norm(F_tot(i,:))];
    F_tot_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*F_tot(i,3)/norm(F_tot(i,:))];
    tau_tot_x(i,:) = [r_mag.data(i,1) r_mag.data(i,1)+0.02*tau_tot(i,1)/norm(tau_tot(i,:))];
    tau_tot_y(i,:) = [r_mag.data(i,2) r_mag.data(i,2)+0.02*tau_tot(i,2)/norm(tau_tot(i,:))];
    tau_tot_z(i,:) = [r_mag.data(i,3) r_mag.data(i,3)+0.02*tau_tot(i,3)/norm(tau_tot(i,:))];
end

for time = 1:size(r_mag.data,1)
% animate the magnet and images
axes(handles.images);
    scatter3(r_mag.data(time,1),r_mag.data(time,2),r_mag.data(time,3),...
        300,[1 0 0],'filled');
    hold on
    scatter3(r_m.data(time,1),r_m.data(time,2),r_m.data(time,3),...
        300,[0 1 0],'filled');
    scatter3(r_f.data(time,1),r_f.data(time,2),r_f.data(time,3),...
        300,[0 1 1],'filled');
    line([0 .02],[0 0],[0 0],'LineWidth',2,'Color',[1 0 1])
    line([0 0],[0 .02],[0 0],'LineWidth',2,'Color',[0 0 1])
    line([0 0],[0 0],[0 .02],'LineWidth',2,'Color',[0 0 0])
    line(mu_x(time,:),mu_y(time,:),mu_z(time,:),'LineWidth',2,'Color',[1 0 0])
    line(mu_m_x(time,:),mu_m_y(time,:),mu_m_z(time,:),'LineWidth',2,'Color',[0 1 0])
    line(mu_f_x(time,:),mu_f_y(time,:),mu_f_z(time,:),'LineWidth',2,'Color',[0 1 1])
    axis(.2*[-.1 .1 -.1 .1 -.1 .1])
    title('Images of magnet')
    hold off    
    drawnow
% animate the magnet and forces
axes(handles.forces);
    scatter3(0,0,0,300,[1 0 0],'filled');
    hold on
    line([0 F_tot(time,1)],[0 F_tot(time,2)],[0 F_tot(time,3)],'LineWidth',2,'Color',[1 0 0])
    line([0 F_m.data(time,1)],[0 F_m.data(time,2)],[0 F_m.data(time,3)],'LineWidth',2,'Color',[0 1 0])
    line([0 F_f.data(time,1)],[0 F_f.data(time,2)],[0 F_f.data(time,3)],'LineWidth',2,'Color',[0 1 1])
    axis(.1*[-.1 .1 -.1 .1 -.1 .1])
    title('Forces on magnet')
    hold off
    drawnow
% animate the magnet and forces
axes(handles.torques);
    scatter3(0,0,0,300,[1 0 0],'filled');
    hold on
    line([0 tau_tot(time,1)],[0 tau_tot(time,2)],[0 tau_tot(time,3)],'LineWidth',2,'Color',[1 0 0])
    line([0 tau_m.data(time,1)],[0 tau_m.data(time,2)],[0 tau_m.data(time,3)],'LineWidth',2,'Color',[0 1 0])
    line([0 tau_f.data(time,1)],[0 tau_f.data(time,2)],[0 tau_f.data(time,3)],'LineWidth',2,'Color',[0 1 1])
    axis(.1*[-.1 .1 -.1 .1 -.1 .1])
    title('Torques on magnet')
    hold off
    drawnow
end

% --- Executes on button press in Generate_Plots.
function Generate_Plots_Callback(hObject, eventdata, handles)
% hObject    handle to Generate_Plots (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

U_m = evalin('base','U_m');
U_f = evalin('base','U_f');
TE = evalin('base','TE');
r_mag = evalin('base','r_mag');
r_f = evalin('base','r_f');
r_m = evalin('base','r_m');
F_f = evalin('base','F_f');
F_m = evalin('base','F_m');
tau_f = evalin('base','tau_f');
tau_m = evalin('base','tau_m');

figure
plot(TE)
hold on
plot(U_m,'g')
plot(U_f,'c')
plot(TE.time,TE.data+U_m.data+U_f.data,'k')
xlabel('time [sec]')
ylabel('Energy [J]')
legend('kinetic','mobile','frozen','total')
title('Energy vs time of Flux Pinned system')

figure
subplot(3,1,1)
plot(r_mag.time,r_mag.data(:,1),'m')
xlabel('time [sec]')
ylabel('x position [m]')
title('Permanent magnet positions')
subplot(3,1,2)
plot(r_mag.time,r_mag.data(:,2))
xlabel('time [sec]')
ylabel('y position [m]')
subplot(3,1,3)
plot(r_mag.time,r_mag.data(:,3),'k')
xlabel('time [sec]')
ylabel('z position [m]')

figure
subplot(2,1,1)
plot(F_m.time,F_f.data+F_m.data)
xlabel('time [sec]')
title('force of images on magnet [N]')
subplot(2,1,2)
plot(tau_m.time,tau_f.data+tau_m.data)
xlabel('time [sec]')
title('torque of images on magnet [N-m]')