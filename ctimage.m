function varargout = ctimage(varargin)
% CTIMAGE MATLAB code for ctimage.fig
%      CTIMAGE, by itself, creates a new CTIMAGE or raises the existing
%      singleton*.
%
%      H = CTIMAGE returns the handle to a new CTIMAGE or the handle to
%      the existing singleton*.
%
%      CTIMAGE('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CTIMAGE.M with the given input arguments.
%
%      CTIMAGE('Property','Value',...) creates a new CTIMAGE or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ctimage_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ctimage_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ctimage

% Last Modified by GUIDE v2.5 19-Apr-2018 19:11:08

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ctimage_OpeningFcn, ...
                   'gui_OutputFcn',  @ctimage_OutputFcn, ...
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
% End initialization code - DO NOT EDIT


% --- Executes just before ctimage is made visible.
function ctimage_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ctimage (see VARARGIN)

% Choose default command line output for ctimage
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ctimage wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = ctimage_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double





% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% show initial image
str = get(handles.edit1,'String');
num = xlsread(str,1);
axes(handles.axes1);
imshow(num);

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% calculate distance between two cameras
str = get(handles.edit1,'String');
num = xlsread(str,2);
% if the value is neither 0 nor 1, let it be 0 or 1
for i = 1 : 512
    for j = 1 : 180
        if num(i,j)>0
            num(i,j) = 1;
        else
        end
    end
end
% for each column, calculate the radius of circle 
% use the smaller circle to calculate the distance between two cameras
rad = zeros(1,180);
for j = 1:180
    time = 0;
    for i = 1:511
        if num(i,j) ~=num(i+1,j)
            time = time + 1;
        end
        if (num(i,j) ~= 0)&&(time<=2)
            rad(1,j) = rad(1,j) + 1;
        end
        if time == 3
            break;
        end
    end
    if time ==2
        rad(1,j) = 512;
    end
end
d = 8/min(rad);
set(handles.edit2,'string',d); 


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% refine the image
str = get(handles.edit1,'String');
num = xlsread(str,5);
k = 1024;
[m,n] = size(num);
initialrad = str2num(get(handles.edit3,'String'));

CT_fft = fft(num,k);
Ramp_lak = 2*[0:(k/2-1), k/2:-1:1]'/k;
CT_filtered = zeros(k,180);
for i = 1:180
    CT_filtered(:,i) = CT_fft(:,i).*Ramp_lak;
end
CT_ifft = real(ifft(CT_filtered));

CT_reim = zeros(m,m);
for i = 1:180
    for x= 1:m
        for y = 1:m
            t = round((x - m/2)*cos((-(initialrad+90)-i)*pi/180) - (y - m/2)*sin((-(initialrad+90)-i)*pi/180) + m/2);
            if (t>0) && (t<=m)
                CT_reim(x,y) = CT_reim(x,y) + CT_ifft(t,i);
            end
        end
    end
end

CT_reim = (CT_reim*pi)/180;
CT_ed = zeros(m,m);
x0 =  str2double(get(handles.edit5,'String'));
y0 =  str2double(get(handles.edit6,'String'));
x0 = x0/0.2764;
y0 = y0/0.2764;
for i = round(-x0+1):m
    for j = round(y0+1):m
        CT_ed(i+round(x0),j-round(y0)) = CT_reim(i,j);
    end
end

axes(handles.axes1);
imshow(CT_ed);



% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% get the initial angle
str = get(handles.edit1,'String');
num = xlsread(str,2);
[m,n] = size(num);
non_zero = zeros(n,1);
for i = 1:n
    for j = 1:m
        if num(j,i) ~= 0
            non_zero(i,1) = non_zero(i,1) + 1;
        end
    end
end

max_one = find(non_zero == max(non_zero));
initialrad = 90 - floor(mean(max_one));% y axes
set(handles.edit3,'string',initialrad);



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double


% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton6.
function pushbutton6_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% obtain errors of axes
str = get(handles.edit1,'String');
num = xlsread(str,2);
[R,F] = iradon(num,1,512);
bw = im2bw(R,0.1);
L = bwlabel(bw);
stats = regionprops(L,'centroid');
imshow(L);% show the wrong image
hold on

for i = 1:numel(stats)
    temp = stats(i).Centroid;
    x(i) = temp(1);
    y(i) = temp(2);
    plot(temp(1),temp(2),'b*');
end

% axes transfer
[m,n] = size(num);
x = x - m/2;
y = m/2 - y;
A = [x(1),y(1)];
B = [x(2),y(2)];
D12 = norm(A - B);
lameda = 45/D12;
a1 = lameda*(sqrt(x(1)^2 + y(1)^2));
b1 = lameda*(sqrt(x(2)^2 + y(2)^2));

% solve equations, get two points
syms u v
[u,v] = solve(u^2 + v^2 - a1^2,(u - 45)^2 + v^2 - b1^2);
solution = double(vpa([u,v],6));
x1 = solution(1,1);
y1 = solution(1,2);
x2 = solution(2,1);
y2 = solution(2,2);

% use the initial angle to choose the right answer
rad1 = atan2(y1,x1);
rad2 = atan2(y2,x2);
real_rad = 29+90*pi/180;
if abs(rad1 - real_rad)<abs(rad2 - real_rad)
    x0 = x1;
    y0 = y1;
else
    x0 = x2;
    y0 = y2;
end
set(handles.edit5,'string',x0);
set(handles.edit6,'string',y0);







function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double


% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double


% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
