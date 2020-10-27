function varargout = ConvolucionEJEMPLOS(varargin)
% CONVOLUCIONEJEMPLOS MATLAB code for ConvolucionEJEMPLOS.fig
%      CONVOLUCIONEJEMPLOS, by itself, creates a new CONVOLUCIONEJEMPLOS or raises the existing
%      singleton*.
%
%      H = CONVOLUCIONEJEMPLOS returns the handle to a new CONVOLUCIONEJEMPLOS or the handle to
%      the existing singleton*.
%
%      CONVOLUCIONEJEMPLOS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in CONVOLUCIONEJEMPLOS.M with the given input arguments.
%
%      CONVOLUCIONEJEMPLOS('Property','Value',...) creates a new CONVOLUCIONEJEMPLOS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before ConvolucionEJEMPLOS_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to ConvolucionEJEMPLOS_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help ConvolucionEJEMPLOS

% Last Modified by GUIDE v2.5 26-Oct-2020 17:02:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @ConvolucionEJEMPLOS_OpeningFcn, ...
                   'gui_OutputFcn',  @ConvolucionEJEMPLOS_OutputFcn, ...
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
end
% --- Executes just before ConvolucionEJEMPLOS is made visible.
function ConvolucionEJEMPLOS_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to ConvolucionEJEMPLOS (see VARARGIN)

% Choose default command line output for ConvolucionEJEMPLOS
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes ConvolucionEJEMPLOS wait for user response (see UIRESUME)
% uiwait(handles.figure1);
strings = {'Seleccione Filtro ','Desenfoque','Enfoque','Realce Bordes','Repujado','Deteccion de bordes','Filtro tipo sobel','Filtro tipo Sharpen','Filtro Norte','Filtro Este','Filtro tipo Gauss'};
handles.dropdown.String = strings;
 set(handles.limpiar,'Visible','off');
 axes(handles.axes8);
handles.imagen=imread('logo.png');
imagesc(handles.imagen)
axis off;
end

% --- Outputs from this function are returned to the command line.
function varargout = ConvolucionEJEMPLOS_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

end
% --- Executes on selection change in dropdown.
function dropdown_Callback(hObject, eventdata, handles)
% hObject    handle to dropdown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns dropdown contents as cell array
%        contents{get(hObject,'Value')} returns selected item from dropdown

end
% --- Executes during object creation, after setting all properties.
function dropdown_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dropdown (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end

% --- Executes on button press in seleccioneImagen.
function seleccioneImagen_Callback(hObject, eventdata, handles)
% hObject    handle to seleccioneImagen (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.limpiar,'Visible','on');
valor=get(handles.dropdown, 'value');
if(valor==1)
    f = warndlg('Seleccione un filtro','Alerta');
    return;
end

archivo=uigetfile('*.jpg','Abrir imagen');



imagenbuena=imread(fullfile(archivo));
axes(handles.axes1);
imagesc(imagenbuena);



switch(valor)
   
   case 2
     k = [1 1 1;1 1 1; 1 1 1]; % Desenfoque
    
   case 3
     k = [0 -1 0;-1 5 -1; 0 -1 0]; % Enfoque
    
   case 4
    k = [0 0 0; -1 1 0; 0 0 0]; % Realce Bordes
  
   case 5
     k = [-2 -1 0; -1 1 1; 0 1 2] % Repujado
   
   case 6
     k = [0 1 0; 1 -4 1; 0 1 0] % Deteccion de bordes
   
   case 7
        k = [-1 0 1; -2 0 2;-1 0 1]; % Filtro tipo sobel
   
       
   case 8
      k = [1 -2 1; -2 5 -2; 1 -2 1]; % Filtro tipo Sharpen
 
    
   case 9
       k = [1 1 1; 1 -2 1; -1 -1 -1]; % Filtro Norte
    
   case 10
       k = [-1 1 1; -1 -2 1; -1 1 1]; % Filtro Este
    
   case 11
       k = [1 2 3 1 1; 2 7 11 7 2; 3 11 17 11 3; 2 7 11 7 2; 1 2 3 2 1]; % Filtro tipo Gauss
end

tamano = size(k);
c=0;
for i=1:tamano(1)
    for j=1:tamano(2)
        c=c+k(i,j);
    end
end
if(c==0)
    c=1;
end
binario1 = 0;
binario2 = 0;
binario3 = 0;

    Imrojo=imagenbuena(:,:,1);
    axes(handles.axes2);
    imshow(Imrojo);
    axis off;
    binario1= double(imagenbuena(:,:,1));
    
    %binario=im2bw(Imrojo);
    %imshow(Imrojo); imtool(Imrojo);

    %imshow(Imverde); imtool(Imverde);
    Imverde=imagenbuena(:,:,2);
    axes(handles.axes5);
    imshow(Imverde);
    binario2= double(imagenbuena(:,:,2));
   
    %binario=im2bw(Imverde);
    axis off;

    %imshow(Imazul); imtool(Imazul);
    Imazul=imagenbuena(:,:,3);
    axes(handles.axes3);
    imshow(Imazul);
    binario3= double(imagenbuena(:,:,3));
    %binario=im2bw(Imazul);
    axis off;
    
    matrizAux1=0;
    matrizAux1=double(padarray(binario1,[1 1]));
    s1=size(matrizAux1);
    result=aplicarFiltro(k,s1,c,matrizAux1);
    axes(handles.axes6);
    imshow(uint8(result));
    
    matrizAux2=0;
    matrizAux2=double(padarray(binario2,[1 1]));
    s2=size(matrizAux2);
    result2=aplicarFiltro(k,s2,c,matrizAux2);
    axes(handles.axes4);
    imshow(uint8(result2));
    
    matrizAux3=0;
    matrizAux3=double(padarray(binario3,[1 1]));
    s3=size(matrizAux3);
    result3=aplicarFiltro(k,s3,c,matrizAux3);
    axes(handles.axes7);
    imshow(uint8(result3));
end
function y= aplicarFiltro(k,s,c, matrizAux)

    result=matrizAux*0;

    for i=2: s(1)-1
        for j=2: s(2)-1
            ventana=matrizAux(i-1:i+1, j-1:j+1);
            prod=ventana .* k;
            pix=(1/c)*sum(sum(prod));
            result(i,j)=pix;
        end
    end
    result(s(1),:)=[];
    result(:,s(2))=[];
    result(1,:)=[];
    result(:,1)=[];

    y=result;
end
function y = convertirVector(matrizResul)
    matrizIndex=random('Normal',0,1,200,200);
    matrizBin=im2bw(matrizResul);
    v=times(matrizIndex,matrizBin);

    for i=1:length(v)
        aux=0;
        aux2=0;
        for j=1:length(v)
            aux=aux+v(i,j);
            aux2=aux2+ v(j,i);
            sumFil(i)=aux;
            sumCol(i)=aux2;
        end
    end 

    aux=1;
    for i=1:1:(length(sumFil)+length(sumCol))
        if i<=length(sumFil)
         y(i)=sumFil(i);
        else
         y(i)=sumCol(aux);
         aux=aux+1;
        end
    end
    y=im2bw(y);   
end


% --- Executes on button press in limpiar.
function limpiar_Callback(hObject, eventdata, handles)
% hObject    handle to limpiar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
    set(handles.limpiar,'Visible','off');
    cla(handles.axes1);
    cla(handles.axes2);
    cla(handles.axes3);
    cla(handles.axes4);
    cla(handles.axes5);
    cla(handles.axes6);
    cla(handles.axes7);
    set(handles.seleccioneImagen,'Visible','on');
end
