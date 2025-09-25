function varargout = GUI(varargin)
% GUI MATLAB code for GUI.fig
%      GUI, by itself, creates a new GUI or raises the existing
%      singleton*.
%
%      H = GUI returns the handle to a new GUI or the handle to
%      the existing singleton*.
%
%      GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI.M with the given input arguments.
%
%      GUI('Property','Value',...) creates a new GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help GUI

% Last Modified by GUIDE v2.5 11-Sep-2024 08:06:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @GUI_OutputFcn, ...
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


% --- Executes just before GUI is made visible.
function GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to GUI (see VARARGIN)

% Choose default command line output for GUI
set(handles.edit3,'String','15');
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1



% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



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
global k_Th
k_Th =  str2double(get(handles.edit3, 'String'));
global gray_img
gray_img = handles.gray_img;
global mask



    try
        menu1 = get(handles.popupmenu1, 'Enable');
        menu2 = get(handles.popupmenu2, 'Enable');
        menu3 = get(handles.popupmenu3, 'Enable');
        menu4 = get(handles.popupmenu4, 'Enable');
        menu5 = get(handles.popupmenu5, 'Enable');
        menu6 = get(handles.popupmenu7, 'Enable');
        if strcmp(menu1, 'on')
            var = get(handles.popupmenu1,'Value');
            switch var
                % 最大均值滤波
                case 1 % 选中第二行                       
                    % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\Maximum_mean_filtering');
                    tic
                    I=gray_img;
                    width=5;
                    xwidth=(width-1)/2;

                    %I = double(I);
                    [m n]=size(I);
                    I2=I;
                    z=zeros(5,width);
                    tem=1;
                    for i=1+xwidth:1:m-xwidth    
                        for j=1+xwidth:1:n-xwidth                
                            for k=-xwidth:1:xwidth            
                                z(1,tem)=I(i,j+k);            
                                z(2,tem)=I(i+k,j);            
                                z(3,tem)=I(i-k,j+k);            
                                z(4,tem)=I(i+k,j+k);            
                                tem=tem+1;        
                            end
                            %最大均值求解
                            k1=mean(z(1,:)); % k1=mean([I(i,j-1),I(i,j),I(i,j+1)]);     
                            k2=mean(z(2,:)); % k2=mean([I(i-1,j),I(i,j),I(i+1,j)]); 
                            k3=mean(z(3,:)); % k3=mean([I(i+1,j-1),I(i,j),I(i-1,j+1)]);
                            k4=mean(z(4,:)); % k4=mean([I(i-1,j-1),I(i,j),I(i+1,j+1)]);  
                            tem=1;
                            I2(i,j)=max([k1,k2,k3,k4]); 
                        end
                    end
                    Ct = double((I-I2)*10);

                  %% 阈值分割
                    mask = Threshold_segmentation(Ct,k_Th);
                    
                  %% show results
                    axes(handles.axes2)
                    mesh(Ct); 
                    axes(handles.axes3)
                    imshow(Ct,[]);
                    axes(handles.axes4)
                    imshow(mask);
                    

                    
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                    
                     % 移除文件夹到 MATLAB 的路径中
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\Maximum_mean_filtering');
                % 最大中值滤波
                case 2 % 选中第二行
                    x=1;
                % TDLMS 
                case 3 % 选中第二行
                    x=1;
            end
        elseif strcmp(menu2, 'on')
            var = get(handles.popupmenu2,'Value');
            switch var
                % New Top_Hat
                case 1                        % 选中第一行
                     % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\New_top_hat');               
                    tic
                    img = gray_img;
                    R_o = 9;
                    R_i = 4;
                    delta_B = newRingStrel(R_o, R_i);
                    B_b = ones(R_i);
                    Ct = MNWTH(img, delta_B, B_b);
                    
                  %% 阈值分割
                    mask = Threshold_segmentation(Ct,k_Th);
                    
                  %% show results
                    axes(handles.axes2)
                    mesh(Ct); 
                    axes(handles.axes3)
                    imshow(Ct);
                    axes(handles.axes4)
                    imshow(mask);
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                     % 移除文件夹到 MATLAB 的路径中
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\New_top_hat');
                % Top_Hat
                case 2 % 选中第二行
                    x=1;
            end
        elseif strcmp(menu3, 'on')
            var = get(handles.popupmenu3,'Value');
            switch var
                %MDWCM
                case 1                        % 选中第一行
                     % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\MDWCM');
                    tic
                    Ct = main_ours(gray_img);
                    
                  %% 阈值分割
                    mask = Threshold_segmentation(Ct,k_Th);
                    
                  %% show results
                    axes(handles.axes2)
                    mesh(Ct);
                    axes(handles.axes3)
                    imshow(Ct,[]);
                    axes(handles.axes4)
                    imshow(mask);
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                     % 移除文件夹到 MATLAB 的路径中
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\MDWCM');
                % AADWCDD
                case 2 % 选中第二行
                    x=1;
            end
        elseif strcmp(menu4, 'on')
            var = get(handles.popupmenu4,'Value');
            switch var
                % IPI
                case 1                        % 选中第一行
                    % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\IPI'); 
                    tic
                    I=gray_img;
                    opt.dw = 50;
                    opt.dh = 50;
                    opt.x_step = 10;
                    opt.y_step = 10;

                    [A, Ct] = winRPCA_median(I, opt);
                    maxv = max(max(double(I)));
                    A = uint8( mat2gray(A) .* maxv );
                    Ct = double(uint8( mat2gray(Ct) .* 255 ));
                  %% 阈值分割
                    mask = Threshold_segmentation(Ct,k_Th);
                  %% show results
                    axes(handles.axes2)
                    imshow(A); 
                    axes(handles.axes3)
                    imshow(Ct,[]); 
                    axes(handles.axes4)
                    imshow(mask); 
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                     % 移除文件夹到 MATLAB 的路径中
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\IPI');
                % WIPI
                case 2 % 选中第二行
                    x=1;
            end
        elseif strcmp(menu5, 'on')
            var = get(handles.popupmenu5,'Value');
            switch var
                % IDPGSM+DWELCM
                case 1                        % 选中第1行
                    % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\my_method');
                    tic
                    %DOG滤波
                    sigma1=2;
                    sigma2=8;
                    window=5;
                    H1=fspecial('gaussian', window, sigma1);
                    H2=fspecial('gaussian', window, sigma2);
                    % 作高斯差分
                    DiffGauss=H1-H2;
                    srcImg_DOD = imfilter(gray_img,DiffGauss,'replicate');   %对任意类型数组或多维图像进行滤波
                    srcImg_DOD = mat2gray(srcImg_DOD);

                    %% Directly produce the final detection results, ignoring the intermediate process.
                    % if using this block of code, you can comment out the next block of code.
                    % detWay = DensityPeaksIR();
                    % [tarPos, tarCon] = detWay.finalDetect(srcImg);   
                    %% Details of the detection method are presented here.
                    detWay = DensityPeaksIR();
                    rhoMat = srcImg_DOD;
                    m = size(rhoMat, 1);
                    [rho, delta] = iterationElection( detWay, rhoMat );
                    [ classInitial ] = singularFind( detWay, rho, delta );
                    singularIndex = find( classInitial ~=  0 );
                    classCenterRows = mod( singularIndex, m );
                    classCenterRows(classCenterRows == 0) = m;
                    classCenterCols = ceil( singularIndex / m );

                    % %候选种子点坐标位置，参与运算的子块9×9
                    classCenterA = [classCenterRows,classCenterCols];
                    count=0;
                    for I=1:size(classCenterRows)
                        if (classCenterRows(I)>17) && (classCenterCols(I)>17) && (classCenterRows(I)<size(rhoMat, 1)-17) && (classCenterCols(I)<size(rhoMat, 2)-17)
                            %ClassCenterA(I,:) = [];
                            count = count + 1;
                        end
                    end

                    classCenterB = zeros(count , 2);
                    count = 0;

                    for I=1:size(classCenterRows)
                        if (classCenterRows(I)>17) && (classCenterCols(I)>17) && (classCenterRows(I)<size(rhoMat, 1)-17) && (classCenterCols(I)<size(rhoMat, 2)-17)
                            count = count + 1;
                            classCenterB(count,1)=classCenterRows(I);
                            classCenterB(count,2)=classCenterCols(I);
                        end
                    end
                    % size(rhoMat, 1)
                    % size(rhoMat, 2)
                    % classCenterA
                    % classCenterB
                    srcContrast = zeros(size(gray_img));
                    for I=1:size(classCenterB,1)
                        for J=1:81
                            patch_In = gray_img((classCenterB(I,1)-17+mod(J,9)):(classCenterB(I,1)+9+mod(J,9)),(classCenterB(I,2)-17+floor(J/9)):(classCenterB(I,2)+9+floor(J/9)));
                            %patch_In = ordfilt2(patch_In,5,ones(3,3));%中值滤波
                            C_n = IRLCM_computation(patch_In);
                            srcContrast(classCenterB(I,1)+mod(J,9),classCenterB(I,2)+floor(J/9)) = C_n(14,14);
                        end
                    end

                    [row,col] = size(srcContrast);
                    srcContrast_modified = zeros(row,col);
                    target_pixel_num = 0;   % 统计小目标在mask中占据的像元数
                    for i = 1:row
                        for j = 1:col
                            if srcContrast(i,j)>=0
                                srcContrast_modified(i,j)=srcContrast(i,j);
                                target_pixel_num = target_pixel_num+1;
                            else
                                srcContrast_modified(i,j) = 0;
                            end
                        end
                    end

                    axes(handles.axes2)
                    mesh(srcContrast_modified)
                    axes(handles.axes3)
                    imshow(srcContrast_modified,[]); 
                  %% 阈值分割
                    mask = Threshold_segmentation(srcContrast_modified,k_Th);
                    
                    axes(handles.axes4)
                    imshow(mask); 
                   
                    
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                    


%                     axes(handles.axes2)
%                     Candidate_targets = gray_img;
%                     imshow(Candidate_targets,[0,255]); 
%                     hold on;
%                     plot(classCenterB(:,2), classCenterB(:,1), 'LineStyle', 'none', ...
%                         'LineWidth', 1, 'Color', 'g', 'Marker', 's', 'MarkerSize', 9 );
                    % 移除文件夹到 MATLAB 的路径中
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\my_method');
                
                % DensityPeaksIR
                case 2                        % 选中第2行
                    % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\DensityPeaksIR');
                    tic
                    srcImg = double(gray_img);      
                    
                 %% Directly produce the final detection results, ignoring the intermediate process.
                    % if using this block of code, you can comment out the next block of code.
                    % detWay = DensityPeaksIR();
                    % [tarPos, tarCon] = detWay.finalDetect(srcImg);

                    %% Details of the detection method are presented here.
                    detWay = DensityPeaksIR();
                    rhoMat = srcImg;
                    m = size(rhoMat, 1);
                    [rho, delta] = iterationElection( detWay, rhoMat );
                    [ classInitial ] = singularFind( detWay, rho, delta );
                    singularIndex = find( classInitial ~=  0 );
                    classCenterRows = mod( singularIndex, m );
                    classCenterRows(classCenterRows == 0) = m;
                    classCenterCols = ceil( singularIndex / m );
                    seedPos = [ classCenterCols, classCenterRows ];
                    gvr = regionGrow( detWay, rhoMat, seedPos );

                    % Threashold operation
                    confidence = confidenceCal( detWay, gvr );
                    posIndex = confidence > detWay.thdQuatile;
                    tarPos = seedPos(posIndex, :);
                    tarCon = confidence(posIndex);

%                     figure
%                     plot(rho, delta, 'LineStyle', 'none', ...
%                        'Color', 'k', 'Marker', '.', 'MarkerSize', 16 ); hold on;
%                     plot(rho(singularIndex), delta(singularIndex), 'LineStyle', 'none', ...
%                        'Color', 'b', 'Marker', '.', 'MarkerSize', 24 );
%                     grid on;
%                     set(gca,'FontSize',16,'GridLineStyle',':','GridColor','k','GridAlpha',1);
%                     xlabel('\rho','FontSize',20); ylabel('\delta','FontSize',20);
%                     title('\rho-\delta space');

                    axes(handles.axes4)
                    imshow(gray_img)
%                     axes(handles.axes3)
%                     imshow(srcImg); 
                    hold on;
                    plot(classCenterCols, classCenterRows, 'LineStyle', 'none', ...
                        'LineWidth', 1, 'Color', 'b', 'Marker', 's', 'MarkerSize', 9 );

                    hold on;
                    plot( tarPos(:, 1), tarPos(:, 2), 'LineStyle', 'none', ...
                       'LineWidth', 1, 'Color', 'r', 'Marker', 's', 'MarkerSize', 9 );
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                    toc %记录程序完成时间
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\DensityPeaksIR');
              
              %% Chain growth
                case 3                        % 选中第3行
                    % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\Chain growth');
                    tic
                    %% 通过引导滤波对原始图像进行滤波，消除噪声
                    % srcImg = imguidedfilter(srcImg);

                    %% 通过区域生长算法获得显著图
                    p0 = PreFilterDirctgrowth_original;  %PreFilterDirctgrowth   PreFilterDirctgrowth_original
                    dstImg0 = p0.process(gray_img);
                    %%
                    k = 30; 
                    result = Adaptive_threshold_segmentation(dstImg0,k);

                    %% 显示每一步算法的计算结果图
                    axes(handles.axes2)
                    mesh(dstImg0)
                    axes(handles.axes3)
                    imshow(result)
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\Chain growth');
                
              %% MPCM
                case 4                        % 选中第4行
                    tic
                    % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\MPCM');
                    I=gray_img;
                    I = imnoise(I ,'gaussian',0, 0.001);
                    I = double(I);
                    Out = MPCM_fun(I);
                  %% 阈值分割
                    mask = Threshold_segmentation(Out,k_Th);
                  %% 显示输入和输出
                    axes(handles.axes2)
                    mesh(Out)
                    axes(handles.axes3)
                    imshow(Out,[])
                    axes(handles.axes4)
                    imshow(mask)
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\MPCM');
              
              %% LCM
                case 5                        % 选中第5行
                    x=1;
              %% RLCM
                case 6                        % 选中第6行
                    x=1;
              %% NLCM
                case 7                        % 选中第7行
                    x=1;
              %% TLLCM
                case 8                        % 选中第8行
                    x=1;
              %% DoG
                case 9                        % 选中第9行
                    x=1;
            end
        elseif strcmp(menu6, 'on')
            var = get(handles.popupmenu7,'Value');
            switch var
              %% low_filter
                case 1                        % 选中第一行
                    tic
                     % 添加文件夹到 MATLAB 的路径中
                    addpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\Frequency domain method');
                    img = gray_img;    
                    % 对图像进行傅里叶变换
                    fft_image = fftshift(fft2(double(img)));
                    % 将图像与低通滤波器相乘
                    [M, N] = size(fft_image);
                    centerX = round(M/2);
                    centerY = round(N/2);
                    radius = 10; % 设置滤波器半径
                    lowpass_filter = zeros(M, N);
                    for i = 1:M
                        for j = 1:N
                            if sqrt((i-centerX)^2 + (j-centerY)^2) <= radius
                                lowpass_filter(i,j) = 1;
                            end
                        end
                    end
                    % 将图像与低通滤波器相乘
                    filtered_image = fft_image .* lowpass_filter;
                    % 对滤波后的图像进行逆傅里叶变换
                    ifft_image = real(ifft2(ifftshift(filtered_image)));
                    % 原图与滤波图做差，得到目标显著图
                    Out = double(gray_img)-double(ifft_image);
                  %% 阈值分割
                    mask = Threshold_segmentation(Out,k_Th);
                  %% 显示输入和输出
                    axes(handles.axes2)
                    mesh(Out)
                    axes(handles.axes3)
                    imshow(Out,[])
                    axes(handles.axes4)
                    imshow(mask)
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
                    rmpath('D:\AcademicResearch\03Doctor\03Code\Model-driven IR detection method\08-自创算法\基于模型驱动算法集合的GUI系统\Frequency domain method');
                
              %% high_filter
                case 2                        % 选中第一行
                    tic
                    img = gray_img;  
                    % 将图像转换为双精度数据类型
                    double_image = im2double(gray_img);

                    % 对图像进行傅里叶变换
                    fft_image = fftshift(fft2(double_image));

                    % 定义高通滤波器
                    [M, N] = size(fft_image);
                    center_x = round(M/2);
                    center_y = round(N/2);
                    cutoff_frequency = 5;  % 频率截断阈值
                    highpass_filter = ones(M, N);
                    highpass_filter(center_x-cutoff_frequency:center_x+cutoff_frequency, center_y-cutoff_frequency:center_y+cutoff_frequency) = 0;
                    filtered_image1 = ifftshift(fft_image .* highpass_filter);
                    % 应用高通滤波器
                    Out = ifft2(filtered_image1);
                  %% 阈值分割
%                     k_Th=15;
                    mask = Out;
                    avg = mean(mean(mask));
                    normal = std(std(mask));
                    th = avg + k_Th*normal;
                    [row,col] = size(mask);
                    for i = 1:row
                        for j = 1:col
                            if mask(i,j)>th
                               mask(i,j)=255;
                            else
                               mask(i,j)=0;
                            end
                        end
                    end

                  %% 显示输入和输出
                    axes(handles.axes2)
                    mesh(Out)
                    axes(handles.axes3)
                    imshow(mask)
                    toc
                    time = round(toc,2);
                    set(handles.edit1,'String',sprintf(num2str(time)))
                    % 使用八连通算法标记连通区域
                    labeledImage = bwlabel(mask, 8);
                    % 统计连通区域的数量
                    numObjects = max(labeledImage(:));
                    % 打印连通区域的数量
                    set(handles.edit2,'String',sprintf(num2str(numObjects)))
            end

        else
            errordlg(Detection_algorithm, 'Please Open');
        end
     catch exception
        % 弹出错误提示
        errordlg(exception.message, 'Please Open');
    end
    handles.mask=mask;
    guidata(hObject, handles)  


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
clc,clear,close all;


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global gray_img
global srcImg

[filename, pathname]=uigetfile({'*.png';'*.jpg';'*.bmp';},'Select Image');
str = [pathname,filename];
srcImg = imread(str);
% 将RGB图像转换为灰度图像
% 判断图像类型
if size(srcImg, 3) == 3
    % disp('输入图像为RGB图像');
    % 将RGB图像转换为灰度图像
    gray_img = rgb2gray(srcImg);
else
    % disp('输入图像为灰度图像');
    gray_img = srcImg;
end
axes(handles.axes1)
imshow(gray_img)
handles.gray_img=gray_img;
guidata(hObject, handles)

% --- Executes on button press in pushbutton4.
function pushbutton4_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
[FileName,PathName] = uiputfile({'*.png','PNG(*.png)';...
                                 '*.jpg','JPEG(*.jpg)';...
                                 '*.bmp','Bitmap(*.bmp)';...
                                 '*.gif','GIF(*.gif)';...
                                 '*.*',  'All Files (*.*)'},...
                                 'Save Picture','Untitled');
if FileName==0
    return;
else
%     h=getframe(handles.axes4);
%     imwrite(h.cdata,[PathName,FileName]);
    mask = handles.mask;
    imwrite(mask,[PathName,FileName]);
end



% --- Executes on button press in pushbutton5.
function pushbutton5_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
open('instructions.pdf');



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, ~, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


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


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4



% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5


% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu7


% --- Executes during object creation, after setting all properties.
function popupmenu7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1
if get(handles.radiobutton1, 'Value') == 1
    set(handles.popupmenu1, 'Enable', 'on'); % 激活文本框

else
    set(handles.popupmenu1, 'Enable', 'off'); % 禁用文本框
end

% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2
if get(handles.radiobutton2, 'Value') == 1
    set(handles.popupmenu2, 'Enable', 'on'); % 激活文本框
else
    set(handles.popupmenu2, 'Enable', 'off'); % 禁用文本框
end

% --- Executes on button press in radiobutton3.
function radiobutton3_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton3
if get(handles.radiobutton3, 'Value') == 1
    set(handles.popupmenu3, 'Enable', 'on'); % 激活文本框
else
    set(handles.popupmenu3, 'Enable', 'off'); % 禁用文本框
end


% --- Executes on button press in radiobutton4.
function radiobutton4_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton4
if get(handles.radiobutton4, 'Value') == 1
    set(handles.popupmenu4, 'Enable', 'on'); % 激活文本框
else
    set(handles.popupmenu4, 'Enable', 'off'); % 禁用文本框
end



% --- Executes on button press in radiobutton5.
function radiobutton5_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton5
if get(handles.radiobutton5, 'Value') == 1
    set(handles.popupmenu5, 'Enable', 'on'); % 激活文本框
else
    set(handles.popupmenu5, 'Enable', 'off'); % 禁用文本框
end


% --- Executes on button press in radiobutton6.
function radiobutton6_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton6
if get(handles.radiobutton6, 'Value') == 1
    set(handles.popupmenu7, 'Enable', 'on'); % 激活文本框
else
    set(handles.popupmenu7, 'Enable', 'off'); % 禁用文本框
end


% --- Executes during object creation, after setting all properties.
function axes4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes4


% --- Executes during object creation, after setting all properties.
function axes3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes3


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2


% --- Executes during object creation, after setting all properties.
function axes1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes1



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
