# LiZIhan
Lane line detection system made by MATLAB
clear all
clc

% % #Camera Matrix
% K = [[  1.15422732e+03   0.00000000e+00   6.71627794e+02]
% 
%  [  0.00000000e+00   1.14818221e+03   3.86046312e+02]
% 
%  [  0.00000000e+00   0.00000000e+00   1.00000000e+00]]
% 
% 
% 
% % #Distortion Coefficients
% dist = [[ -2.42565104e-01  -4.77893070e-02
%   -1.31388084e-03  -8.79107779e-05
%     2.20573263e-02]]

%视频读取

vid = VideoReader('carVid.avi');%Read video into var vid
vidObj = VideoWriter('myVideo.avi');%创建一个VideoWriter对象写入myVideo.avi视频中
writer.FrameRate=vid.FrameRate;
open(vidObj);  %打开改文件以进行写入*********************************************************************************************************************
% NewVid = VideoWriter('OUT');%Writin the new video %写出新的处理完成的输出视频
% NewVid.FrameRate = 25;     %每秒钟处理的图像帧数是25帧
% 
% open(NewVid);%open the new video  打开新的处理过后的视频
firstRun = 0;
fucked = 0;
prev = 0;%for previous values recording  纪录先前的处理结果
Direction = 'LEFT';

for n=1:50 %hasFrame(vid) %Use 20 for a good curve, imshow(New)

frame = readFrame(vid);%Read a specific frame of the video as an image将视频的特定帧读取为图片1
redChannel = frame(:, :, 1);
imshow(frame)  %*******************************************************************************************************************************************

%% GAUSSIAN FILTER高斯滤波器，去除杂质部分
G = fspecial('gaussian', [5,5],2); %生成一个5×5的高斯核,标准差为2
I = imfilter(frame,G,'same');%gaussian filter 使用高斯核G对原图frame进行滤波。'same'表示输出图像大小和输入图像相同。
I = rgb2gray(frame);%change to gray scale
 imshow(I);     

%% EDGE DETECTION边缘检测
edgeD = edge(I,'Roberts');%edge detection   Roberts Operator是最简单的差分算子,用于检测图像中的强边缘。它通过计算两个相邻像素点的灰度差值来检测边缘。
[rows columns] = size(edgeD);%get dimensions of image 
 imshow(edgeD)                %********************************************************************************************************************************

%% GET RID OF TOP HALF OF IMAGE 去掉图像的上半部分
imArea = edgeD;%new var to start specifying area of interest
for i = 1:rows
    for j = 1:columns
        if (i<426)%if top half of image, no road there  判断当前点是否在上半图像区域,根据样本图像大小,将上半部分定义为前426行
            imArea(i,j) = 0;% get rid of any white pixels, dont care about that part of image如果是,则将当前点置为0,即黑色
        end
    end
end
imshow(imArea) %遍历完成后,上半图像区域所有的像素都被置为黑色,实现了去除上半图像的效果
%去除上半图像部分的作用是减少后续处理的数据量和干扰。由于车道线出现在图像下半部分区域,上半图像部分对后续车道线检测算法没有实际贡献,所以可以安全去除

%Clean up image, get rid of smaller pixel areas 清理图像，去掉像素较小的区域
imAreaClean = bwareaopen(imArea,50); %bwareaopen(imArea,50)函数对二值图像imArea进行面积开操作,去除面积小于50的连通区域
imshow(imAreaClean)   %*************************************************************************************************************

         
%Get rid of pixels that arent high on the red scale, since lines are white
%去掉红色刻度上不高的像素
%and yellow 提高车道线区域的相对比例,利于后续检测。由于去除了红色通道值较低的区域,车道线对应的高值区域所占比例会相应提高。这可以减少后续检测算法考虑的输入数据量,提高检测效率和精度
colorThresh = imAreaClean; %定义colorThresh作为颜色阈值分割结果图像。初始化为清理后的图像imAreaClean
indexR = (redChannel < 100); %找出红色通道值小于100的索引
colorThresh(indexR) = 0; %将这些像素置零
imshow(colorThresh)  

%Region props it 区域支持
hold on
bad = [];  %初始化bad为一个空数组
check = regionprops(colorThresh,'BoundingBox','Orientation');  %计算二值图像colorThresh中的每个连通区域的包围框(BoundingBox)和方向(Orientation)
for k = 1 : length(check)
  BB = check(k).BoundingBox;
  if(check(k).Orientation < 30 && check(k).Orientation > -20)%Check if they are horizonal ish遍历每个区域,判断其方向Orientation是否在-20到30度之间,如果是,则判断为非车道线区域
  rectangle('Position', BB,...
  'EdgeColor','r','LineWidth',2 )    %使用rectangle函数绘制被判断为非车道线区域的包围框,便于观察
   bad = [bad ; k];%Keep track of which regions are bad regions, too horizontalbad  存储被判断为非车道线的区域索引
 
  end    
end


%Make all pixels black in the bad regions 在不合适的区域设置所有像素为黑色，这段代码实现了在先前判断为非车道线区域的基础上,将这些区域全部置为黑色
noHor = colorThresh;
for a = 1:length(bad)
    BB = int16(check(bad(a)).BoundingBox); %遍历bad数组中所有的非车道线区域索引
    for c = BB(1):BB(1)+BB(3)
        for r = BB(2):BB(2)+BB(4)
          noHor(r,c) = 0; %将遍历到的每个像素置为黑色,即0
        end
    end
end
imshow(noHor)           %*************************************************************************************************************

%ISOLATE noHOR IMAGE TWO ONLY KEEP LANES, NO CARS OR NOTHING 这段代码实现了通过行列索引对noHor图像进行进一步分割,仅保留感兴趣区域
for a = 1:columns
    for b = 1:rows
        if (b < 510 && a > 856 || a < 250)
            noHor(b,a) = 0;
        end
    end
end
            

%CUT OFF NOHOR IMAGE AT A Y LEVEL WE FIND TO BE
%APROPRIATE在我们认为合适的水平线上截断任何图像
for a = 1:rows
    for b = 1:columns
        if (b <450) 
            noHor(b,a) = 0;
        end
    end
end


%Get rid of small pixel groups, nosise and dilate, then fill
se1 = strel('disk',3);
New = bwareaopen(noHor,20); %这段代码实现了对noHor图像进行进一步处理,包括去除小区域、膨胀、填充空洞等操作
New = imdilate(New,se1);
New = imfill(New,'holes'); %对bwareaopen结果New进行膨胀操作,使用se1作为结构元素。膨胀操作可以连接临近目标区域,填平目标区域内的小空洞,为后续填充空洞做准备
%对膨胀结果New进行内部空洞填充。使用'holes'表示填充内部空洞。
imshow(New)       %*************************************************************************************************************
   hold on    %*************************************************************************************************************

 %Get regions of image, should mostly be lanes这段代码实现了对填充后的图像New进行区域属性分析
check2 = regionprops(New,'BoundingBox','Orientation','Eccentricity');
for k = 1 : length(check2)
  BB = check2(k).BoundingBox;
  %if(check2(k).Orientation < 30 && check(k).Orientation > -20)%Check if they are horizonal ish
  %rectangle('Position', BB,...
  %'EdgeColor','r','LineWidth',2 )
  check2(k).Orientation;
  Ecc = check2(k).Eccentricity;
  BB;
    %end    
end





%% HOUGH
%Make new totally black image 将非黑色图像设置为黑色，达到黑白二值图像的效果
houghIm = noHor;             %将全黑图像赋值给houghIm
index = (redChannel > -1);   %查找redChannel通道值大于-1的像素索引
houghIm(index) = 0;          %将redChannel通道值大于-1的像素点设置为0，即变为黑色

%Hough Transform
[H,T,R] = hough(noHor); % get transformatio H, theta T, rho R 使用霍夫变换检测直线，获得霍夫空间映射H和对应直线的极角T和距离rho R

%Hough Peaks
P  = houghpeaks(H ,10,'threshold',ceil(0.1*max(H(:)))); %在H矩阵中查找峰值,也就是直线,10指定返回的峰值数目
%P  = houghpeaks(H,10,'threshold', 0);
x = T(P(:,2)); y = R(P(:,1)); %从P中提取对应直线的极角x和距离y,也就是直线方程的参数。
plot(x,y,'s','color','white');   %使用plot函数根据x和y绘制直线

%Hough lines, plot them
lines = houghlines(noHor,T,R,P,'MinLength',15); %使用houghlines函数从noHor图像和直线参数(T、R和P)中提取最长长度不小于15的直线
%figure
imshow(noHor), hold on %what figure to plot lines  显示绘制成功的图像
%over*********************************************************************************************
max_len = 0;
lineTrack = []; %存储所有直线的端点坐标xy
posSlope = [];  %存储正斜率直线的端点坐标xy
negSlope = [];  %存储负斜率直线的端点坐标xy 
Pslope = [];  %两个空数组，用于储存后续的政府斜率直线信息
Nslope = [];  %
for k = 1:length(lines) %遍历所有检测到的直线 ，储存政正负斜率的信息
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','red');
   if( abs(xy(1,2)-xy(2,2)) > 2)%no stray horizontals, check endpoint y difference如果两个端点在y方向的差值大于2，则保留直线
   lineTrack = [lineTrack; xy];%Keep track of xy endpoints of each line, each line gives 2x2
   
  
   
     %Lines always listed in increasing x, so check y values to find if
   %positive or negative slope, add to group起点的y坐标减去重点的y坐标
     if (xy(1,2)>xy(2,2) && xy(1,1)<xy(2,1))%pos slope  起点的x坐标和重点的x坐标之间差值
       posSlope = [posSlope ; xy];
   end
   
   if (xy(1,2)<xy(2,2) && xy(1,1)<xy(2,1))%neg slope
       negSlope = [negSlope ; xy];
   end
   end
 
%    % Plot beginnings and ends of lines
%    plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
%    plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
end

 for i = 1:2:length(lineTrack)  %对所有的直线进行分类，左车道线和右车道线分别储存在不同的数组中
       if (lineTrack(i,1) < 630)%if left lane line看车道线的起点x坐标是否小于630，以此来划分车道线的左侧和右侧
           Pslope = [Pslope ; [lineTrack(i,1) lineTrack(i,2)] ; [lineTrack(i+1,1) lineTrack(i+1,2)]];
        end
           
        
       if (lineTrack(i,1) > 630)  %车道线起点的x坐标如果大于630，该直线被划分到右边直线中去
           Nslope = [Nslope ; [lineTrack(i,1) lineTrack(i,2)]; [lineTrack(i+1,1) lineTrack(i+1,2)]];   
       end
 end

%Using Pslope and N slope, find min and max x,y values to plot line (which
%will be extrapolated)
minX = min(Pslope(:,1));% 从Pslope中提取所有左车道线起点和终点x坐标的值,取最小值,存储在minX中
maxX= max(Pslope(:,1));% 从Pslope中提取所有左车道线起点和终点x坐标的值,取最大值,存储在maxX中
minY = min(Pslope(:,2));%y
maxY= max(Pslope(:,2));%y

minXN = min(Nslope(:,1));%右侧车道线的值
maxXN= max(Nslope(:,1));
minYN = min(Nslope(:,2));
maxYN= max(Nslope(:,2));

%REcord previous values for next run
if (firstRun == 1)   %将本次车道线储存起来，作为上一帧的参考
    minXprev = minX;%将本次左车道线x方向最小值minX存储在minXprev中,作为上一帧的minX
    maxXprev = maxX;
    minYprev = minY;
    maxYprev = maxY;
    

    prev = 1;%将prev置为1，表示已经至少运行过一次
end


% PLOTTING 1. 获取当前图像并存储
          %2. 创建一个全黑图像,用于存储外推的车道线
%Set new image since we plotted on top of an image
%This  will be extrapolated/ interpolated lines from the ones in Lanes
frameGrab = getframe(gca); %获取当前图像并储存在frameGrab中
Lanes = frame2im(frameGrab); %提取图像数据，存在Lanes中
%figure
LaneExtrap = Lanes; %创建LaneExtrap并将Lanes中的图像数据复制到LaneExtrap,LaneExtrap是一个全黑图像
%Want an all black image to start with. We are only extrapolating the
%largest line for each slope, so we dont want the other smaller lines in
%the picture
for q = 1:columns
    for w = 1:rows
              LaneExtrap(w,q) = 0;%Make all black image创建可一个只包含外推结果的图像
     end
end 
imshow(LaneExtrap)       %*************************************************************************************************************
hold on

%Find the largest line in pos slope and neg slope, only want to extrapolate
%one line for each
%Pos左边直线
maxLine = 0; %找到最长线段
for p = 1:2:size(posSlope) %取Pslope中储存的每两行，对应每条支线的两个端点坐标each line is 2x2, endpoints (x1,y1);(x2,y2) So cycle through by two each time
x1 = posSlope(p,1);
x2 = posSlope(p+1,1);
y1 = posSlope(p,2);
y2 = posSlope(p+1,2);     %提取两点坐标
lineDist = ((y2-y1)^2 +(x2-x1)^2)^.5;%get the length of the line计算两点间的坐标长度

if (lineDist > maxLine)%如果新的长度为最大值，则保留新端点if the length of the line is the new max, save endpoints
    end_pointsX = [posSlope(p,1) posSlope(p+1,1)];
    end_pointsY = [posSlope(p,2) posSlope(p+1,2)];
end
end

%Neg右边直线
maxLine = 0;  
for p = 1:2:size(negSlope)%each line is 2x2, endpoints (x1,y1);(x2,y2) So cycle through by two each time
x1 = negSlope(p,1);
x2 = negSlope(p+1,1);
y1 = negSlope(p,2);
y2 = negSlope(p+1,2);
lineDist = ((y2-y1)^2 +(x2-x1)^2)^.5;%get the length of the line

if (lineDist > maxLine)%if the length of the line is the new max, save endpoints
    end_pointsXneg = [negSlope(p,1) negSlope(p+1,1)];
    end_pointsYneg = [negSlope(p,2) negSlope(p+1,2)];
end
end



%Plot the positive longest line extrapolated/interpolated
X = end_pointsX;
Y = end_pointsY;
%NEW X AND Y METHOD WITH Pslope INSTEAD OF posSlope
X = [minX maxX]; %定义左车道线的起点和终点 x 和 y 坐标
Y = [maxY minY];
XI = [0:1:columns]; %定义内推和外推的x坐标范围,columns为图像宽度
YI = interp1(X,Y,XI,'linear','extrap');%interpolate lines使用interp1函数对X和Y进行内推和外推,得到YI
plot(XI,YI,'LineWidth',5,'Color','red')%Plot the lines在图像上绘制内推和外推后的左车道线

%右边
%Plot the negative slope longest line extapolated/interpolated右车道线的内推外推和绘制
Xneg = end_pointsXneg;
Yneg = end_pointsYneg;
%NEW X AND Y METHOD WITH Nslope INSTEAD OF negSlope
Xneg = [minXN+5 maxXN];
Yneg = [minYN maxYN];
XIneg = [0:1:columns];
YIneg = interp1(Xneg,Yneg,XIneg,'linear','extrap');%interpolate lines
plot(XIneg,YIneg,'LineWidth',5,'Color','red')%Plot the lines

%Find the minimum y value, want to cut off where we plot so
%lanes lines dont criss cross
%If something fucked up assume the last frames minY
%找到图像中车道线y坐标的最小值minY和最大值maxY,并与上一帧的minYprev比较,
%如果变化过大则保持上一帧的值,为限定车道线绘画范围做准备。为了保证视频的连贯性
minY = min(lineTrack(:,2));
minY = 460; %如果minY检测结果异常,默认取460作为minY 
maxY = 670;% max(lineTrack(:,2));
if (firstRun == 1)
   if (abs(minY-minYprev) > 20)
       minY = minYprev;
       fucked = fucked+1
   end
end
minYprev = minY;




plottedY = [end_pointsY end_pointsYneg];
%minY =  min(plottedY);
%maxY = max(plottedY);

%Cut off everything below minY, towards the top of the image
%Cut off everything above maxY towards bottom of image
frameGrab = getframe(gca);% 获取当前图像并存储在frameGrab中
finalLanes = frame2im(frameGrab);%提取图像数据

%进行裁剪
for q = 1:columns
    for w = 1:rows % 如果点的y坐标w小于minY或者大于maxY,则设置 finalLanes(w,q) = 0; 
        if (w < minY  || w > maxY)
            finalLanes(w,q) = 0;
        end
    end
end
%figure 
size(finalLanes)  %得到裁剪后的图像finalLanes的大小 
imshow(finalLanes);
[finalLanes_rows, finalLanes_columns, ~] = size(finalLanes);  % 获取finalLanes的size

framewLines = zeros(size(frame));  
rows=size(frame,1);
columns=size(frame,2);
%Put red lines on original frame now将绘制的红色车道线叠加在原始图像frame上
for b = 1:rows
    for a = 1:columns
       if a<=columns 
            if b>finalLanes_rows   || a>finalLanes_columns
                continue;
            end 
            if (finalLanes(b,a,1) <5)%check red channel since we plotted lines in red 红色通道的值一般比5打大，如果不是红色车道线，就只想下列语句
                framewLines(b,a,1) = frame(b,a,1);%if not red, change pixel to frame pixel
                framewLines(b,a,2) = frame(b,a,2);%frame的红色通道值赋值给framewLines 
                framewLines(b,a,3) = frame(b,a,3);
            end
       end
    end
end

%%FILL LANE BETWEEN RED LINES WITH COLOR在车道线之间填充颜色
%redLeft1 is left side of left red line,  2 is right side of left 
%Cycle through pixels across when we first hit perfect red mark it with
%redLeft1, when we hit a non perfect red we know the line is over, mark it
%with red Left2 same with right. Color when redLeft 2 triggers and before
%redRight1 triggers
laneFill = framewLines; %用于填充
for y = minY:maxY
    redLeft1 = 0;
    redLeft2 = 0;
    redRight1 = 0;
    redRight2 = 0;
    for x = 1:1200
        if (redLeft1 == 0 && redLeft2 == 0 && framewLines(y,x,1) == 255 && framewLines(y,x,3) < 1)
            redLeft1 = 1; %
        end
        if (redLeft1 == 1 && redLeft2 == 0 && framewLines(y,x,2) >1 && framewLines(y,x+1,2) > 1 )
            redLeft2 = 1; %左车道线结束，且绿色通道较大
        end
        if(redLeft2 == 1 && redRight1 == 0 && framewLines(y,x,1) == 255 && framewLines(y,x,3) < 1)
            redRight1 = 1;
        end
        if (redLeft2 ==1 && redRight1 == 0)
           laneFill(y,x,1) = 0;
           laneFill(y,x,2) = 0;
           laneFill(y,x,3) = 255;
        end
        
    end
end

%PUT TRANSPARENCY COLOR ON LANE FROM STACKOVERFLOW
finalLane = frame;
for x = 1:1200
    %pt1 is bottom left of lane
    if(laneFill(maxY,x,3) == 255 && laneFill(maxY,x,1) == 0 && laneFill(maxY,x-1,3) == 0)
        pt1 = [x, maxY];%左下角点的坐标
    end
    
    %pt2 is top left of lane

    if(laneFill(minY,x,3) == 255 && laneFill(minY,x,1) == 0 && laneFill(minY,x-1,3) == 0)
        pt2 = [x, minY];%左上角点的坐标
    end
    
    %pt3 is top right of lane
    if(laneFill(minY,x,3) == 255 && laneFill(minY,x,1) == 0 && laneFill(minY,x+1,1) == 255)
        pt3 = [x, minY];%右上角的坐标
    end
    
    %pt4 is bottom right of lane
    if(laneFill(maxY,x,3) == 255 && laneFill(maxY,x,1) == 0 && laneFill(maxY,x+1,1) == 255)
        pt4 = [x, maxY];%右下角点的坐标
    end
  
end

%定义四个角的坐标变量
    pt1 = [342, 670];
    pt2 = [605, 450];
    pt3 = [676, 451];
    pt4 = [1100, 667];
    pt_x = [pt1(1) pt2(1) pt3(1) pt4(1)];%提取四个点的x和y坐标
    pt_y = [pt1(2) pt2(2) pt3(2) pt4(2)];
    BW = poly2mask(pt_x, pt_y, 720, 1280); %根据四个点生成掩膜BW
    
    
    clr = [255 0 255];            % color of lane颜色设置
    a = 0.3;                 % play around with 透明度因子
    z = false(size(BW));   %生成与BW相同大小的0矩阵
    
    %将BW,z,z按第3维连接,得到一个3通道mask 
    mask = cat(3,BW,z,z); finalLane(mask) = a*clr(1) + (1-a)*finalLane(mask);
    mask = cat(3,z,BW,z); finalLane(mask) = a*clr(2) + (1-a)*finalLane(mask);
    mask = cat(3,z,z,BW); finalLane(mask) = a*clr(3) + (1-a)*finalLane(mask);
    
  
 
  %Take red lines from framewLines and put them on finaLanes pic with
  %transparent stuff
  for b = 1:rows
      for a = 1:columns %红色点，属于车道线
          if (framewLines(b,a,1) == 255 && framewLines(b,a,2) == 0 && framewLines(b,a,3) == 0)%if red line
              finalLane(b,a,1) = 255;
              finalLane(b,a,2) = 0;
              finalLane(b,a,3) = 0;  %将GRB设置为[255,0,0]
          end 
      end
  end
 
imshow(finalLane)   %最终的标线，但是跟视频结合
%imshow(noHor)

%FINDING INTERSECTION OF HOUGH LINES

x1 = [pt_x(1) pt_x(2)]; %起点和重点坐标
y1 = [pt_y(1) pt_y(2)];
x2 = [pt_x(3) pt_x(4)];
y2 = [pt_y(3) pt_y(4)];

%fit linear polynomial拟合出两条直线的方程
p1 = polyfit(x1,y1,1);
p2 = polyfit(x2,y2,1);

%calculate intersection
x_intersect = fzero(@(x) polyval(p1-p2,x),3); %求出两条直线交点的坐标
y_intersect = polyval(p1,x_intersect);

%plot intersection
% imshow(noHor)
% hold on
% line(x1,y1);
% hold on;
% line(x2,y2);
% plot(x_intersect,y_intersect,'r*')

%USE INTERSECTION TO DECIDE IF TURN OR NA
%LEft of x = 616? for left
%Stragiht was like x = 642  
if (x_intersect < 630)
    Direction = 'Left'
end
if (x_intersect > 630)
    Direction = 'Straight'
end
if (x_intersect > 680)
    Direction = 'Right'
end

tex = text(600,150,Direction,'Color','k','FontSize',30); %在图上坐标（600,150）处现实文本Direction
frameGrab = getframe(gca); %获取当前图像并储存在矩阵中
videoWrite = frame2im(frameGrab); %将矩阵转化为图像
%figure
firstRun = 1;
% imshow(framewLines)
imshow(laneFill)
writeVideo(vidObj,finalLane) %Add frame to video将VideoWrite写入视频文件
%cla
%n
end
close(vidObj);


            
            
