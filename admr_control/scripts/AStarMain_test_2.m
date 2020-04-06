%clear
%%%%%%%%%%%%%%%%%%%%%%
robot_radius = 0.15;
safety_factor = 4;
map_pixels_x = 1309;
map_pixels_y = 735;
real_map_x = 50;
real_map_y= 30;
%%%%ros initilizations
x_topic = rospublisher('/desired_path_x', 'std_msgs/Float32MultiArray') ;
y_topic = rospublisher('/desired_path_y', 'std_msgs/Float32MultiArray') ;
x_path = rosmessage('std_msgs/Float32MultiArray');
y_path = rosmessage('std_msgs/Float32MultiArray');


%%%%% importing and processing the map
MAP = imbinarize(rgb2gray(imread('uni.jpg')));
imshow(MAP);
MAP = ~MAP;
MAP1 = MAP;
%disp(size(MAP));
[rows_map, columns_map] = size(MAP);
%disp([rows_map, columns_map]);

%%%%% taking start and target from gui
h=msgbox('Please Select the Vehicle initial position using the Left Mouse button');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
but=0;
while (but ~= 1)
    [xval,yval,but]=ginput(1);
    xval=floor(xval);
    yval=floor(yval);
end
xStart=xval;
yStart=yval;
StartX=xStart;
StartY=yStart;
disp([StartX, StartY])
h=msgbox('Please Select the Target using the Left Mouse button');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
but=0;
while (but ~= 1)
    [xval,yval,but]=ginput(1);
end
xval=floor(xval);
yval=floor(yval);
xTarget=xval;
yTarget=yval;
disp([xTarget, yTarget])

%%%%%%%%% computing the path
GoalRegister=int8(zeros(rows_map, columns_map));
GoalRegister(yTarget, xTarget)=1;
Connecting_Distance=4;
physical_constraints=round((robot_radius/(real_map_x/map_pixels_x))*safety_factor);
% physical_constraints=20;
for t=2:3
OptimalPath=ASTARPATH_mod(StartX,StartY,MAP,GoalRegister,Connecting_Distance,physical_constraints);
if (t==2)
    OptimalPath1=OptimalPath;
end
if(t==3)
    OptimalPath2=OptimalPath;
end
[rows_OP, columns_OP] = size(OptimalPath);
limit1 = round(rows_OP/4);
limit2 = limit1+round(rows_OP/2);
%%%%%%%%% visualizing the path
if size(OptimalPath,2)>1
figure(t)
imagesc((MAP1))
    colormap(flipud(gray));
hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')
for s = limit1:limit2
MAP(OptimalPath(s,1),OptimalPath(s,2))=1;
end
legend('Goal','Start','Path')
else 
     pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
 end
end
%%%%%%% publishing the path
x_path.Data=OptimalPath(:,1)*(real_map_x/map_pixels_x);
x_path.Data=flip(x_path.Data);
y_path.Data=OptimalPath(:,2)*(real_map_y/map_pixels_y);
y_path.Data=flip(y_path.Data);
send(x_topic,x_path);
send(y_topic,y_path);
