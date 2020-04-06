%clear
%%%%%%%%%%%%%%%%%%%%%%
robot_radius = 0.15;
safety_factor = 4;
map_pixels_w = 1309;
% map_pixels_h = 735;
% real_map_x = 30;
% real_map_y= 50;
% x_scale = real_map_x/map_pixels_h;
% y_scale = real_map_y/map_pixels_w;
scaling = 1/26;
%%%%ros initilizations
x_topic = rospublisher('/desired_path_x', 'std_msgs/Float32MultiArray') ;
y_topic = rospublisher('/desired_path_y', 'std_msgs/Float32MultiArray') ;
x_path = rosmessage('std_msgs/Float32MultiArray');
y_path = rosmessage('std_msgs/Float32MultiArray');

%%%%% importing and processing the map
MAP = imbinarize(rgb2gray(imread('uni.jpg')));
imshow(MAP);
MAP = ~MAP;
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
physical_constraints=round((robot_radius/scaling)*safety_factor);
% physical_constraints=20;
OptimalPath=ASTARPATH(StartX,StartY,MAP,GoalRegister,Connecting_Distance,physical_constraints);

%%%%%%%%% visualizing the path
if size(OptimalPath,2)>1
figure(10)
imagesc((MAP))
    colormap(flipud(gray));
hold on
plot(OptimalPath(1,2),OptimalPath(1,1),'o','color','k')
plot(OptimalPath(end,2),OptimalPath(end,1),'o','color','b')
plot(OptimalPath(:,2),OptimalPath(:,1),'r')
legend('Goal','Start','Path')
else 
     pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
 end

%%%%%% publishing the path
x_path.Data=OptimalPath(:,1)*scaling;
x_path.Data=flip(x_path.Data);
y_path.Data=OptimalPath(:,2)*scaling;
y_path.Data=flip(y_path.Data);
send(x_topic,x_path);
send(y_topic,y_path);
