bag = rosbag('bags/integ.bag');
% path planner msgs
path_x_selection = select(bag, 'Topic' , '/desired_path_x');
path_x_msg = readMessages(path_x_selection);
path_y_selection = select(bag, 'Topic' , '/desired_path_y');
path_y_msg = readMessages(path_y_selection);
desired_x = path_x_msg{1,1}.Data;
desired_y = path_y_msg{1,1}.Data;
desired_path = [desired_x desired_y];
%gazebo msgs
realStates_selection = select(bag,'Topic','/real_states');
realStates_msg = readMessages(realStates_selection);
real_sz = size (realStates_msg);
real_states = zeros(real_sz(1) , 3);
for i=1 : real_sz(1)
    real_states(i,:) = realStates_msg {i,1}.Data;
end
% odometery msgs_raw
odom_selection = select(bag,'Topic','/odom_raw_sync');
odom_msg = readMessages(odom_selection);
odom_sz = size(odom_msg);
odom_states = zeros(odom_sz(1),3);
for i=1:odom_sz(1)
    odom_states(i,:) = odom_msg{i,1}.Data;
end
% Clipping odometry readings
clip_index=1;
for i=1:odom_sz(1)
    if abs(odom_states(i,1)-desired_x(1))<=0.01 && abs(odom_states(i,2)-desired_y(1))<=0.01
        clip_index=i;
        break;
    end
end
odom_states = odom_states( clip_index:end, :);
odom_sz = size(odom_states);
% get filtered data
filter_selection = select(bag,'Topic','/filtered_odom');
filter_msg = readMessages(filter_selection);
filter_sz = size(filter_msg,1);
filter_states=zeros(filter_sz(1),3);
for i=1:filter_sz
    if isempty(filter_msg{i,1}.Data)
        filter_states(i,:)=0;
        
    else
    filter_states(i,:)=filter_msg{i,1}.Data;
    end
end
filter_clip = 1;
for i=1:filter_sz
    if filter_states(i,:) ~=0
    filter_clip=i;
    break;
    end
end
filter_states=filter_states(filter_clip:end,:);
% get noisy data
noisy_selection = select(bag,'Topic','/noisy_odom');
noisy_msg = readMessages(noisy_selection);
noisy_sz = size(noisy_msg,1);
noisy_states=zeros(noisy_sz(1),3);
for i=1:noisy_sz
    if isempty(noisy_msg{i,1}.Data)
        noisy_states(i,:)=0;
        
    else
    noisy_states(i,:)=noisy_msg{i,1}.Data;
    end
end
noisy_clip = 1;
for i=1:noisy_sz
    if noisy_states(i,:) ~=0
    noisy_clip=i;
    break;
    end
end
noisy_states=noisy_states(noisy_clip:end,:);
% get control inputs
control_input_selection = select(bag,'Topic','/cmd_vel');
control_input_msg = readMessages(control_input_selection);
input_sz = size(control_input_msg);
control_input = zeros(input_sz(1), 2);
for i=1:input_sz(1)
    control_input(i, 1) = control_input_msg{i,1}.Linear.X;
    control_input(i, 2) = control_input_msg{i,1}.Angular.Z;
end 
control_input = control_input(clip_index:end, :);
input_sz = size(control_input);
dt = 0.1;
% calculate ds and dtheta
s = zeros(input_sz(1), 1);
for i=1:input_sz(1)
    s(i) = dt*trapz(control_input(1:i, 1)); %10HZ publisher
end
theta = zeros(input_sz(1), 1);
for i=1:input_sz(1)
    theta(i) = dt*trapz(control_input(1:i, 2));
end
delta_s = zeros(input_sz(1), 1);
delta_s(2:end) = diff(s);
delta_theta = zeros(input_sz(1), 1);
delta_theta(2:end) = diff(theta);
% IMU_raw
imu_selection = select(bag, 'Topic' , '/imu_raw_sync');
imu_msg = readMessages(imu_selection);
imu_sz=size(imu_msg);
imu_orientation_quat=zeros(imu_sz(1),4);
imu_linearAcc=zeros(imu_sz(1),2);
imu_linearVel=zeros(imu_sz(1),2);
imu_position=zeros(imu_sz(1),2);
theta_imu = zeros(imu_sz(1) , 3);
theta_imu_z = zeros(imu_sz(1) , 1);
dt = 0.1;
% get imu_sync states
for i=1:imu_sz(1)
   imu_orientation_quat(i,:) =imu_msg{i, 1}.Data(3:end);
   imu_linearAcc(i,1)=imu_msg{i,1}.Data(1);
   imu_linearAcc(i,2)=imu_msg{i,1}.Data(2);
   theta_imu(i,:) = quat2angle(imu_orientation_quat(i,:),'ZYX');
   theta_imu_z(i) = theta_imu(i,1);
end
% double integrate acc to get position
for i=1:imu_sz(1)-1
    imu_linearVel(i,1) = dt*trapz(imu_linearAcc(i:i+1 ,1));
    imu_position(i,1) = dt*trapz(imu_linearVel(i:i+1 ,1));
    imu_linearVel(i,2) = dt*trapz(imu_linearAcc(i:i+1 ,2));
    imu_position(i,2) = dt*trapz(imu_linearVel(i:i+1 ,2));
end
imu_raw_states = zeros(imu_sz(1),3);
imu_raw_states(:,1) = imu_position(:,1);
imu_raw_states(:,2) = imu_position(:,2);
imu_raw_states(:,3) = theta_imu_z(:,1);
imu_raw_states = imu_raw_states(clip_index:end,:);
% IMU_noisy
imu_selection = select(bag, 'Topic' , '/imu_noisy_sync');
imu_msg = readMessages(imu_selection);
imu_sz=size(imu_msg);
imu_orientation_quat=zeros(imu_sz(1),4);
imu_linearAcc=zeros(imu_sz(1),2);
imu_linearVel=zeros(imu_sz(1),2);
imu_position=zeros(imu_sz(1),2);
theta_imu = zeros(imu_sz(1) , 3);
theta_imu_z = zeros(imu_sz(1) , 1);
% get imu noisy synced data
for i=1:imu_sz(1)
   imu_orientation_quat(i,:) =imu_msg{i, 1}.Data(3:end);
   imu_linearAcc(i,1)=imu_msg{i,1}.Data(1);
   imu_linearAcc(i,2)=imu_msg{i,1}.Data(2);
   theta_imu(i,:) = quat2eul(imu_orientation_quat(i,:),'ZYX');
   theta_imu_z(i) = theta_imu(i,1);
end
% double integrate acc to get positions
for i=1:imu_sz(1)-1
    imu_linearVel(i,1) = dt*trapz(imu_linearAcc(i:i+1 ,1));
    imu_position(i,1) = dt*trapz(imu_linearVel(i:i+1 ,1));
    imu_linearVel(i,2) = dt*trapz(imu_linearAcc(i:i+1 ,2));
    imu_position(i,2) = dt*trapz(imu_linearVel(i:i+1 ,2));
end
imu_noisy_states = zeros(imu_sz(1),3);
imu_noisy_states(:,1) = imu_position(:,1);
imu_noisy_states(:,2) = imu_position(:,2);
imu_noisy_states(:,3) = theta_imu_z(:,1);
imu_noisy_states = imu_noisy_states(clip_index:end,:);
% error analysis
error_x = abs(filter_states(:,1)-real_states(:,1));
max_error_x = max(error_x)
min_error_x = min(error_x)
rms_error_x = rms(error_x)
error_y = abs(filter_states(:,2)-real_states(:,2));
max_error_y = max(error_y)
min_error_y = min(error_y)
rms_error_y = rms(error_y)
error_th = abs(filter_states(:,3)-real_states(:,3));
max_error_th = max(error_th)
min_error_th= min(error_th)
rms_error_th = rms(error_th)
% % path plotting
figure(1);
grid;
% plot the desired path
plot(desired_y ,-1*desired_x,'m','LineWidth',2);
hold on
% plot odometry msgs
plot(filter_states(:,2),-1*filter_states(:,1),'b','LineWidth',2);
hold on
% plot odometry noisy msgs
plot(noisy_states(:,2),-1*noisy_states(:,1),'g','LineWidth',2);
hold on
plot(real_states(:,2),-1*real_states(:,1),'r','LineWidth',2);
legend('desired path','filtered','measured noisy','controlled');
% odom_readings
% x_readings
figure(2);
grid;
% plot x_raw
plot(odom_states(:,1) ,'m','LineWidth',2);
hold on
% plot x_noisy
plot(filter_states(:,1),'b','LineWidth',2);
hold on
% plot x_filtered
plot(noisy_states(:,1),'g','LineWidth',2);
legend('x raw','x filtered','x noisy');
% y_readings
figure(3);
grid;
% plot y_raw
plot(odom_states(:,1) ,'m','LineWidth',2);
hold on
% plot y_noisy
plot(filter_states(:,1),'b','LineWidth',2);
hold on
% plot y_filtered
plot(noisy_states(:,1),'g','LineWidth',2);
legend('y raw','y filtered','y noisy');
% theta_readings
figure(4);
grid;
% plot theta_raw
plot(odom_states(:,3) ,'m','LineWidth',2);
hold on
% plot theta_noisy
plot(filter_states(:,3),'b','LineWidth',2);
hold on
% plot theta_filtered
plot(noisy_states(:,3),'g','LineWidth',2);
legend('theta raw','theta filtered','theta noisy');
% plot linear velocity profile
figure(5);
grid;
plot(control_input(:, 1));
% plot angular velocity profile
figure(6);
grid;
plot(control_input(:,2));
% plot imu readings
% imu_x
figure(7);
grid;
% plot x_real
plot(odom_states(:,1) ,'m','LineWidth',2);
hold on
% plot x_raw
plot(imu_raw_states(:,1),'b','LineWidth',2);
hold on
% plot x_noisy
plot(imu_noisy_states(:,1),'g','LineWidth',1);
legend('x real','x raw','x noisy');
% imu_y
figure(8);
grid;
% plot y_real
plot(odom_states(:,2) ,'m','LineWidth',2);
hold on
% plot y_raw
plot(imu_raw_states(:,2),'b','LineWidth',2);
hold on
% plot y_noisy
plot(imu_noisy_states(:,2),'g','LineWidth',1);
legend('y real','y raw','y noisy');
% imu_th
figure(9);
grid;
% plot th_real
plot(odom_states(:,3) ,'m','LineWidth',2);
hold on
% plot th_raw
plot(imu_raw_states(:,3),'b','LineWidth',2);
hold on
% plot th_noisy
plot(imu_noisy_states(:,3),'g','LineWidth',1);
legend('th real','th raw','th noisy');