
% This is the program for path planning of PUMA560
% This program is made up of 2 sections : 
% first section is for the path planning of Cartesian Move
% second section is for the path planning of Joint Move
% both of the path planning are based on polynomial path planning
% last modified by YuTung, Cheng
% last modified Jun 14th,2018


clear all; clc; close all;


%%  Section 1 : Path Planning based on Cartesian Coordinate

% =========== set up parameters ===========%
% set A, B, C points for the trajectory
% A is the start point, B is the via point, and C is the end point of this
% path
A = [0 0 1 -0.30; -1 0 0 0.20; 0 -1 0 0.30; 0 0 0 1]; %unit : m
B = [0 0 1 0.50; 0 1 0 -0.20; -1 0 0 0.60; 0 0 0 1]; %unit : m
C = [-1 0 0 -0.25; 0 0 1 0.20; 0 1 0 -0.30; 0 0 0 1]; %unit : m

% changed the position into Cartesian space
position_A = DHconvert2cartesian(A);
position_B = DHconvert2cartesian(B);
position_C = DHconvert2cartesian(C);

% ===============calculation of the path planning=============%
% Polynomial Path Planning
tacc = 1;
T = 5*tacc; % If the T is more times of tacc, it makes the trajectory sharper
t = [-tacc:0.001:T];
q = zeros(4,4,length(t)); % trajectory for the planned path
q_dot = zeros(4,4,length(t)); % velocity
q_dou_dot = zeros(4,4,length(t)); % acceleration

del_C = C-B;
del_B = A-B;

for i = 1:1:length(t)
    
    % two time segments is considered
    % first segments is for the time with acceleration
    % second segments is the linear part to reach end point after the
    % acceleration time
    
    if t(i)<=tacc
        % the time between -tacc to +tacc
        h = (t(1,i)+tacc)/2*tacc;
        q(:,:,i) = [ (del_C*(tacc/T)+del_B)*(2-h)*h^2 - 2*del_B ]*h + B + del_B;
        q_dot(:,:,i) = [ (del_C*(tacc/T)+del_B)*(1.5-h)*2*h^2 - del_B ]*(1/tacc);
        q_dou_dot(:,:,i) = [ ( del_C*(tacc/T) + del_B )*( 1-h ) ]*(3*h/tacc^2);
    else
        % the time between -tacc to +T
        q(:,:,i) = del_C*t(1,i)/T+B;
        q_dot(:,:,i) = del_C/T;
        q_dou_dot(:,:,i) = 0;
    end
end

%  ============Plotting==================  %
% first figure, plot the trajectory, velocity, acceleration of x, y, z axis

x = zeros(1,length(t));
y = zeros(1,length(t));
z = zeros(1,length(t));
x_dot = zeros(1,length(t));
y_dot = zeros(1,length(t));
z_dot = zeros(1,length(t));
x_dou_dot = zeros(1,length(t));
y_dou_dot = zeros(1,length(t));
z_dou_dot = zeros(1,length(t));

for i = 1:1:length(t)
    position = DHconvert2cartesian(q(:,:,i));
    x(1,i) = position(1,1);
    y(1,i) = position(1,2);
    z(1,i) = position(1,3);
    
    velocity = DHconvert2cartesian(q_dot(:,:,i));
    x_dot(1,i) = velocity(1,1);
    y_dot(1,i) = velocity(1,2);
    z_dot(1,i) = velocity(1,3);
    
    acc = DHconvert2cartesian(q_dou_dot(:,:,i));
    x_dou_dot(1,i) = acc(1,1);
    y_dou_dot(1,i) = acc(1,2);
    z_dou_dot(1,i) = acc(1,3);    
end

figure(1);
title('3D path of Cartesian Move');
plot3(x,y,z)
axis tight;
grid on;
hold on;
scatter3(position_A(1,1),position_A(1,2),position_A(1,3),'r*')
hold on;
scatter3(position_B(1,1),position_B(1,2),position_B(1,3),'r*')
hold on;
scatter3(position_C(1,1),position_C(1,2),position_C(1,3),'r*')
hold off;

% second figure, plot the trajectory, velocity, acceleration of x, y, z
% axis separately
 
figure(2);
title('position of x');
subplot(3,3,1),plot(t(1,:),x(1,:))
axis tight;
grid on;

title('position of y');
subplot(3,3,4),plot(t(1,:),y(1,:))
axis tight;
grid on;

title('position of z');
subplot(3,3,7),plot(t(1,:),z(1,:))
axis tight;
grid on;

title('velocity of x');
subplot(3,3,2),plot(t(1,:),x_dot(1,:))
axis tight;
grid on;

title('velocity of y');
subplot(3,3,5),plot(t(1,:),y_dot(1,:))
axis tight;
grid on;

title('velocity of z');
subplot(3,3,8),plot(t(1,:),z_dot(1,:))
axis tight;
grid on;

title('acceleration of x');
subplot(3,3,3),plot(t(1,:),x_dou_dot(1,:))
axis tight;
grid on;

title('acceleration of y');
subplot(3,3,6),plot(t(1,:),y_dou_dot(1,:))
axis tight;
grid on;

title('acceleration of z');
subplot(3,3,9),plot(t(1,:),z_dou_dot(1,:))
axis tight;
grid on;

figure(5)
title('3D path of Cartesian Move')
plot_euler(A);hold on;
plot_euler(B);hold on;
plot_euler(C);hold on;
for i = 1:1:length(t)/50
    plot_euler(q(:,:,50*i));hold on;
end
axis tight;
grid on;
hold off;


%% Section 2 : Path Planning based on Joint Variables

% =========== set up parameters ===========%
% set A, B, C points for the trajectory
% A is the start point, B is the via point, and C is the end point of this
% path

A = [0 0 1 -0.30; -1 0 0 0.20; 0 -1 0 0.30; 0 0 0 1]; %unit : m
B = [0 0 1 0.50; 0 1 0 -0.20; -1 0 0 0.60; 0 0 0 1]; %unit : m
C = [-1 0 0 -0.25; 0 0 1 0.20; 0 1 0 -0.30; 0 0 0 1]; %unit : m

% find joint variables of each points

j_A = inverse_Kinematics(A);
j_B = inverse_Kinematics(B);
j_C = inverse_Kinematics(C);

% ===============calculation of the path planning=============%
% polynomial path planning
j_tacc = 1;
j_T = 5*tacc;
j_t = [-tacc:0.001:T];
j_q = zeros(6,length(t)); % 6xt vector for 6 join variables
j_q_dot = zeros(6,length(t)); % 6xt vector for 6 joint variables
j_q_dou_dot = zeros(6,length(t)); %6xt vector for 6 joint variables

j_del_C = (j_C-j_B).'; % 6x1 vector
j_del_B = (j_A-j_B).'; % 6x1 vector

for i = 1:1:length(t)
    
    % two time segments is considered
    % first segments is for the time with acceleration
    % second segments is the linear part to reach end point after the
    % acceleration time
    
    if j_t(i)<=j_tacc
        % the time between -tacc to +tacc
        h = (j_t(1,i)+j_tacc)/2*j_tacc; % time grid
        j_q(:,i) = [ (j_del_C*(j_tacc/j_T)+j_del_B)*(2-h)*h^2 - 2*j_del_B ]*h + j_B.' + j_del_B;
        j_q_dot(:,i) = [ (j_del_C*(j_tacc/j_T)+j_del_B)*(1.5-h)*2*h^2 - j_del_B ]*(1/j_tacc);
        j_q_dou_dot(:,i) = [ ( j_del_C*(j_tacc/j_T) + j_del_B )*( 1-h ) ]*(3*h/j_tacc^2);
    else
        % the time between -tacc to +T 
        j_q(:,i) = j_del_C*t(1,i)/j_T+ j_B.';
        j_q_dot(:,i) = j_del_C/j_T;
        j_q_dou_dot(:,i) = [0 0 0 0 0 0].';
    end
end

%  ============Plotting==================  %
% use forward kinematics to turn joint variables into cartesian space
j_q_cartesian = zeros(6,length(j_t));
for i = 1:1:length(j_t)
    j_q_cartesian(:,i) = foward_Kinematics(j_q(:,i)*(pi/180)).';
end

% figure 3 is for the 3D path of Joint Move
% this plot will first use the function of forward Kinematics to obtain the
% Cartesian space location of end effector of the robot manipulator; and it
% will plot the Cartesian space of each point after that

figure(3);
title('3D path of Joint Move');
plot3(j_q_cartesian(1,:),j_q_cartesian(2,:),j_q_cartesian(3,:));
axis tight;
grid on;
hold on;
A_plot = DHconvert2cartesian(A);
B_plot = DHconvert2cartesian(B);
C_plot = DHconvert2cartesian(C);
scatter3(A_plot(1,1),A_plot(1,2),A_plot(1,3),'r*');
hold on;
scatter3(B_plot(1,1),B_plot(1,2),B_plot(1,3),'r*');
hold on;
scatter3(C_plot(1,1),C_plot(1,2),C_plot(1,3),'r*');
hold off;


% figure 4 is the angular position, angular velocity, and angular
% acceleration of each joint
figure(4);

for i = 1:1:6
    subplot(6,3,(i-1)*3+1),plot(j_t(1,:),j_q(i,:))
    axis tight;
    grid on;
    if i == 1
        title('Joint Value');
    end
    ylabel(['theta_' num2str(i)]);
end

for i = 1:1:6
    subplot(6,3,(i-1)*3+2),plot(j_t(1,:),j_q_dot(i,:))
    axis tight;
    grid on;
    
    if i == 1
        title('Velocity');
    end
end

for i = 1:1:6
    subplot(6,3,(i-1)*3+3),plot(j_t(1,:),j_q_dou_dot(i,:))
    axis tight;
    grid on;
    if i == 1
        title('Acceleration');
    end
end


figure(6)
title('3D path of Joint Move')
plot_euler(A);hold on;
plot_euler(B);hold on;
plot_euler(C);hold on;
for i = 1:1:length(j_t)/50
    plot_euler( forward_Kinematics_T( j_q(1:6,50*i)*(pi/180) ));hold on;
end
axis tight;
grid on;
hold off;