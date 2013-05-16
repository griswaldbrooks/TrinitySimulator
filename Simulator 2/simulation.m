cla, clc, clear
max_dim = 2.5;
axis([-max_dim - 0.2, max_dim + 0.2, -max_dim - 0.2, max_dim + 0.2])
axis square
axis manual
hold all


% Plotting variables
HEADING_LENGTH = 0.25;

% Robot variables
r_diameter = 0.25; % (m)
wh_radius = 0.05; % (m)

C = [0.5*wh_radius,        0.5*wh_radius,         0, 0;
     wh_radius/r_diameter, -wh_radius/r_diameter, 0, 0];


% Robot Pose (x,y,theta)
r_pose = [0,0,0]';
r_pose_est = [0,0,0]';
% State variables
x_init = [10,10,0,0,0,0,0,0]';  % [right wheel velocity, left wheel velocity, 
                              %  right wheel torque, left wheel torque]
                              % [right wheel velocity estimate, left wheel velocity estimate, 
                              %  right wheel torque estimate, left wheel torque estimate]
%x_dot = [0,0,0,0]';
% Output variables
v = 0;
om = 0;
y = [v,om]';
% Control input (voltage)
%u = [0,0]';


% Time variables (seconds)
dt = 0.1;
T = 300;

disp('Computing simulation.');

% Calculate simulation
tic
[t, x] = ode45('calc_xdot', 0:dt:T, x_init);
toc

figure(1);

disp('Plotting simulation.');
% Plot simulation
for n = 1:length(t)
    cla
    
    %%% MOTION MODEL %%%
    y = C*(x(n,1:4)');
    y_est = C*(x(n,5:8)');
    
    v = y(1);
    om = y(2);
    
    v_est = y_est(1);
    om_est = y_est(2);
    
    r_pose(3) = r_pose(3) + om*dt;
    r_pose(1) = r_pose(1) + v*cos(r_pose(3))*dt;
    r_pose(2) = r_pose(2) + v*sin(r_pose(3))*dt;
    
    r_pose_est(3) = r_pose_est(3) + om_est*dt;
    r_pose_est(1) = r_pose_est(1) + v_est*cos(r_pose_est(3))*dt;
    r_pose_est(2) = r_pose_est(2) + v_est*sin(r_pose_est(3))*dt;
    
    disp(r_pose);
    
    %%% PLOT ROBOT %%%
    % Robot Transformation Matrix
    T = [cos(r_pose(3)),-sin(r_pose(3)), r_pose(1);
         sin(r_pose(3)), cos(r_pose(3)), r_pose(2);
                      0,              0,        1];
    plotRobot(T, r_diameter, HEADING_LENGTH, 'k');
    
    T_est = [cos(r_pose_est(3)),-sin(r_pose_est(3)), r_pose_est(1);
             sin(r_pose_est(3)), cos(r_pose_est(3)), r_pose_est(2);
                                      0,              0,        1];
    plotRobot(T_est, r_diameter, HEADING_LENGTH, 'r');
    
    pause(1/256);
end

figure(2);
cla
plot(t, x(:,1) - x(:,5), t, x(:,2) - x(:,6), t, x(:,3) - x(:,7), t, x(:,4) - x(:,8));
hlegend = {'Angular Velocity, Right', 'Angular Velocity, Left', 'Torque, Right', 'Torque, Right'};
legend(hlegend);