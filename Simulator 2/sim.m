cla, clc, clear
max_dim = 248;
axis([-max_dim - 20, max_dim + 20, -max_dim - 20, max_dim + 20])
axis square
axis manual
hold all


% Plotting variables
HEADING_LENGTH = 25;

% Robot variables
r_diameter = 0.25; % (m)
r_mass = 5; % (kg)
wh_radius = 0.05; % (m)
wh_mass = 0.125;
wh_inertia = (wh_mass/2)*(wh_radius^2);
K_acc = ((2/wh_radius)*wh_inertia + wh_radius*r_mass)^-1;
K_ang = ((r_diameter/wh_radius)*wh_inertia + wh_radius*(r_mass/2))^-1;
% Robot Pose (x,y,theta)
r_pose = [0,0,0]';
v = 0;
om = 0;
torque_l = 0;
torque_r = 0;

% Time variables (seconds)
dt = 0.1;
T = 300;

for t = 0:dt:T
    cla
    
    %%% PLOT ROBOT %%%
    % Robot Transformation Matrix
    T = [cos(r_pose(3)),-sin(r_pose(3)), r_pose(1);
         sin(r_pose(3)), cos(r_pose(3)), r_pose(2);
                      0,              0,        1];
    plotRobot(T, r_diameter, HEADING_LENGTH, 'k');
    
    %%% MOTION MODEL %%%
    
    acc_r = K_acc*(torque_r + torque_l);
    ang_r = K_ang*(torque_r - torque_l);
    
    v = v + acc_r*dt - 0.002*rand(1)*v;
    om = om + ang_r*dt - 0.0005*rand(1)*om;
    
    r_pose(3) = r_pose(3) + om*dt;
    r_pose(1) = r_pose(1) + v*cos(r_pose(3))*dt;
    r_pose(2) = r_pose(2) + v*sin(r_pose(3))*dt;
    
    torque_l = 0.8*sin(t);
    torque_r = 0.9*sin(t);
    
    %r_pose
    
    pause(1/256);
end
