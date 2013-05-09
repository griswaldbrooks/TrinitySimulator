cla, clc, clear
max_dim = 250;
axis([-max_dim - 20, max_dim + 20, -max_dim - 20, max_dim + 20])
axis square
axis manual
hold all


% Plotting variables
HEADING_LENGTH = 25;

% Robot variables
r_diameter = 0.25; % (m)
r_mass = 2.5; % (kg)
r_inertia = 0.5*r_mass*(r_diameter/2)^2;
wh_radius = 0.05; % (m)
wh_mass = 0.125;
wh_inertia = (wh_mass/2)*(wh_radius^2);
D1 = 2*(0.125*(r_mass*(wh_radius^2)) + 0.5*r_inertia*(wh_radius/r_diameter)^2 + 0.5*wh_inertia);
D2 = 0.25*(r_mass*(wh_radius^2)^2) - r_inertia*(wh_radius/r_diameter)^2;
L = [D1,D2;
     D2,D1];        % Mass Matrix
kf = 0.9;           % Wheel friction coefficient
kbwr = 2*pi*(10);   % Motor bandwidth
kbwl = 2*pi*(10);   % Motor bandwidth
kgr = 0.1;          % Voltage gain
kgl = 0.1;          % Voltage gain

A11 = -kf*inv(L);
A12 = zeros(2);
A21 = inv(L);
A22 = [-kbwr, 0;
           0, -kbwl];
A = [A11, A21; A12, A22];

B = [0,   0;
     0,   0;
     kgr, 0;
     0,   kgl];
C = [0.5*wh_radius,        0.5*wh_radius,         0, 0;
     wh_radius/r_diameter, -wh_radius/r_diameter, 0, 0];

% Robot Pose (x,y,theta)
r_pose = [0,0,0]';
% State variables
x = [0.1,0,0,0]';  % [right wheel velocity, left wheel velocity, 
                 %  right wheel torque, left wheel torque]
x_dot = [0,0,0,0]';
% Output variables
v = 0;
om = 0;
y = [v,om]';
% Control input (voltage)
u = [0,0]';


% Time variables (seconds)
dt = 0.0001;
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
    
    x_dot = A*x + B*u;
    y = C*x;
    
    x = x + x_dot*dt;
    
    %y(1) = (1 - 0.002*rand(1))*y(1);
    %y(2) = (1 + 0.0005*rand(1))*y(2);
    v = y(1);
    om = y(2);
    
    r_pose(3) = r_pose(3) + om*dt;
    r_pose(1) = r_pose(1) + v*cos(r_pose(3))*dt;
    r_pose(2) = r_pose(2) + v*sin(r_pose(3))*dt;
    
    r_pose
    x
    u = 100*[sin(t),sin(t)]';
    %u = [0,0]';
    
    pause(1/256);
end
