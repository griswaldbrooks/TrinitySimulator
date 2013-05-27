cla, clc, clear
max_dim = 2.5;
hold all

% Robot variables
r_diameter = 0.25; % (m)
wh_radius = 0.05; % (m)

C = [0.5*wh_radius,        0.5*wh_radius,         0, 0;
     wh_radius/r_diameter, -wh_radius/r_diameter, 0, 0];

% State variables
x_init = [2.1,1.5,0,0,0,0,0,0]';  % [right wheel velocity, left wheel velocity, 
                              %  right wheel torque, left wheel torque]
                              % [right wheel velocity estimate, left wheel velocity estimate, 
                              %  right wheel torque estimate, left wheel torque estimate]

% Time variables (seconds)
dt = 0.001;
T = 1;

disp('Computing simulation.');

% Calculate simulation
tic
[t, x] = ode45('calc_xdot', 0:dt:T, x_init);
toc

% Display states and state error
figure(1);
cla
plot(t, x(:,1) - x(:,5), t, x(:,2) - x(:,6), t, x(:,3) - x(:,7), t, x(:,4) - x(:,8));
hlegend = {'Angular Velocity, Right', 'Angular Velocity, Left', 'Torque, Right', 'Torque, Right'};
legend(hlegend);
title('State Error');
figure(2);
cla
plot(t, x(:,1), t, x(:,2), t, x(:,3), t, x(:,4), t, square(t*(8*pi)), 'r');
hlegend = {'Angular Velocity, Right', 'Angular Velocity, Left', 'Torque, Right', 'Torque, Right', 'Angular Velocity Reference'};
legend(hlegend);
title('State');