function [x_dot_out] = calc_xdot(t, x)
x_act = x(1:4, :);
x_hat = x(5:8, :);
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
kf = 0.0522;           % Wheel friction coefficient
kbwr = 2*pi*(10);   % Motor bandwidth
kbwl = 2*pi*(10);   % Motor bandwidth
kgr = 0.5;          % Voltage gain
kgl = 0.5;          % Voltage gain

A11 = -kf.*inv(L);
A12 = zeros(2);
A21 = inv(L);
A22 = [-kbwr,     0;
           0, -kbwl];

A = [A11, A21; A12, A22];

B = [0,   0;
     0,   0;
     kgr, 0;
     0,   kgl];
C = eye(4);

persistent u;
if isempty(u)
    u = [0,0]';
end

A_hat = A;

% State Estimator
%x_hat_dot = estimator(A_hat, B, C, x_hat, C*x_act + [0.002*rand(2,1);0.01*rand(2,1)], u);
x_hat_dot = [0,0,0,0]';
% Control input (voltage)
%u = 12*[sin(0.1*t),cos(0.1*t)]';

u = controller(x_act, A_hat, B);

% Motion Model
x_dot = A*x_act + B*u;


x_dot_out = [x_dot;x_hat_dot];