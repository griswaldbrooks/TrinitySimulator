function x_hat = estimator(y,u)

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
%kf = 0.9;           % Wheel friction coefficient
kbwr = 2*pi*(10);   % Motor bandwidth
kbwl = 2*pi*(10);   % Motor bandwidth
kgr = 0.5;          % Voltage gain
kgl = 0.5;          % Voltage gain

A11 = -kf.*inv(L);
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
% Process noise transformation
F = eye(4);
% Process noise
v = [0.01, 0.01, 0.02,0.02]';
V = v*v'
% Sensor noise
w = [0.001,0.001]';
W = w*w'
P = are(A', (C'/(W))*C, F*V*F');
G = P*C'/(W);
 
x_hatdot = A*x_hat + B*u - G*(y - C*x_hat);
x_hat = 0;