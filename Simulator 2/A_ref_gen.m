function A_ref = A_ref_gen()

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
kf = 0.1;           % Wheel friction coefficient
kbwr = 2*pi*(5);   % Motor bandwidth
kbwl = 2*pi*(5);   % Motor bandwidth


A11 = -kf.*inv(L);
A12 = zeros(2);
A21 = inv(L);
A22 = [-kbwr,     0;
           0, -kbwl];
       
A_ref = [A11, A21; A12, A22];