function A_new = adapt_sys(x_hat, x_act)

persistent kf;
if(isempty(kf))
    kf = 0.00010;           % Wheel friction coefficient
end
persistent kbwr;
if(isempty(kbwr))
    kbwr = 2*pi*(100);   % Motor bandwidth
end
persistent kbwl;
if(isempty(kbwl))
    kbwl = 2*pi*(100);   % Motor bandwidth
end

% Update estimates 
kf1 = 0.01*(x_hat(1) - x_act(1))*kf + kf;
kf2 = 0.01*(x_hat(2) - x_act(2))*kf + kf;
kf = (kf1 + kf2)/2;
kbwr = 0.01*(x_hat(3) - x_act(3))*kbwr + kbwr;
kbwl = 0.01*(x_hat(4) - x_act(4))*kbwl + kbwl;

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


A11 = -kf.*inv(L);
A12 = zeros(2);
A21 = inv(L);
A22 = [-kbwr,     0;
           0, -kbwl];
       
A_new = [A11, A21; A12, A22];



