%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om, rm1, rm2, rm3] = robotNav(ranges, rm1, rm2, rm3, r_pose, vp, omp, dt)

plot_scaler = 1000;
OM_MAX = 0.5;

% Calculate Occlusion Force
f_ranges = 0.75*OccForce(ranges);

% Range Memory Forces
f_VSRanges = 0.5*VSForce(rm1, rm2, rm3);
%f_VSRanges = 0;

% Calculate Inertia
f_heading = 0.75*[cos(0),sin(0)]';

% Sum Forces
f = f_ranges + f_VSRanges + f_heading;

om = 0.1*omp + 0.9*(sqrt(f(2)^2 + f(1)^2))*(atan2(f(2),f(1)))*dt;

% Angular Saturation
if(om > OM_MAX)
    om = OM_MAX;
elseif (om < -OM_MAX)
    om = -OM_MAX;
end

v_desired = 0.5*log((min(ranges)- 10)/10)*(tanh(om)/om);
v = 0.1*vp + 0.9*v_desired;

% Plot Forces
plotForce(f_ranges, r_pose, plot_scaler, 'g');
plotForce(f_heading, r_pose, plot_scaler, 'k');
plotForce(f, r_pose, plot_scaler, 'b');
plotVSOcc(rm1, rm2, rm3, r_pose);

% Update Range Memory
[rm1, rm2, rm3] = updateRM(rm1, rm2, rm3, ranges, v, om, dt);