%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om, rm1, rm2, om_s] = robotNav2(ranges, rm1, rm2, r_pose, goal, vp, om_s, dt)

plot_scaler = 1000;
RANGE_OFF = 15;
RANGE_COEFF = 0.25;
OM_MAX = 1;
sub_ang = 15*(pi/180);

% Magnitude 1/r^2
f_range10 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 - 2*sub_ang),sin(-pi/4 - 2*sub_ang)]';
f_range11 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 - sub_ang),sin(-pi/4 - sub_ang)]';
f_range12 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4),sin(-pi/4)]';
f_range13 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 + sub_ang),sin(-pi/4 + sub_ang)]';
f_range14 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 + 2*sub_ang),sin(-pi/4 + 2*sub_ang)]';

f_range1 = f_range10 + f_range11 + f_range12 + f_range13 + f_range14;
%f_range1 = f_range12;

f_range20 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 - 2*sub_ang),sin(pi/4 - 2*sub_ang)]';
f_range21 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 - sub_ang),sin(pi/4 - sub_ang)]';
f_range22 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4),sin(pi/4)]';
f_range23 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 + sub_ang),sin(pi/4 + sub_ang)]';
f_range24 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 + 2*sub_ang),sin(pi/4 + 2*sub_ang)]';

f_range2 = f_range20 + f_range21 + f_range22 + f_range23 + f_range24;
%f_range2 = f_range22;

f_ranges = f_range1 + f_range2;

% Plot occlusion force components
plotForce(f_range1, r_pose, plot_scaler, 'y');
plotForce(f_range2, r_pose, plot_scaler, 'y');
% Plot occlusion force
plotForce(f_ranges, r_pose, plot_scaler, 'g');

goal_range = sqrt((r_pose(1) - goal(1))^2 + (r_pose(2) - goal(2))^2);
f_goal = (1/((0.5*min(goal_range,25000))^2))*[goal(1) - r_pose(1), goal(2) - r_pose(2)]';
f_goal = [cos(-r_pose(3)),-sin(-r_pose(3));sin(-r_pose(3)),cos(-r_pose(3))]*f_goal;
% Plot goal force
plotForce(f_goal, r_pose, plot_scaler, 'm');

f_heading = 0.01*[cos(0),sin(0)]';
% Plot current heading force
%fh = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_heading;
%line([r_pose(1), plot_scaler*fh(1) + r_pose(1)],[r_pose(2), plot_scaler*fh(2) + r_pose(2)], 'Color','k');
plotForce(f_heading, r_pose, plot_scaler, 'k');

f = f_ranges + f_goal + f_heading;

% Plot resultant force
fres = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f;
line([r_pose(1), plot_scaler*fres(1) + r_pose(1)],[r_pose(2), plot_scaler*fres(2) + r_pose(2)], 'Color','b');

th_in = (atan2(f(2),f(1)));
v_desired = 0.5*log((min(ranges)- 10)/10)*(0.5*(tanh(goal_range - 10) + 1));
om = (7.0*(sqrt(f(2)^2 + f(1)^2))*th_in*dt);
text(10,6,'om: ');
text(70, 6, num2str(om));
if(om > OM_MAX)
    om = OM_MAX;
elseif (om < -OM_MAX)
    om = -OM_MAX;
end
v = 0.1*vp + 0.9*v_desired;
if(goal_range < 10)
    v = 0;
    om = 0;
end