%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav(ranges, r_pose, goal, vp, omp, dt)

plot_scaler = 1000;

% Magnitude 1/r^2
f_range1 = -1/(0.25*((ranges(1) - 20)^2))*[cos(-pi/4),sin(-pi/4)]';
f_range2 = -1/(0.25*((ranges(2) - 20)^2))*[cos( pi/4),sin( pi/4)]';
% Plot occlusion force components
% Rotate for graph
fr1 = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_range1;
fr2 = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_range2;
line([r_pose(1), plot_scaler*fr1(1) + r_pose(1)],[r_pose(2), plot_scaler*fr1(2) + r_pose(2)], 'Color','y');
line([r_pose(1), plot_scaler*fr2(1) + r_pose(1)],[r_pose(2), plot_scaler*fr2(2) + r_pose(2)], 'Color','y');

f_ranges = f_range1 + f_range2;
% Plot occlusion force
frs = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_ranges;
line([r_pose(1), plot_scaler*frs(1) + r_pose(1)],[r_pose(2), plot_scaler*frs(2) + r_pose(2)], 'Color','g');

goal_range = sqrt((r_pose(1) - goal(1))^2 + (r_pose(2) - goal(2))^2);
f_goal = 1/((0.5*goal_range)^2)*[goal(1) - r_pose(1), goal(2) - r_pose(2)]';
% Plot goal force
line([r_pose(1), plot_scaler*f_goal(1) + r_pose(1)],[r_pose(2), plot_scaler*f_goal(2) + r_pose(2)], 'Color','m');
goal_angle = -r_pose(3) + atan2(f_goal(2),f_goal(1));
f_goal = [cos(goal_angle),-sin(goal_angle);sin(goal_angle),cos(goal_angle)]*f_goal;

f_heading = 0.05*[cos(0),sin(0)]';
% Plot current heading force
fh = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_heading;
line([r_pose(1), plot_scaler*fh(1) + r_pose(1)],[r_pose(2), plot_scaler*fh(2) + r_pose(2)], 'Color','k');

f = f_ranges + f_goal + f_heading;

text(10,6,'f_{goal}: ');
text(25, 6, num2str(sqrt(f_goal(2)^2 + f_goal(1)^2),5));
% Plot resultant force
fres = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f;
line([r_pose(1), plot_scaler*fres(1) + r_pose(1)],[r_pose(2), plot_scaler*fres(2) + r_pose(2)], 'Color','b');

v_desired = f/abs(f);

v = 0.1*vp + 0.9*(sqrt(f(1)^2 + f(2)^2))*dt;
om = 0.1*omp + 0.9*(atan2(f(2),f(1)))*dt;

