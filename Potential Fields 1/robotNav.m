%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav(ranges, r_pose, goal, vp, omp, dt)

plot_scaler = 1000;

% Magnitude 1/r^2
f_ranges = [];

goal_range = sqrt((r_pose(1) - goal(1))^2 + (r_pose(2) - goal(2))^2);
f_goal = (goal_range^-2)*[goal(1) - r_pose(1), goal(2) - r_pose(2)]'
% Plot goal force

line([r_pose(1), plot_scaler*f_goal(1) + r_pose(1)],[r_pose(2), plot_scaler*f_goal(2) + r_pose(2)], 'Color','m');
f_heading = [];

%f = f_ranges + f_goal + f_heading;
f = f_goal;

v_desired = f/abs(f);

v = 0.25*vp + 0.75*(sqrt(f(1)^2 + f(2)^2))*dt;
om = 0.25*omp + 0.75*(atan2(f(2),f(1)) - r_pose(3))*dt;
