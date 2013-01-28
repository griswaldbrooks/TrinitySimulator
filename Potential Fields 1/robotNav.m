%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om, rm1, rm2] = robotNav(ranges, rm1, rm2, r_pose, goal, vp, omp, dt)

plot_scaler = 1000;
RANGE_OFF = 20;
RANGE_COEFF = 0.25;

% Magnitude 1/r^2
f_range1 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4),sin(-pi/4)]';
f_range2 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos( pi/4),sin( pi/4)]';

% Range Memory Forces
f_rm11 = -1/(RANGE_COEFF*((rm1(1,1) - RANGE_OFF)^2))*[cos(rm1(2,1)),sin(rm1(2,1))]';
f_rm12 = -1/(RANGE_COEFF*((rm1(1,2) - RANGE_OFF)^2))*[cos(rm1(2,2)),sin(rm1(2,2))]';
f_rm13 = -1/(RANGE_COEFF*((rm1(1,3) - RANGE_OFF)^2))*[cos(rm1(2,3)),sin(rm1(2,3))]';
f_rm14 = -1/(RANGE_COEFF*((rm1(1,4) - RANGE_OFF)^2))*[cos(rm1(2,4)),sin(rm1(2,4))]';
f_rm15 = -1/(RANGE_COEFF*((rm1(1,5) - RANGE_OFF)^2))*[cos(rm1(2,5)),sin(rm1(2,5))]';

f_rm1 = f_rm11 + f_rm12 + f_rm13 + f_rm14 + f_rm15;

f_rm21 = -1/(RANGE_COEFF*((rm2(1,1) - RANGE_OFF)^2))*[cos(rm2(2,1)),sin(rm2(2,1))]';
f_rm22 = -1/(RANGE_COEFF*((rm2(1,2) - RANGE_OFF)^2))*[cos(rm2(2,2)),sin(rm2(2,2))]';
f_rm23 = -1/(RANGE_COEFF*((rm2(1,3) - RANGE_OFF)^2))*[cos(rm2(2,3)),sin(rm2(2,3))]';
f_rm24 = -1/(RANGE_COEFF*((rm2(1,4) - RANGE_OFF)^2))*[cos(rm2(2,4)),sin(rm2(2,4))]';
f_rm25 = -1/(RANGE_COEFF*((rm2(1,5) - RANGE_OFF)^2))*[cos(rm2(2,5)),sin(rm2(2,5))]';

f_rm2 = f_rm21 + f_rm22 + f_rm23 + f_rm24 + f_rm25;  

% Plot occlusion force components
% Rotate for graph
fr1 = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_range1;
fr2 = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_range2;
line([r_pose(1), plot_scaler*fr1(1) + r_pose(1)],[r_pose(2), plot_scaler*fr1(2) + r_pose(2)], 'Color','y');
line([r_pose(1), plot_scaler*fr2(1) + r_pose(1)],[r_pose(2), plot_scaler*fr2(2) + r_pose(2)], 'Color','y');

for n = 1:5
    rm_x = rm1(1,n)*cos(rm1(2,n) + r_pose(3)) + r_pose(1);
    rm_y = rm1(1,n)*sin(rm1(2,n) + r_pose(3)) + r_pose(2);
    plot(rm_x, rm_y, 'ro');
end
for n = 1:5
    rm_x = rm2(1,n)*cos(rm2(2,n) + r_pose(3)) + r_pose(1);
    rm_y = rm2(1,n)*sin(rm2(2,n) + r_pose(3)) + r_pose(2);
    plot(rm_x, rm_y, 'ro');
end

f_ranges = f_range1 + f_range2 + f_rm1 + f_rm2;
% Plot occlusion force
frs = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_ranges;
line([r_pose(1), plot_scaler*frs(1) + r_pose(1)],[r_pose(2), plot_scaler*frs(2) + r_pose(2)], 'Color','g');

goal_range = sqrt((r_pose(1) - goal(1))^2 + (r_pose(2) - goal(2))^2);
f_goal = 1/((0.5*goal_range)^2)*[goal(1) - r_pose(1), goal(2) - r_pose(2)]';
% Plot goal force
line([r_pose(1), plot_scaler*f_goal(1) + r_pose(1)],[r_pose(2), plot_scaler*f_goal(2) + r_pose(2)], 'Color','m');
goal_angle = -r_pose(3) + atan2(f_goal(2),f_goal(1));
f_goal = [cos(goal_angle),-sin(goal_angle);sin(goal_angle),cos(goal_angle)]*f_goal;

f_heading = 0.01*[cos(0),sin(0)]';
% Plot current heading force
fh = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_heading;
line([r_pose(1), plot_scaler*fh(1) + r_pose(1)],[r_pose(2), plot_scaler*fh(2) + r_pose(2)], 'Color','k');

f = f_ranges + f_goal + f_heading;

text(10,6,'r_{min}: ');
text(25, 6, num2str(min(ranges)));
% Plot resultant force
fres = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f;
line([r_pose(1), plot_scaler*fres(1) + r_pose(1)],[r_pose(2), plot_scaler*fres(2) + r_pose(2)], 'Color','b');

v_desired = 0.5*log((min(ranges)- 10)/10)*(0.5*(tanh(goal_range - 10) + 1));

v = 0.1*vp + 0.9*v_desired;
%v = 0.1*vp + 0.9*(sqrt(f(1)^2 + f(2)^2))*dt;
om = 0.1*omp + 0.9*(atan2(f(2),f(1)))*dt;

%%% CHANGE IN MOTION MODEL %%%
dx = sqrt((v*cos(om*dt))^2 + (v*sin(om*dt))^2);
%%% UPDATE RANGE MEMORY %%%
rm1(1,5) = sqrt( rm1(1,4)^2 + dx^2 - 2*rm1(1,4)*dx*cos(rm1(2,4)));
rm1(2,5) = rm1(2,4) - om*dt;
rm1(1,4) = sqrt( rm1(1,3)^2 + dx^2 - 2*rm1(1,3)*dx*cos(rm1(2,3)));
rm1(2,4) = rm1(2,3) - om*dt;
rm1(1,3) = sqrt( rm1(1,2)^2 + dx^2 - 2*rm1(1,2)*dx*cos(rm1(2,2)));
rm1(2,3) = rm1(2,2) - om*dt;
rm1(1,2) = sqrt( rm1(1,1)^2 + dx^2 - 2*rm1(1,1)*dx*cos(rm1(2,1)));
rm1(2,2) = rm1(2,1) - om*dt;
rm1(1,1) = sqrt( ranges(1)^2 + dx^2 - 2*ranges(1)*dx*cos(-pi/4));
rm1(2,1) = -pi/4 - om*dt;

rm2(1,5) = sqrt( rm2(1,4)^2 + dx^2 - 2*rm2(1,4)*dx*cos(rm2(2,4)));
rm2(2,5) = rm2(2,4) - om*dt;
rm2(1,4) = sqrt( rm2(1,3)^2 + dx^2 - 2*rm2(1,3)*dx*cos(rm2(2,3)));
rm2(2,4) = rm2(2,3) - om*dt;
rm2(1,3) = sqrt( rm2(1,2)^2 + dx^2 - 2*rm2(1,2)*dx*cos(rm2(2,2)));
rm2(2,3) = rm2(2,2) - om*dt;
rm2(1,2) = sqrt( rm2(1,1)^2 + dx^2 - 2*rm2(1,1)*dx*cos(rm2(2,1)));
rm2(2,2) = rm2(2,1) - om*dt;
rm2(1,1) = sqrt( ranges(2)^2 + dx^2 - 2*ranges(2)*dx*cos(pi/4));
rm2(2,1) = pi/4 - om*dt;