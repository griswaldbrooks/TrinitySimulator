%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om, rm1, rm2, rm3] = robotNav(ranges, rm1, rm2, rm3, r_pose, vp, omp, dt)

plot_scaler = 1000;
RANGE_OFF = 15;
RANGE_COEFF = 0.05;
% Magnitude 1/r^2
f_range1 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4),sin(-pi/4)]';
f_range2 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(    0),sin(    0)]';
f_range3 = -1/(RANGE_COEFF*((ranges(3) - RANGE_OFF)^2))*[cos( pi/4),sin( pi/4)]';

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

f_rm31 = -1/(RANGE_COEFF*((rm3(1,1) - RANGE_OFF)^2))*[cos(rm3(2,1)),sin(rm3(2,1))]';
f_rm32 = -1/(RANGE_COEFF*((rm3(1,2) - RANGE_OFF)^2))*[cos(rm3(2,2)),sin(rm3(2,2))]';
f_rm33 = -1/(RANGE_COEFF*((rm3(1,3) - RANGE_OFF)^2))*[cos(rm3(2,3)),sin(rm3(2,3))]';
f_rm34 = -1/(RANGE_COEFF*((rm3(1,4) - RANGE_OFF)^2))*[cos(rm3(2,4)),sin(rm3(2,4))]';
f_rm35 = -1/(RANGE_COEFF*((rm3(1,5) - RANGE_OFF)^2))*[cos(rm3(2,5)),sin(rm3(2,5))]';

f_rm3 = f_rm31 + f_rm32 + f_rm33 + f_rm34 + f_rm35;  

%%% Plot occlusion force components %%%
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
for n = 1:5
    rm_x = rm3(1,n)*cos(rm3(2,n) + r_pose(3)) + r_pose(1);
    rm_y = rm3(1,n)*sin(rm3(2,n) + r_pose(3)) + r_pose(2);
    plot(rm_x, rm_y, 'ro');
end

%%% %%%

f_ranges = f_range1 + f_range2 + f_range3 + f_rm1 + f_rm2 + f_rm3;
%%% Plot occlusion force %%%
frs = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_ranges;
line([r_pose(1), plot_scaler*frs(1) + r_pose(1)],[r_pose(2), plot_scaler*frs(2) + r_pose(2)], 'Color','g');
%%% %%%

f_heading = 2*[cos(0),sin(0)]';
%%% Plot current heading force %%%
fh = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f_heading;
line([r_pose(1), plot_scaler*fh(1) + r_pose(1)],[r_pose(2), plot_scaler*fh(2) + r_pose(2)], 'Color','k');
%%% %%%

f = f_ranges + f_heading;
%%% Plot resultant force %%%
text(10,6,'r_{min}: ');
text(25, 6, num2str(min(ranges)));
fres = [cos(r_pose(3)),-sin(r_pose(3));sin(r_pose(3)),cos(r_pose(3))]*f;
line([r_pose(1), plot_scaler*fres(1) + r_pose(1)],[r_pose(2), plot_scaler*fres(2) + r_pose(2)], 'Color','b');
%%% %%%

v_desired = 0.5*log((min(ranges)- 10)/10);

v = 0.1*vp + 0.9*v_desired;
om = 0.1*omp + 0.9*(atan2(f(2),f(1)))*dt;

%%% CHANGE IN MOTION MODEL %%%
dx = v*dt;
dy = 0;
H = T_2D(dx, dy, om*dt);
%%% UPDATE RANGE MEMORY %%%
for ndx = 5:-1:2
    rxn = rm1(1,ndx-1)*cos(rm1(2,ndx-1));
    ryn = rm1(1,ndx-1)*sin(rm1(2,ndx-1));
    rpn = [rxn, ryn, 1]';
    rpn = H\rpn;
    rm1(1,ndx) = sqrt(rpn(1)^2 + rpn(2)^2);
    rm1(2,ndx) = atan2(rpn(2), rpn(1));
end
rx1 = ranges(1)*cos(-pi/4);
ry1 = ranges(1)*sin(-pi/4);
rp1 = [rx1,ry1,1]';
rp1 = H\rp1;
rm1(1,1) = sqrt((rp1(1))^2 + (rp1(2))^2);
rm1(2,1) = atan2(rp1(2), rp1(1));


for ndx = 5:-1:2
    rxn = rm2(1,ndx-1)*cos(rm2(2,ndx-1));
    ryn = rm2(1,ndx-1)*sin(rm2(2,ndx-1));
    rpn = [rxn, ryn, 1]';
    rpn = H\rpn;
    rm2(1,ndx) = sqrt(rpn(1)^2 + rpn(2)^2);
    rm2(2,ndx) = atan2(rpn(2), rpn(1));
end
rx2 = ranges(2)*cos(0);
ry2 = ranges(2)*sin(0);
rp2 = [rx2,ry2,1]';
rp2 = H\rp2;
rm2(1,1) = sqrt((rp2(1))^2 + (rp2(2))^2);
rm2(2,1) = atan2(rp2(2), rp2(1));



for ndx = 5:-1:2
    rxn = rm3(1,ndx-1)*cos(rm3(2,ndx-1));
    ryn = rm3(1,ndx-1)*sin(rm3(2,ndx-1));
    rpn = [rxn, ryn, 1]';
    rpn = H\rpn;
    rm3(1,ndx) = sqrt(rpn(1)^2 + rpn(2)^2);
    rm3(2,ndx) = atan2(rpn(2), rpn(1));
end
rx3 = ranges(3)*cos(pi/4);
ry3 = ranges(3)*sin(pi/4);
rp3 = [rx3,ry3,1]';
rp3 = H\rp3;
rm3(1,1) = sqrt((rp3(1))^2 + (rp3(2))^2);
rm3(2,1) = atan2(rp3(2), rp3(1));