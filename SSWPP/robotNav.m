%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
%               index 1: Right Beam
%               index 2: Left Beam
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om, map] = robotNav(ranges, rangesp, vp, omp, view_angle, r_pose, T1, T2, map)

om = 0.6*atan(ranges(2) - ranges(1));
v = exp(-(om^2))*log(mean([ranges(1),ranges(2)])/23);

R = [cos(r_pose(3)),-sin(r_pose(3)); sin(r_pose(3)),cos(r_pose(3))];

map = zeros(length(map));
if(ranges(1) < 98)
    for dth = (-0.5*view_angle):0.05:(0.5*view_angle)
        rx1 = ranges(1)*cos(dth);
        ry1 = ranges(1)*sin(dth);
        
        p1 = (R*T1(1:2,1:2))*[rx1;ry1];
        p1(1) = floor(p1(1)/2);
        p1(2) = floor(p1(2)/2);
        
        map(p1(1) + 50, p1(2) + 50) = 1;
    end
end
if(ranges(2) < 98)
    for dth = (-0.5*view_angle):0.05:(0.5*view_angle)
        rx2 = ranges(2)*cos(dth);
        ry2 = ranges(2)*sin(dth);
        
        p2 = (R*T2(1:2,1:2))*[rx2;ry2];
        p2(1) = floor(p2(1)/2);
        p2(2) = floor(p2(2)/2);
        
        map(p2(1) + 50, p2(2) + 50) = 1;
    end
end
