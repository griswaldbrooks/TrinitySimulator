%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav(ranges)


v = log(ranges(2)/30);
om = 0.4*atan(ranges(3) - ranges(1));
