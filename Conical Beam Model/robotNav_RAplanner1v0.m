%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav(ranges)

om = 0.6*atan(ranges(2) - ranges(1));
v = exp(-(om^2))*log(mean([ranges(1),ranges(2)])/23);

