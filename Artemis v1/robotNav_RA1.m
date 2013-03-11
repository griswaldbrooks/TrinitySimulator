%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav_RA1(ranges)

%om = 0.6*atan(ranges(3) - ranges(1));
om = 1*(atan(ranges(3)) - atan(ranges(1)));
v = 1*exp(-(om^2))*log(mean([ranges(1),ranges(3)])/30)*log(ranges(2)/50);

