%%% Simple right wall follow algorithm %%%
function [v, om] = PocLoc1(laser_rp)

%v = 0.006*(-23 + laser_rp(8));
v = log(laser_rp(8)/23);
%om1 = atan(laser_rp(8) - laser_rp(1));
%om2 = atan(laser_rp(7) - laser_rp(8));
%om = (om1+om2);
om = atan(laser_rp(1) - laser_rp(7));
om = 0.2*(om);