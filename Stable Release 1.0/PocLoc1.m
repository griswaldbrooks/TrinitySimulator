%%% Simple right wall follow algorithm %%%
function [v, om] = PocLoc1(laser_rp)

v = -0.2 + 0.006*laser_rp(8);
om1 = -30 + laser_rp(1);
om2 = +30 - laser_rp(7);
om = (om1+om2)/2;
om = 0.02*(om);