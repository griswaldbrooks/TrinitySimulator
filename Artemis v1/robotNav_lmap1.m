%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav_lmap1(ranges)

% Map Scaler
m_sc = 0.5;

% Local Map
persistent lmap;
if(isempty(lmap))
    lmap = zeros(100);
end

% Robot position within map
r_pose = [length(lmap)/2, length(lmap)/2, 0]';

% Occlusion positions relative to robot
occ1_r = [ranges(1)*cos(-45*(pi/180) + r_pose(3)), ranges(1)*sin(-45*(pi/180) + r_pose(3))]';
occ2_r = [ranges(2)*cos(  0*(pi/180) + r_pose(3)), ranges(2)*sin(  0*(pi/180) + r_pose(3))]';
occ3_r = [ranges(3)*cos( 45*(pi/180) + r_pose(3)), ranges(3)*sin( 45*(pi/180) + r_pose(3))]';

% Transformed occlusion positions in map
occ1 = floor(m_sc*occ1_r) + r_pose(1:2,:);
occ2 = floor(m_sc*occ2_r) + r_pose(1:2,:);
occ3 = floor(m_sc*occ3_r) + r_pose(1:2,:);

% Insert points into map
lmap(occ1(1), occ1(2)) = 1;
lmap(occ2(1), occ2(2)) = 1;
lmap(occ3(1), occ3(2)) = 1;

% Plot map
surf(lmap);

%om = 0.6*atan(ranges(3) - ranges(1));
om = 1*(atan(ranges(3)) - atan(ranges(1)));
v = 1*exp(-(om^2))*log(mean([ranges(1),ranges(3)])/30)*log(ranges(2)/50);

