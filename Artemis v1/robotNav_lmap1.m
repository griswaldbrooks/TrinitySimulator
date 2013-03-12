%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav_lmap1(ranges, dt)

% Map Scaler
m_sc = 0.5;
% Max Beam Distance
RANGE_MAX = 70;
% Decay Rate
dc_rt = 0.995;

% Local Map
persistent lmap;
if(isempty(lmap))
    lmap = zeros(100);
end

% Robot position within map
persistent r_pose;
if(isempty(r_pose))
    r_pose = [length(lmap)/2, length(lmap)/2, 0]';
end

% Previous velocity
persistent vp;
if(isempty(vp))
    vp = [0,0]';
    % vp(1): Linear Velocity
    % vp(2): Angular Velocity
end

% Map translator
persistent m_tr;
if(isempty(m_tr))
    m_tr = [0,0]';
end

% Update robot orientation
r_pose(3) = r_pose(3) + vp(2)*dt;
% Update map translation
m_tr = m_tr + [vp(1)*cos(r_pose(3)), vp(1)*sin(r_pose(3))]'

% Translate the map in the x direction
if(m_tr(1) > 1/m_sc)
    for ndx = 1:(length(lmap)-1)
        lmap(ndx,:) = lmap(ndx + 1, :);
    end
    m_tr(1) = m_tr(1) - 1/m_sc;
elseif(m_tr(1) < -1/m_sc)
    for ndx = length(lmap):2
        lmap(ndx,:) = lmap(ndx - 1, :);
    end
    m_tr(1) = m_tr(1) + 1/m_sc;    
end

% Translate the map in the y direction
if(m_tr(2) > 1/m_sc)
    for ndx = length(lmap):2
        lmap(:,ndx) = lmap(:,ndx - 1);
    end
    m_tr(2) = m_tr(2) - 1/m_sc;
elseif(m_tr(2) < -1/m_sc)
    for ndx = 1:(length(lmap)-1)
        lmap(:,ndx) = lmap(:,ndx + 1);
    end
    m_tr(2) = m_tr(2) + 1/m_sc;    
end

lmap = dc_rt*lmap;

% Occlusion positions relative to robot
occ1_r = [ranges(1)*cos( 45*(pi/180) - r_pose(3)), ranges(1)*sin( 45*(pi/180) - r_pose(3))]';
occ2_r = [ranges(2)*cos(  0*(pi/180) - r_pose(3)), ranges(2)*sin(  0*(pi/180) - r_pose(3))]';
occ3_r = [ranges(3)*cos(-45*(pi/180) - r_pose(3)), ranges(3)*sin(-45*(pi/180) - r_pose(3))]';

% Transformed occlusion positions in map
occ1 = floor(m_sc*occ1_r) + r_pose(1:2,:);
occ2 = floor(m_sc*occ2_r) + r_pose(1:2,:);
occ3 = floor(m_sc*occ3_r) + r_pose(1:2,:);

% Insert points into map
if(ranges(1) < RANGE_MAX)
    lmap(occ1(1), occ1(2)) = 10;
end
if(ranges(2) < RANGE_MAX)
    lmap(occ2(1), occ2(2)) = 10;
end
if(ranges(3) < RANGE_MAX)
    lmap(occ3(1), occ3(2)) = 10;
end

% Plot map
subplot(1,2,2);
surf(lmap);
set(gca, 'View', [0,90]);
axis square

%om = 0.6*atan(ranges(3) - ranges(1));
om = 20*(atan(ranges(3)) - atan(ranges(1)));
v = 5*exp(-(om^2))*log(mean([ranges(1),ranges(3)])/30)*log(ranges(2)/50);

vp = [v,om]';


