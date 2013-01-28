%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
%               index 1: Right Beam
%               index 2: Left Beam
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om] = robotNav_RAM1(ranges, rangesp, vp, omp)

% Angular Velocity coefficient
% If there is a nearby occlusion, turn harder
% Slower turning seems to keep the robot from getting too close to corners
% and edges when turning
% Turn randomization takes affect when there is a large difference between
% beam measurements
rand_turn = 2/(1 + exp((ranges(1) - ranges(2))^2 - 20))*exp(-0.1*min(ranges));
C_om = 0.10 + rand_turn;

% Turn towards free space, velocity limited
% Bias towards left
turn_bias = 1/(1 + exp(-2*(rand(1) - 0.5)));
turn_open = atan(turn_bias*ranges(2) - (1 - turn_bias)*ranges(1));

% Dampen large changes in range
% Large differences will scale down angular velocity
leftDamp  = exp(-0.01*(rangesp(2) - ranges(2))^2);
rightDamp = exp(-0.01*(rangesp(1) - ranges(1))^2);

% Calculate Angular Velocity
om = C_om*turn_open*rightDamp*leftDamp;

% Linear Velocity coefficient
C_v = 5;

% Use average of left and right distances to determine how free the space
% is in front of the robot. If free, go faster, with limited velocity. If
% occluded, back up.
% The equilibrium point is where the velocity is zero. If the distance
% average is large compared to the equilibrium point, then the velocity
% will be large. If small, velocity will be negative.

% Shift the equlibrium point to the right, the faster the robot is going,
% requiring a longer average distance for the resultant velocity to be
% positive. If the robot is going slowly (vp < 1), shift equilibrium point 
% quickly to the left, to get the robot to speed up. The constant is the
% minimum of the equilibrium point to ensure the robot does not get too
% close.
eq_point = 200*vp^4 + 100*exp(-0.1*min(ranges - 10)^2) + 23
free_go_forward = log(mean([ranges(1),ranges(2)])/eq_point)

% If Angular Velocity is large, Linear Velocity -> small
% (turning fast, go slow)
slow_if_turn = exp(-10*(om^2))

% Calculate Linear Velocity
v = C_v*slow_if_turn*free_go_forward

% Weights for Linear and Angular Velocity
% If the difference between the current and previous velocities are large,
% favor the previous velocity
%wv  = exp(-1.0*(v - vp)^2);
%wom = exp(-1.0*(om - omp)^2);

% Calculate weighted velocity
%v = wv*v + (1 - wv)*vp
%om = wom*om + (1 - wom)*omp;


