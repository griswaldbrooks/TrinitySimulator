%%% Robot Navigation Algorithm %%%
% Input:
%     ranges    A 1D arrary of scalar distance measurements
% Output:
%     v         Robot linear velocity in cm/sec
%     om        Robot angular velocity in radians/sec
function [v, om, rm1, rm2, poses, stuck_timer, rand_goal] = robotNav2(ranges, base_angle, r_pose, poses, stuck_timer, rand_goal, rm1, rm2, goal, vp, omp, dt)

% Constants
plot_scaler = 1000;
OM_MAX = 1;
sub_angle = 15*(pi/180);
VAR_THRESH = 5;


% Calculate Occlusion Forces
f_ranges = OccForce(ranges, base_angle, sub_angle);

% Calculate Virtual Sensor Forces
f_VSranges = VSForce(rm1, rm2);
f_VSranges = 0;

% Calculate Goal Force
goal_range = sqrt((r_pose(1) - goal(1))^2 + (r_pose(2) - goal(2))^2);
f_goal = (1/((0.5*min(goal_range,25000))^2))*[goal(1) - r_pose(1), goal(2) - r_pose(2)]';
f_goal = [cos(-r_pose(3)),-sin(-r_pose(3));sin(-r_pose(3)),cos(-r_pose(3))]*f_goal;

% Check for being Stuck
[stuck_timer, rand_goal] = CheckStuck(poses, VAR_THRESH, stuck_timer, rand_goal);

if((stuck_timer.time > 0) && (goal_range > 50))  
    goal = rand_goal;
    goal_range = sqrt((r_pose(1) - goal(1))^2 + (r_pose(2) - goal(2))^2);
    f_goal = (1/((0.5*min(goal_range,25000))^2))*[goal(1) - r_pose(1), goal(2) - r_pose(2)]';
    f_goal = [cos(-r_pose(3)),-sin(-r_pose(3));sin(-r_pose(3)),cos(-r_pose(3))]*f_goal;
    stuck_timer.time = stuck_timer.time - 1
    if(stuck_timer.time == 0)
        stuck_timer.stuck = 0;
    end
end

% Calculate Inertia
f_heading = 0.01*[cos(0),sin(0)]';

% Sum Forces
f = f_ranges + f_VSranges + f_goal + f_heading;

% Calculate Linear Velocity
v_desired = 0.5*log((min(ranges)- 10)/10)*(0.5*(tanh(goal_range - 10) + 1));
v = 0.1*vp + 0.9*v_desired;

% Calculate Angular Velocity
th_in = (atan2(f(2),f(1)));
%TURN_MAGNITUDE = 7.0*(sqrt(f(2)^2 + f(1)^2));
TURN_MAGNITUDE = 1;
om = TURN_MAGNITUDE*th_in*dt;
% Angular Saturation
if(om > OM_MAX)
    om = OM_MAX;
elseif (om < -OM_MAX)
    om = -OM_MAX;
end

% Discourage rapid changes in angular direction
% if (sign(om) ~= sign(omp))
%     disp('Flip');
%     om
%     omp
%     om = 0.6*omp + 0.4*om;
% end

% Stop at goal
if(goal_range < 10)
    v = 0;
    om = 0;
end

% Plot Forces
plotForce(f_ranges, r_pose, plot_scaler, 'g');
plotForce(f_goal, r_pose, plot_scaler, 'm');
plotForce(f_heading, r_pose, plot_scaler, 'k');
plotForce(f, r_pose, plot_scaler, 'b');
%plotVSOcc(rm1, rm2, r_pose);

% In Graph Text
text(10,6,'om: ');
text(70, 6, num2str(om));

% Update Range Memory
[rm1, rm2] = updateRM(rm1, rm2, ranges, base_angle, v, om, dt);

% Update Poses
poses = updatePoses(poses, v, om, dt);

