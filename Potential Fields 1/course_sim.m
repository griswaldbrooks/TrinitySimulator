% This simulator models a differential drive robot moving in a planar
% environment. It displays the robot with some diameter, but the model does
% not take that into account, though there is some basic motion dampening
% modeled. The sensors are modeled as conical beams in the plane, returning
% the distance to the closest object.

cla, clc, clear

figure(1)
hold all
max_dim = 248; % Maximum field dimensions in centimeters
axis([-20,2*max_dim+20,-20,2*max_dim+20])
axis square

%%% FIELD WALLS %%%
field_walls = generateFieldWalls(max_dim);

%%% GOAL POSITION %%%
goal = [375,375]';
%goal = 450*rand(2,1);
rand_goal = [0,0]';
at_goal = 0;

%%% ROBOT VARIABLES %%%
HEADING_LENGTH = 25;
ROBOT_DIAMETER = 30;
%r_pose = [100,225,(-90)*(pi/180)]'; % Starting pose of the robot [x,y,theta]
r_pose = [50,450,(-90)*(pi/180)]'; % Starting pose of the robot [x,y,theta]
%r_pose = [450*rand(1),450*rand(1),r_pose(3)]';
v = 0;  % Linear Velocity, cm/sec
om = 0; % Angular Velocity, rad/sec

%%% BEAM VARIABLES %%%
VIEW_ANGLE = (60)*(pi/180);
MAX_BEAM_RANGE = 250; % 0.80 meters
MIN_BEAM_RANGE = 25; % 0.10 meters
% Beam Angles
base_angle = 45;
ang1 = -base_angle*(pi/180);
ang2 =  base_angle*(pi/180);
% Beam Angle Transformation Matricies
T1 = [cos(ang1),-sin(ang1), 0;
      sin(ang1), cos(ang1), 0;
              0,         0, 1];
T2 = [cos(ang2),-sin(ang2), 0;
      sin(ang2), cos(ang2), 0;
              0,         0, 1];

%%% RANGE MEMORY %%%
% rmn(1,x) = range
% rmn(2,x) = angle
rm1 = zeros(2,5);
rm2 = zeros(2,5);

%%% ESTIMATED POSES %%%
poses = zeros(3,100);

%%% TIMER FOR STUCK DETECTION %%%
stuck_timer = StuckTimer();

%%% PATH VECTOR %%%
path = r_pose;

%%% TIME VARIABLES %%%
dt = 0.25; % Time step
Tf = 5000; % Final Time

pause(1);

% *** ITERATE THROUGH POSITIONS *** %
for t = 0:dt:Tf
    cla
    
    %%% PLOT ROBOT %%%
    % Robot Transformation Matrix
    T = [cos(r_pose(3)),-sin(r_pose(3)), r_pose(1);
         sin(r_pose(3)), cos(r_pose(3)), r_pose(2);
                      0,              0,        1];
    plotRobot(T, ROBOT_DIAMETER, HEADING_LENGTH, 'k');
    
    %%% PLOT WALLS %%%
    for iter = 1:2:length(field_walls)
        line(field_walls(iter:iter+1,1),field_walls(iter:iter+1,2))
    end
    
    %%% PLOT GOAL %%%
    plot(goal(1), goal(2), 'm*');
    
    %%% CALCULATE RANGES %%%
    % Get ranges to walls
    [range1, feature1] = getRangeBeam(field_walls', VIEW_ANGLE, MAX_BEAM_RANGE, MIN_BEAM_RANGE, T*T1);
    [range2, feature2] = getRangeBeam(field_walls', VIEW_ANGLE, MAX_BEAM_RANGE, MIN_BEAM_RANGE, T*T2);
    
    % Add sensor noise
    range1 = range1 + 0.2*rand(1);
    range2 = range2 + 0.2*rand(1);
   
    %%% DRAW BEAM %%%
    drawBeam(VIEW_ANGLE, MAX_BEAM_RANGE, range1, T*T1, 'r');
    drawBeam(VIEW_ANGLE, MAX_BEAM_RANGE, range2, T*T2, 'r');
    
    ranges = [range1,range2];
    
    % *** Robot Navigation *** %
	vp = v;
    omp = om;
    [v, om, rm1, rm2, poses, stuck_timer, rand_goal, at_goal] = robotNav2(ranges, base_angle, r_pose, poses, stuck_timer, rand_goal, rm1, rm2, goal, at_goal, vp, omp, dt);    
    if(at_goal == 1)
        break;
    end
    
    %%% MOTION NOISE %%%
    v = v + .2*rand(1);
    om = om + .01*rand(1);
    
    %%% MOTION DAMPENING %%%
    v = 0.1*v + 0.9*vp;
    om = 0.1*om + 0.9*omp;
    
    %%% MOTION MODEL %%%
    r_pose(3) = r_pose(3) + om*dt;
    r_pose(1) = r_pose(1) + v*cos(r_pose(3));
    r_pose(2) = r_pose(2) + v*sin(r_pose(3));
    
    %%% ADD TO PATH %%%
    if(mod(t,2) == 0)
        path = [path,r_pose];
    end
    
    pause(1/256);
end

cla
%%% PLOT WALLS %%%
for iter = 1:2:length(field_walls)
    line(field_walls(iter:iter+1,1),field_walls(iter:iter+1,2))
end

%%% PLOT START %%%
plot(path(1,1), path(2,1), 'ro');
text(425,475,'Robot Start');
plot(420,475, 's', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r');

%%% PLOT GOAL %%%
plot(goal(1), goal(2), 'm*');
text(425,460,'Goal');
plot(420,460, 's', 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'm');

%%% PLOT PATH %%%
line(path(1,:), path(2,:), 'Color', 'k');
line([path(1,length(path)), goal(1)], [path(2,length(path)), goal(2)], 'Color', 'k');
text(425,445,'Path');
plot(420,445, 's', 'MarkerFaceColor', 'k', 'MarkerEdgeColor', 'k');