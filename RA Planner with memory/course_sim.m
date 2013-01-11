% This simulator models a differential drive robot moving in a planar
% environment. It displays the robot with some diameter, but the model does
% not take that into account, though there is some basic motion dampening
% modeled. The sensors are modeled as conical beams in the plane, returning
% the distance to the closest object.

cla, clc, clear

figure(1)
hold all
max_dim = 248; % Maximum field dimensions in centimeters
axis([-20,max_dim+20,-20,max_dim+20])
axis square

%%% FIELD WALLS %%%
field_walls = generateFieldWalls(max_dim);

%%% ROBOT VARIABLES %%%
HEADING_LENGTH = 25;
ROBOT_DIAMETER = 30;
r_pose = [100,225,(-90)*(pi/180)]'; % Starting pose of the robot [x,y,theta]
v = 0.000001;  % Linear Velocity, cm/sec
om = 0; % Angular Velocity, rad/sec

%%% BEAM VARIABLES %%%
VIEW_ANGLE = (60)*(pi/180);
MAX_BEAM_RANGE = 250; % 0.80 meters
MIN_BEAM_RANGE = 25; % 0.10 meters
% Beam Angles
ang1 = (-45)*(pi/180);
ang2 = (45)*(pi/180);
% Beam Angle Transformation Matricies
T1 = [cos(ang1),-sin(ang1), 0;
      sin(ang1), cos(ang1), 0;
              0,         0, 1];
T2 = [cos(ang2),-sin(ang2), 0;
      sin(ang2), cos(ang2), 0;
              0,         0, 1];
%Range vectors
ranges = [0,0];
rangesp = ranges;
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
    
    rangesp = ranges;
    ranges = [range1,range2];
    
    % *** Robot Navigation *** %
	vp = v;
    omp = om;
    [v, om] = robotNav(ranges, rangesp, v, om);    
    
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
    
    pause(1/256);
end