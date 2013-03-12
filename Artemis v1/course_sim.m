% This simulator models a differential drive robot moving in a planar
% environment. It displays the robot with some diameter, but the model does
% not take that into account, though there is some basic motion dampening
% modeled. The sensors are modeled as conical beams in the plane, returning
% the distance to the closest object.

cla, clc, clear all, clf

subplot(1,2,1);
hold all
max_dim = 248; % Maximum field dimensions in centimeters
axis([-20,max_dim+20,-20,max_dim+20])
axis square

%%% FIELD WALLS %%%
%%% Field Walls are set up here as a list of line segments %%%
% Wall structure [x1,y1;x2,y2] end points of line segment
outer_wall1 = [0,0;0,max_dim];
outer_wall2 = [0,0;max_dim,0];
outer_wall3 = [0,max_dim;max_dim,max_dim];
outer_wall4 = [max_dim,0;max_dim,max_dim];
outer_walls = [outer_wall1;outer_wall2;outer_wall3;outer_wall4];

island_wall1 = [202,137;202,202];
island_wall2 = [118,137;118,202];
island_wall3 = [118,202;202,202];
island_wall4 = [164,137;202,137];
island_walls = [island_wall1;island_wall2;island_wall3;island_wall4];

lr_wall1 = [118,91;200,91];
lr_wall2 = [118,45;118,0];
lr_walls = [lr_wall1;lr_wall2];

ll_wall1 = [0,103;72,103];
ll_wall2 = [72,103;72,46];
ll_walls = [ll_wall1;ll_wall2];

ur_wall1 = [72,157;72,max_dim];
ur_wall2 = [46,157;72,157];
ur_walls = [ur_wall1;ur_wall2];

field_walls = [outer_walls;island_walls;lr_walls;ll_walls;ur_walls];

%%% ROBOT VARIABLES %%%
HEADING_LENGTH = 25;
ROBOT_DIAMETER = 25;
r_pose = [100,225,(-90)*(pi/180)]'; % Starting pose of the robot [x,y,theta]
v = 0;  % Linear Velocity, cm/sec
om = 0; % Angular Velocity, rad/sec

%%% BEAM VARIABLES %%%
VIEW_ANGLE = (5)*(pi/180);
MAX_BEAM_RANGE = 80; % 0.80 meters
MIN_BEAM_RANGE = 10; % 0.10 meters
% Beam Angles
ang1 = (-45)*(pi/180);
ang2 = (  0)*(pi/180);
ang3 = ( 45)*(pi/180);
% Beam Angle Transformation Matricies
T1 = [cos(ang1),-sin(ang1), 0;
      sin(ang1), cos(ang1), 0;
              0,         0, 1];
T2 = [cos(ang2),-sin(ang2), 0;
      sin(ang2), cos(ang2), 0;
              0,         0, 1];
T3 = [cos(ang3),-sin(ang3), 0;
      sin(ang3), cos(ang3), 0;
              0,         0, 1];

%%% RANGE MEMORY %%%
% rmn(1,x) = range
% rmn(2,x) = angle
rm1 = zeros(2,10);
rm2 = zeros(2,10);
rm3 = zeros(2,10);

%%% TIME VARIABLES %%%
dt = 0.25; % Time step
Tf = 5000; % Final Time

pause(1);

% *** ITERATE THROUGH POSITIONS *** %
for t = 0:dt:Tf
    subplot(1,2,1);
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
    [range3, feature3] = getRangeBeam(field_walls', VIEW_ANGLE, MAX_BEAM_RANGE, MIN_BEAM_RANGE, T*T3);
    
    % Add sensor noise
    range1 = range1 + 0.2*rand(1);
    range2 = range2 + 0.2*rand(1);
    range3 = range3 + 0.2*rand(1);
    %%% DRAW BEAM %%%
    drawBeam(VIEW_ANGLE, MAX_BEAM_RANGE, range1, T*T1, 'r');
    drawBeam(VIEW_ANGLE, MAX_BEAM_RANGE, range2, T*T2, 'r');
    drawBeam(VIEW_ANGLE, MAX_BEAM_RANGE, range3, T*T3, 'r');
    
    ranges = [range1,range2,range3];
    
    % *** Robot Navigation *** %
	vp = v;
    omp = om;
    %[v, om, rm1, rm2, rm3] = robotNav_PF1(ranges, rm1, rm2, rm3, r_pose, vp, omp, dt);    
    %[v, om] = robotNav_RAM1(ranges, rangesp, vp, omp);
    %[v, om] = robotNav_RA1(ranges);
    [v, om] = robotNav_lmap1(ranges, dt);
    
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