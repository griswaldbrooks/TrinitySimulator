This simulator models a differential drive robot moving in a planar
environment. It displays the robot with some diameter, but the model does
not take that into account, though there is some basic motion dampening
modeled. The sensors are modeled as conical beams in the plane, returning
the distance to the closest object.

To modify the navigation algorithm, edit robotNav.m

There are several sections to the simulator code. Some have to be configured by the 
user.

%%% FIELD WALLS %%%
Field Walls are set up as a vector of line segments. If the course layout 
needs to be changed, it is done here.
Wall structure:
	[x1,y1;x2,y2]
These are the end points of the line segments.

%%% ROBOT VARIABLES %%%
This section contains constants and initializations for the robot. 

%%% BEAM VARIABLES %%%
This section contains constants for the distance sensors. To add more sensors, 
additional angles (angn) and transformation matricies (Tn) must be added, as
well as to the CALCULATE RANGES section. n is the number of the new beam.
To add a new transformation matrix Tn, the format is:
Tn = [cos(angn),-sin(angn), 0;
      sin(angn), cos(angn), 0;
              0,         0, 1];

%%% TIME VARIABLES %%%
This section contains constants for the simulation run time.

% *** ITERATE THROUGH POSITIONS *** %
This section contains the simulation loop
   
%%% PLOT ROBOT %%%
This section plots the robot.

%%% PLOT WALLS %%%
This section plots the walls
    
%%% CALCULATE RANGES %%%
This section is where the distance measurements are calculated. To add new sensors,
modify the following subsections
% Get ranges to walls
	Add a line where Tn is the constant added in BEAM VARIABLES, and [rangen, featuren]
	are returns containing the range to the closest object and the closest feature (wall).
	The featuren does not need to be used.
	
    [rangen, featuren] = getRangeBeam(field_walls', VIEW_ANGLE, MAX_BEAM_RANGE, MIN_BEAM_RANGE, T*Tn);
    
% Add sensor noise
	Add a line where rangen is the returned range from the previous subsection.
    rangen = rangen + 0.2*rand(1);

%%% DRAW BEAM %%%
	Add a line where rangen is the returned range from the previous subsection and
	Tn is the constant added in BEAM VARIABLES.
    drawBeam(VIEW_ANGLE, MAX_BEAM_RANGE, rangen, T*Tn, 'r');

The new range value needs to be added to the ranges vector.
	ranges = [range1,range2,range3, rangen];
    
% *** Potential Field Navigation *** %
This section is where the robot movement is calculated and executed. The file robotNav.m
should be modified to change the navigation algorithm. 
