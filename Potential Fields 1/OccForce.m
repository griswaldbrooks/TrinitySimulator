function f_ranges = OccForce(ranges, sub_angle)
RANGE_OFF = 15;
RANGE_COEFF = 0.25;

% Right Sensor
f_range10 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 - 2*sub_angle),sin(-pi/4 - 2*sub_angle)]';
f_range11 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 - sub_angle),sin(-pi/4 - sub_angle)]';
f_range12 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4),sin(-pi/4)]';
f_range13 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 + sub_angle),sin(-pi/4 + sub_angle)]';
f_range14 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4 + 2*sub_angle),sin(-pi/4 + 2*sub_angle)]';

f_range1 = f_range10 + f_range11 + f_range12 + f_range13 + f_range14;

% Left Sensor
f_range20 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 - 2*sub_angle),sin(pi/4 - 2*sub_angle)]';
f_range21 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 - sub_angle),sin(pi/4 - sub_angle)]';
f_range22 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4),sin(pi/4)]';
f_range23 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 + sub_angle),sin(pi/4 + sub_angle)]';
f_range24 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(pi/4 + 2*sub_angle),sin(pi/4 + 2*sub_angle)]';

f_range2 = f_range20 + f_range21 + f_range22 + f_range23 + f_range24;

f_ranges = f_range1 + f_range2;