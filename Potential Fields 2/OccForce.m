function f_ranges = OccForce(ranges, base_angle, sub_angle)
RANGE_OFF = 15;
RANGE_COEFF = 0.25;

% Right Sensor
f_range10 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-base_angle - 2*sub_angle),sin(-base_angle - 2*sub_angle)]';
f_range11 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-base_angle - sub_angle),sin(-base_angle - sub_angle)]';
f_range12 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-base_angle),sin(-base_angle)]';
f_range13 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-base_angle + sub_angle),sin(-base_angle + sub_angle)]';
f_range14 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-base_angle + 2*sub_angle),sin(-base_angle + 2*sub_angle)]';

f_range1 = f_range10 + f_range11 + f_range12 + f_range13 + f_range14;
%f_range1 = f_range10 + f_range11 + f_range12;
%f_range1 = f_range12;

% Left Sensor
f_range20 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(base_angle - 2*sub_angle),sin(base_angle - 2*sub_angle)]';
f_range21 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(base_angle - sub_angle),sin(base_angle - sub_angle)]';
f_range22 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(base_angle),sin(base_angle)]';
f_range23 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(base_angle + sub_angle),sin(base_angle + sub_angle)]';
f_range24 = -1/(RANGE_COEFF*((ranges(2) - RANGE_OFF)^2))*[cos(base_angle + 2*sub_angle),sin(base_angle + 2*sub_angle)]';

f_range2 = f_range20 + f_range21 + f_range22 + f_range23 + f_range24;
%f_range2 = f_range22 + f_range23 + f_range24;
%f_range2 = f_range22;

f_ranges = f_range1 + f_range2;