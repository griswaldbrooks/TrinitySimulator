function f_ranges = OccForce(ranges)
RANGE_OFF = 20;
RANGE_COEFF = 0.25;

% Right Sensor
f_range1 = -1/(RANGE_COEFF*((ranges(1) - RANGE_OFF)^2))*[cos(-pi/4),sin(-pi/4)]';
% Center Sensor
f_range2 = -1/(RANGE_COEFF*((ranges(2) - (RANGE_OFF + 10))^2))*[cos(    0),sin(    0)]';
% Left Sensor
f_range3 = -1/(RANGE_COEFF*((ranges(3) - RANGE_OFF)^2))*[cos( pi/4),sin( pi/4)]';

f_ranges = f_range1 + f_range2 + f_range3;