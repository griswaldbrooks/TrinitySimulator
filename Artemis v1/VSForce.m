function f_virtualSensor = VSForce(rm1, rm2, rm3)

RANGE_OFF = 20;
RANGE_COEFF = 0.25;
FMAG = 0.1;

f_rm1 = 0;
f_rm2 = 0;
f_rm3 = 0;

% Range Memory Forces
for ndx = 1:length(rm1)
    f_rm = -1/(RANGE_COEFF*((rm1(1,ndx) - RANGE_OFF)^2))*[cos(rm1(2,ndx)),sin(rm1(2,ndx))]';
    f_rm1 = f_rm1 + f_rm;
end

for ndx = 1:length(rm2)
    f_rm = -1/(RANGE_COEFF*((rm2(1,ndx) - RANGE_OFF)^2))*[cos(rm2(2,ndx)),sin(rm2(2,ndx))]';
    f_rm2 = f_rm2 + f_rm;
end

for ndx = 1:length(rm3)
    f_rm = -1/(RANGE_COEFF*((rm3(1,ndx) - RANGE_OFF)^2))*[cos(rm3(2,ndx)),sin(rm3(2,ndx))]';
    f_rm3 = f_rm3 + f_rm;
end

f_virtualSensor = FMAG*(f_rm1 + f_rm2 + f_rm3);