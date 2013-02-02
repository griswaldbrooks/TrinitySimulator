function f_virtualSensor = VSForce(rm1, rm2)

RANGE_OFF = 15;
RM_COEFF = 0.25;
FMAG = 0.05;

% Range Memory Forces
f_rm11 = -1/(RM_COEFF*((rm1(1,1) - RANGE_OFF)^2))*[cos(rm1(2,1)),sin(rm1(2,1))]';
f_rm12 = -1/(RM_COEFF*((rm1(1,2) - RANGE_OFF)^2))*[cos(rm1(2,2)),sin(rm1(2,2))]';
f_rm13 = -1/(RM_COEFF*((rm1(1,3) - RANGE_OFF)^2))*[cos(rm1(2,3)),sin(rm1(2,3))]';
f_rm14 = -1/(RM_COEFF*((rm1(1,4) - RANGE_OFF)^2))*[cos(rm1(2,4)),sin(rm1(2,4))]';
f_rm15 = -1/(RM_COEFF*((rm1(1,5) - RANGE_OFF)^2))*[cos(rm1(2,5)),sin(rm1(2,5))]';

f_rm1 = f_rm11 + f_rm12 + f_rm13 + f_rm14 + f_rm15;

f_rm21 = -1/(RM_COEFF*((rm2(1,1) - RANGE_OFF)^2))*[cos(rm2(2,1)),sin(rm2(2,1))]';
f_rm22 = -1/(RM_COEFF*((rm2(1,2) - RANGE_OFF)^2))*[cos(rm2(2,2)),sin(rm2(2,2))]';
f_rm23 = -1/(RM_COEFF*((rm2(1,3) - RANGE_OFF)^2))*[cos(rm2(2,3)),sin(rm2(2,3))]';
f_rm24 = -1/(RM_COEFF*((rm2(1,4) - RANGE_OFF)^2))*[cos(rm2(2,4)),sin(rm2(2,4))]';
f_rm25 = -1/(RM_COEFF*((rm2(1,5) - RANGE_OFF)^2))*[cos(rm2(2,5)),sin(rm2(2,5))]';

f_rm2 = f_rm21 + f_rm22 + f_rm23 + f_rm24 + f_rm25;  

f_virtualSensor = FMAG*(f_rm1 + f_rm2);