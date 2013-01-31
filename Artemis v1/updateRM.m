function [rm1, rm2, rm3] = updateRM(rm1, rm2, rm3, ranges, v, om, dt)
om = 1*om;
%%% CHANGE IN MOTION MODEL %%%
dx = v*dt;
dy = 0;
H = T_2D(dx, dy, om*dt);
%%% UPDATE RANGE MEMORY %%%
for ndx = length(rm1):-1:2
    rxn = rm1(1,ndx-1)*cos(rm1(2,ndx-1));
    ryn = rm1(1,ndx-1)*sin(rm1(2,ndx-1));
    rpn = [rxn, ryn, 1]';
    rpn = H\rpn;
    rm1(1,ndx) = sqrt(rpn(1)^2 + rpn(2)^2);
    rm1(2,ndx) = atan2(rpn(2), rpn(1));
end
rx1 = ranges(1)*cos(-pi/4);
ry1 = ranges(1)*sin(-pi/4);
rp1 = [rx1,ry1,1]';
rp1 = H\rp1;
rm1(1,1) = sqrt((rp1(1))^2 + (rp1(2))^2);
rm1(2,1) = atan2(rp1(2), rp1(1));


for ndx = length(rm2):-1:2
    rxn = rm2(1,ndx-1)*cos(rm2(2,ndx-1));
    ryn = rm2(1,ndx-1)*sin(rm2(2,ndx-1));
    rpn = [rxn, ryn, 1]';
    rpn = H\rpn;
    rm2(1,ndx) = sqrt(rpn(1)^2 + rpn(2)^2);
    rm2(2,ndx) = atan2(rpn(2), rpn(1));
end
rx2 = ranges(2)*cos(0);
ry2 = ranges(2)*sin(0);
rp2 = [rx2,ry2,1]';
rp2 = H\rp2;
rm2(1,1) = sqrt((rp2(1))^2 + (rp2(2))^2);
rm2(2,1) = atan2(rp2(2), rp2(1));

for ndx = length(rm3):-1:2
    rxn = rm3(1,ndx-1)*cos(rm3(2,ndx-1));
    ryn = rm3(1,ndx-1)*sin(rm3(2,ndx-1));
    rpn = [rxn, ryn, 1]';
    rpn = H\rpn;
    rm3(1,ndx) = sqrt(rpn(1)^2 + rpn(2)^2);
    rm3(2,ndx) = atan2(rpn(2), rpn(1));
end
rx3 = ranges(3)*cos(pi/4);
ry3 = ranges(3)*sin(pi/4);
rp3 = [rx3,ry3,1]';
rp3 = H\rp3;
rm3(1,1) = sqrt((rp3(1))^2 + (rp3(2))^2);
rm3(2,1) = atan2(rp3(2), rp3(1));