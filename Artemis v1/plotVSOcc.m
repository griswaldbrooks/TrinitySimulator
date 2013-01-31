function plotVSOcc(rm1, rm2, rm3, r_pose)

for n = 1:length(rm1)
    rm_x = rm1(1,n)*cos(rm1(2,n) + r_pose(3)) + r_pose(1);
    rm_y = rm1(1,n)*sin(rm1(2,n) + r_pose(3)) + r_pose(2);
    plot(rm_x, rm_y, 'ro');
end
for n = 1:length(rm2)
    rm_x = rm2(1,n)*cos(rm2(2,n) + r_pose(3)) + r_pose(1);
    rm_y = rm2(1,n)*sin(rm2(2,n) + r_pose(3)) + r_pose(2);
    plot(rm_x, rm_y, 'ro');
end
for n = 1:length(rm3)
    rm_x = rm3(1,n)*cos(rm3(2,n) + r_pose(3)) + r_pose(1);
    rm_y = rm3(1,n)*sin(rm3(2,n) + r_pose(3)) + r_pose(2);
    plot(rm_x, rm_y, 'ro');
end