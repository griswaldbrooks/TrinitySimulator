function [field_walls] = generateFieldWalls(max_dim)

%%% Field Walls are set up here as a list of line segments %%%
% Wall structure [x1,y1;x2,y2] end points of line segment
outer_wall1 = [0,0;0,max_dim];
outer_wall2 = [0,0;max_dim,0];
outer_wall3 = [0,max_dim;max_dim,max_dim];
outer_wall4 = [max_dim,0;max_dim,max_dim];
outer_walls = [outer_wall1;outer_wall2;outer_wall3;outer_wall4];

%%% Trinity %%%
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

field_walls = 2*[outer_walls;island_walls;lr_walls;ll_walls;ur_walls];

%%% Two Walls %%%
% wall_box11 = [50,0;50,200];
% wall_box12 = [50,200;100,200];
% wall_box13 = [100,200;100,0];
% wall_box14 = [100,0;50,0];
% wall_box1 = [wall_box11; wall_box12;wall_box13;wall_box14];
% 
% wall_box2 = wall_box1 + [100,48;100,48;100,48;100,48;100,48;100,48;100,48;100,48];
% 
% field_walls = 2*[outer_walls;wall_box1;wall_box2];

%%% Four Squares %%%
% wall_box11 = [50,50;50,150];
% wall_box12 = [50,150;150,150];
% wall_box13 = [150,150;150,50];
% wall_box14 = [150,50;50,50];
% wall_box1 = [wall_box11; wall_box12;wall_box13;wall_box14];
% 
% wall_box21 = [50,50;50,100];
% wall_box22 = [50,100;100,100];
% wall_box23 = [100,100;100,50];
% wall_box24 = [100,50;50,50];
% wall_box2 = [wall_box11; wall_box12;wall_box13;wall_box14];
% 
% wall_box3 = wall_box2 + [75,75;75,75;75,75;75,75;75,75;75,75;75,75;75,75];
% 
% wall_box4 = wall_box1 + [100,0;100,0;100,0;100,0;100,0;100,0;100,0;100,0];
% 
% 
% field_walls = 2*[outer_walls;wall_box1;wall_box3];