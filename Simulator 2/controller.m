function u = controller(x, x1_ref, A, B)
% x1_ref are the desired wheel angular velocities

% Decomposed states
x1 = x(1:2);    % Wheel angular velocities
x2 = x(3:4);    % Motor torques

% Decomposed system matricies
A11 = A(1:2,1:2);
A12 = A(1:2,3:4);
A22 = A(3:4,3:4);
B2 = B(3:4,:);

% Required motor reference torques to achieve angular velocity reference
x2_ref = -inv(A12)*A11*x1_ref;
% Control input resultant reference voltage
u_ref = -inv(B2)*A22*x2_ref;

% State cost
Q = [50,0,0,0;
     0,50,0,0;
     0,0,1,0;
     0,0,0,1];

% Input cost
R = 1e-4*[1,0;
          0,1];

 
M = are(A, (B/(R))*B', Q);
% Control gain 
K = (-R\B')*M;

% Error gain input
v = K*[x1 - x1_ref;x2 - x2_ref];
% Controller output
u = v + u_ref;
