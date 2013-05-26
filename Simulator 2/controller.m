function u = controller(x, x1_ref, A, B)

x1 = x(1:2);
x2 = x(3:4);

A11 = A(1:2,1:2);
A12 = A(1:2,3:4);
A22 = A(3:4,3:4);
B2 = B(3:4,:);

x2_ref = -inv(A12)*A11*x1_ref;
u_ref = -inv(B2)*A22*x2_ref;

Q = [2,0,0,0;
     0,2,0,0;
     0,0,1,0;
     0,0,0,1];
%Q = -(P'*P);

R = 1e-4*[1,0;
          0,1];

 
 M = are(A, (B/(R))*B', Q);
 K = (-R\B')*M;
 %u = K*x;
%u = [0,0]';
 v = K*[x1 - x1_ref;x2 - x2_ref];
 u = v + u_ref;
%  disp('Controller');
%  disp(A);
%  disp(A + B*K);
%  disp(eig(A+B*K));
