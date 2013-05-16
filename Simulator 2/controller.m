function u = controller(x, A, B)

P = [2,1,1,1;
     0,2,1,1;
     0,0,2,1;
     0,0,0,2];
Q = -(P'*P);

R = 1e-6*[2,1;
          1,2];

 
 M = are(A, (B/(R))*B', Q);
 K = (-R\B')*M;
 u = K*x;
%u = [0,0]';

%  disp('Controller');
%  disp(A);
%  disp(A + B*K);
%  disp(eig(A+B*K));
