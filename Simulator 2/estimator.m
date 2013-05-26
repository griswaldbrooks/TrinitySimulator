function x_hatdot = estimator(A, B, C, x_hat,y,u)

% Process noise transformation
F = eye(4);
% Process noise
%v = [0.01, 0.01, 0.02,0.02]';
V = eye(4);
% Sensor noise covariance
W = 1e-4*   [   2, 0, 0, 0;
                0, 2, 0, 0;
                0, 0, 1, 0;
                0, 0, 0, 1];
         
P = are(A', (C'/(W))*C, F*V*F');
G = P*C'/(W);
% disp('Observer');
% disp(V)
% disp(W)
% disp(G)
% disp(eig(G));
% disp(eig(A - G*C));
% input('Pause');
x_hatdot = A*x_hat + B*u - G*(C*x_hat - y);
