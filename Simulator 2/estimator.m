function x_hatdot = estimator(A, B, C, x_hat,y,u)

% Process noise transformation
F = eye(4);
% Process noise covariance
V = eye(4);
% Sensor noise covariance
W = 1e-4*   [   2, 0, 0, 0;
                0, 2, 0, 0;
                0, 0, 1, 0;
                0, 0, 0, 1];
         
P = are(A', (C'/(W))*C, F*V*F');
% Kalman gain
G = P*C'/(W);

% State estimate
x_hatdot = A*x_hat + B*u - G*(C*x_hat - y);
