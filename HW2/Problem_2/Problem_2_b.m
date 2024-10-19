clc;
clear;

%% parameters
syms x theta x_dot theta_dot x_ddot theta_ddot real
syms M m L g real

% q
q = [x; theta];
q_dot = [x_dot; theta_dot];

%% compute La = K - P
K_cart = 0.5 * M * x_dot^2; 
K_pole = 0.5 * m * (x_dot - theta_dot * L * cos(theta))^2 + 0.5 * m * (-theta_dot * L * sin(theta))^2; 
K = K_cart + K_pole; 

P = m * g * L * cos(theta); 

La = K - P;
La = simplify(La); 

%% compute D, N
% D(q)
D = jacobian(jacobian(La, q_dot), q_dot);
D = simplify(D); 

% C(q,q_dot)*q_dot
f = D * q_dot;
Cq_dot = jacobian(f,q) * q_dot - jacobian(K,q)';

% G(q)
G = jacobian(P, q)';

% N(q,q_dot)
N = Cq_dot + G;
N = simplify(N);

%% answer
disp('D(q):');
disp(D);
disp('N(q, q_dot):');
disp(N);
