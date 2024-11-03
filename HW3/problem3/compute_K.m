clc;
clear;
close all;

% 定义参数
M = 1;
m = 0.2;
L = 0.3;
g = 9.81;

% 使用符号工具箱定义符号变量
syms x dx ddx theta dtheta ddtheta u real

% 动力学方程
eq1 = (M + m) * ddx + m * L * sin(theta) * dtheta^2 - m * L * cos(theta) * ddtheta == u;
eq2 = m * L^2 * ddtheta - m * L * cos(theta) * ddx - m * g * L * sin(theta) == 0;
sol = solve([eq1, eq2], [ddx, ddtheta]);

% 定义状态变量 X 和状态方程 f
ddx_solution = simplify(sol.ddx);
ddtheta_solution = simplify(sol.ddtheta);
X = [x; theta; dx; dtheta];
f = [dx; dtheta; ddx_solution; ddtheta_solution];

% 计算 A 和 B 矩阵
A = double(subs(jacobian(f, X), [x, theta, dx, dtheta], [0, 0, 0, 0]));
B = double(subs(jacobian(f, u), [x, theta, dx, dtheta], [0, 0, 0, 0]));

% 定义 LQR 权重矩阵 Q 和 R
Q = diag([10, 10, 1, 1]);  % 状态权重矩阵，可以根据需求调整
R = 1;                     % 控制权重矩阵

% 计算反馈增益矩阵 K
K = lqr(A, B, Q, R);

% 计算闭环矩阵和特征值
Acl = A - B * K;
D = eig(Acl);  % 计算闭环矩阵的特征值

% 输出反馈增益矩阵和闭环特征值的实部
disp('反馈增益矩阵 K 为：');
disp(K);
disp('闭环特征值的实部为：');
disp(real(D));