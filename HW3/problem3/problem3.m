clc;
clear;
close all;

%% Linearization
% 使用符号工具箱定义符号变量
syms x dx ddx theta dtheta ddtheta u real

% 定义参数
M = 1;
m = 0.2;
L = 0.3;
g = 9.81;

% 原始动力学方程
eq1 = (M + m) * ddx + m * L * sin(theta) * dtheta^2 - m * L * cos(theta) * ddtheta == u;
eq2 = m * L^2 * ddtheta - m * L * cos(theta) * ddx - m * g * L * sin(theta) == 0;
sol = solve([eq1, eq2], [ddx, ddtheta]);

% 分离出 ddx 和 ddtheta 的解，并进行简化
ddx_solution = simplify(sol.ddx);
ddtheta_solution = simplify(sol.ddtheta);

% 定义状态变量 X 和状态方程 f
X = [x; theta; dx; dtheta];
f = [dx; dtheta; ddx_solution; ddtheta_solution];

% 计算 A 和 B 矩阵
A = double(subs(jacobian(f, X), [x, theta, dx, dtheta], [0, 0, 0, 0]));
B = double(subs(jacobian(f, u), [x, theta, dx, dtheta], [0, 0, 0, 0]));

%% location of poles
st_d = [0.5 1];
zetaomegan = 5./st_d

overshoot_d = [10, 20];
zeta_range = zeros(1, 2);
syms zeta
for i = 1:2
    eq = exp(-zeta * pi / sqrt(1 - zeta^2)) * 100 == overshoot_d(i);
    zeta_solution = double(solve(eq, zeta));
    zeta_range(i) = zeta_solution(imag(zeta_solution) == 0 & zeta_solution >= 0 & zeta_solution <= 1);
end

% 显示结果
fprintf('对于超调量范围 [%g%%, %g%%]，阻尼比 ζ 的范围约为 [%.4f, %.4f]\n', overshoot_d(1), overshoot_d(2), zeta_range(1), zeta_range(2));

%% compute K
% 设置期望的极点位置（根据系统要求调整）
desired_poles = [-6+3j, -6-3j, -60, -55];

% 计算增益矩阵 K
K = place(A, B, desired_poles)
% 定义闭环系统矩阵
Acl = A - B * K
sys_cl = ss(Acl, [], eye(4), []);
D = eig(Acl)

% 初始条件
X0 = [0.1; 0.1; 0; 0];  % 初始状态，可以根据实际情况调整

% 仿真
t = 0:0.01:2;  % 仿真时间
[X, T] = initial(sys_cl, X0, t);

% 提取 x(t) 和 theta(t)
x_t = X(:, 1);
theta_t = X(:, 2);

% 计算控制输入 u(t) = -K * X(t)
u_t = -K * X';

% 绘制 x(t)、theta(t) 和 u(t)
figure;

subplot(3, 1, 1);
plot(T, x_t);
xlabel('Time (s)');
ylabel('x(t)');
title('Position x(t)');

subplot(3, 1, 2);
plot(T, theta_t);
xlabel('Time (s)');
ylabel('\theta(t)');
title('Angle \theta(t)');

subplot(3, 1, 3);
plot(T, u_t);
xlabel('Time (s)');
ylabel('u(t)');
title('Control Input u(t)');