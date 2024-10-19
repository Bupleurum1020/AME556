clc;
clear;
close all;

% parameters
m = 0.5;
I = diag([0.01, 0.01, 0.05]); 
g = [0; 0; -9.81]; 

% I.C.
p0 = [0.5; 0.5; 1]; 
v0 = [0; 0; 0]; 
R0 = eul2rotm([pi/6, pi/8, pi/4]); 
w0 = [0; -0.1; 0.1]; 

q0 = [p0; reshape(R0, [], 1); v0; w0];

% ode
tspan = [0, 2]; 
[t, q] = ode45(@(t, q) uav_dynamics(t, q, m, I, g), tspan, q0);

% extract from q
position = q(:, 1:3); 
R = reshape(q(:, 4:12)', 3, 3, []);
euler = zeros(length(t), 3);
for i = 1:length(t)
    euler(i, :) = rotm2eul(R(:, :, i));
end

%% plot
if ~exist('plots', 'dir')
    mkdir('plots');
end

% pos
figure;
plot(t, position);
xlabel('Time (s)');
ylabel('Position (m)');
legend('x', 'y', 'z');
title('COM Position vs Time');
saveas(gcf, fullfile('plots', 'Position.png')); 

% euler
figure;
plot(t, euler);
xlabel('Time (s)');
ylabel('Euler Angles (rad)');
legend('Roll', 'Pitch', 'Yaw');
title('Euler Angles vs Time');
saveas(gcf, fullfile('plots', 'Euler.png')); 

%% animate
generate_animation(t, q);

%% dynamics
function dq = uav_dynamics(~, s, m, I, g)
    p = s(1:3); 
    R = reshape(s(4:12), 3, 3); 
    v = s(13:15);
    wb = s(16:18);

    dp = v; 
    dR = R * skew_symmetric(wb); 
    dv = g; 

    tau = [0; 0; 0]; 
    dwb = I \ (tau - cross(wb, I * wb)); 

    dq = [dp; reshape(dR, [], 1); dv; dwb];
end

%% Compute S(omega)
function S = skew_symmetric(omega)
    S = [0, -omega(3), omega(2);
         omega(3), 0, -omega(1);
        -omega(2), omega(1), 0];
end

