clc;
clear;
close all;

%% load 2.c
Problem_2_c;

%% load simscape
load_system("Problem_2_d.slx");
out = sim("Problem_2_d.slx");
save_system("Problem_2_d.slx");
close_system("Problem_2_d.slx");

%% plot x(t) & theta(t)
if ~exist('plots', 'dir')
    mkdir('plots');
end

figure;
subplot(2, 1, 1);
plot(out.x);
xlabel('Time (s)');
ylabel('Cart Position (x)');
title('Cart Position');

subplot(2, 1, 2);
plot(out.theta);
xlabel('Time (s)');
ylabel('Pendulum Angle (theta)');
title('Pendulum Angle');
saveas(gcf, fullfile('plots', '2.d.png')); 

%% compare theta(t) from 2.c & 2.d

figure;
hold on;

plot(t, theta, 'r-', 'LineWidth', 2); % Red line for 2c
plot(out.theta, 'b--', 'LineWidth', 2); % Blue dashed line for 2d

xlabel('Time (s)');
ylabel('Pendulum Angle (theta)');
title('Comparison of Joint Angle \theta(t) - 2.c vs 2.d');
legend('2.c (Compute)', '2.d (Simscape)');

grid on;
hold off;

saveas(gcf, fullfile('plots', 'compare.png')); 
