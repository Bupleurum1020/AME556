clc;
clear;
close all;

%% load 2.c
problem1;

%% load simscape
load_system("problem1.slx");
out = sim("problem1.slx");
save_system("problem1.slx");
close_system("problem1.slx");

%% plot theta(t) x(t) y(t)
if ~exist('plots', 'dir')
    mkdir('plots');
end

figure;
subplot(3, 1, 1);
plot(out.x);
xlabel('Time (s)');
ylabel('x (m)');
title('COM position x');

subplot(3, 1, 2);
plot(out.y);
xlabel('Time (s)');
ylabel('y (m)');
title('COM position y');

subplot(3, 1, 3);
plot(out.theta);
xlabel('Time (s)');
ylabel('theta (rad)');
title('Theta');

saveas(gcf, fullfile('plots', '1b.png')); 