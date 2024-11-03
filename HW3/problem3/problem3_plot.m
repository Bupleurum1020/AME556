clc;
clear;
close all;

%% load 2a
problem3_sim;

%% load simscape
load_system("problem3_sim.slx");
out = sim("problem3_sim.slx");
save_system("problem3_sim.slx");
close_system("problem3_sim.slx");

%% plot theta(t) x(t) y(t)
if ~exist('plots', 'dir')
    mkdir('plots');
end

figure;
subplot(3, 1, 1);
plot(out.x);
xlabel('Time (s)');
ylabel('x (m)');
title('x');

subplot(3, 1, 2);
plot(out.theta);
xlabel('Time (s)');
ylabel('Theta (rad)');
title('theta');

subplot(3, 1, 3);
plot(out.theta);
xlabel('Time (s)');
ylabel('u');
title('u');

saveas(gcf, fullfile('plots', '3.png')); 