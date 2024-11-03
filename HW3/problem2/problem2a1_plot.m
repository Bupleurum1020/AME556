clc;
clear;
close all;

%% load 2a
problem2a1;

%% load simscape
load_system("problem2a1.slx");
out = sim("problem2a1.slx");
save_system("problem2a1.slx");
close_system("problem2a1.slx");

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

saveas(gcf, fullfile('plots', '2a_1.png')); 

%% plot q(t)
figure;
subplot(2, 2, 1);
plot(out.q1);
xlabel('Time (s)');
ylabel('q_1 (rad)');
title('Joint Angle of Leg_1');

subplot(2, 2, 2);
plot(out.q2);
xlabel('Time (s)');
ylabel('q_2 (rad)');
title('Joint Angle of Leg_2');

subplot(2, 2, 3);
plot(out.q3);
xlabel('Time (s)');
ylabel('q_3 (rad)');
title('Joint Angle of Leg_3');

subplot(2, 2, 4);
plot(out.q4);
xlabel('Time (s)');
ylabel('q_4 (rad)');
title('Joint Angle of Leg_4');

saveas(gcf, fullfile('plots', '2a_2.png')); 