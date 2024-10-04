clc;
clear;
close all;

%% Euler Angle & Rotation Matrix
roll = pi/3;
pitch = -pi/4;
yaw = pi/2;

R = eul2rotm([yaw, pitch, roll], 'ZYX');
disp('Rotation Matrix R:');
disp(R);

%% p & p1
p = [1; 2; 3];
p1 = R * p;
disp('Coordinates of p1:');
disp(p1);

%% Plot
figure;
hold on;

plot3([0 p(1)], [0 p(2)], [0 p(3)], 'b', 'LineWidth', 2);
text(p(1), p(2), p(3), ' p', 'FontSize', 12, 'Color', 'b');

plot3([0 p1(1)], [0 p1(2)], [0 p1(3)], 'r', 'LineWidth', 2);
text(p1(1), p1(2), p1(3), ' p1', 'FontSize', 12, 'Color', 'r');

axis equal; 
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Original Vector p and Rotated Vector p1');
legend('Original Vector p', 'Rotated Vector p1');
view(3); 
hold off;
