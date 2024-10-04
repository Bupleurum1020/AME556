clc;
clear;
close all;

%% (a) Triangle
A0 = [0; 0; 0];
B0 = [2; 0; 0];
C0 = [0; 1; 0];
triangle0 = [A0, B0, C0]; 

figure;
view(3);
axis equal; 
grid on;
patch(triangle0(1, :), triangle0(2, :), triangle0(3, :), 'blue'); 
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Original Triangle ABC');

xlim([-2 2]);
ylim([-2 2]);
zlim([-2 2]);

%% (b) rotate pi/6 about x-axis
R1 = eul2rotm([pi/6, 0, 0], 'XYZ'); 
triangle1 = rotateTriangle(triangle0, R1);

%% (c) rotate -pi/4 about y-axis
R2 = eul2rotm([0, -pi/4, 0], 'XYZ'); 
triangle2 = rotateTriangle(triangle1, R2);

%% (d) rotate 2pi/3 about z-axis
R3 = eul2rotm([0, 0, 2*pi/3], 'XYZ'); 
triangle3 = rotateTriangle(triangle2, R3);

%% (e) rotate back
R_total = R3 * R2 * R1;
R_inverse = inv(R_total);

triangle4 = rotateTriangle(triangle3, R_inverse);

%% rotation function
function rotatedTriangle = rotateTriangle(triangle, R)

    rotatedTriangle = R * triangle;

    % plot
    figure;
    view(3); 
    axis equal;
    grid on;
    patch(rotatedTriangle(1, :), rotatedTriangle(2, :), rotatedTriangle(3, :), 'red'); 
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Triangle Rotated using Rotation Matrix');
    
    xlim([-2 2]);
    ylim([-2 2]);
    zlim([-2 2]);
end
