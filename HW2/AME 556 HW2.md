## AME 556 HW2

Name: Shuai Zhao	Uscid: 7722927131

# Problem 1

## a. Derive the dynamics of the system.

<img src="/Users/zhaoshuai/Library/Application Support/typora-user-images/image-20241015182311055.png" alt="image-20241015182311055" style="zoom:50%;" />

<div style="page-break-after: always;"></div>

## b. & c. 

* Plots

  <img src="/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_1/plots/trajectory_1.b.png" alt="trajectory_1.b" style="zoom: 25%;" />

  <img src="/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_1/plots/trajectory_1.c.i.png" alt="trajectory_1.c.i" style="zoom: 25%;" />

  <img src="/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_1/plots/trajectory_1.c.ii.png" alt="trajectory_1.c.ii" style="zoom: 25%;" />

* Videos

  https://drive.google.com/drive/folders/1am3pdWizw8bbAHBCgThMnKUuKT9bJgz0?usp=share_link

  

## Code

```matlab
clc;
clear;
close all;

%% parameters
m = 0.5;  
g = 9.81;  
c = [0.05, 0.05, 0.5];  
tspan = [0, 2];

% initial_conditions
X0s = [
    0, 0, 1, 2;   % 1.b
    0, 0, 2, 1;   % 1.c.1
    0, 0, 1, 2    % 1.c.2
];

% labels on the plots and videos
labels = {'1.b', '1.c.i', '1.c.ii'};  

%% main
for i = 1:length(c)
    % parameters
    current_c = c(i);
    X0 = X0s(i, :);

    % compute ode
    [t, X] = ode45(@(t, X) dynamics(t, X, m, g, current_c), tspan, X0);

    % Extract states from X
    x = X(:, 1);
    y = X(:, 2);
    vx = X(:, 3);
    vy = X(:, 4);

    plot_trajectory(x, y, labels{i});
    generate_video(t, x, y, labels{i});
end

function X_dot = dynamics(~, X, m, g, c)
    % |v|
    vx = X(3);
    vy = X(4);
    v = sqrt(vx^2 + vy^2);

    % a
    ax = -c / m * v * vx;
    ay = -g - c / m * v * vy;

    % q_dot
    X_dot = [vx; vy; ax; ay];
end

%% plot function 
function plot_trajectory(x, y, label)
    if ~exist('plots', 'dir')
        mkdir('plots');
    end

    % plot
    figure;
    plot(x, y);
    title(['Ball Trajectory - ' label]);
    xlabel('x (m)');
    ylabel('y (m)');
    axis equal;
    grid on;

    % save plots
    saveas(gcf, fullfile('plots', ['trajectory_' label '.png']));
end

%% video function 
function generate_video(t, x, y, label)
    if ~exist('videos', 'dir')
        mkdir('videos');
    end

    % video
    video = VideoWriter(fullfile('videos', ['animation_' label '.mp4']), 'MPEG-4');
    video.FrameRate = 30;  
    open(video);

    % plot
    figure;
    hold on;
    plot(x, y, 'r--');  
    ball = plot(x(1), y(1), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');  
    axis equal;  
    title(['Ball Motion Animation - ' label]);
    xlabel('x (m)');
    ylabel('y (m)');
    grid on;

    time_text = text(0.1, 0.9, '', 'Units', 'normalized', 'FontSize', 12, 'Color', 'black');

    for i = 1:length(t)
        set(ball, 'XData', x(i), 'YData', y(i));

        % timer
        set(time_text, 'String', sprintf('Time: %.2f s', t(i)));

        frame = getframe(gcf);
        writeVideo(video, frame);
    end

    hold off;
    close(video);  
end

```

<div style="page-break-after: always;"></div>

# Problem 2

## a. Derive the dynamic of the robot.

![image-20241015184254622](/Users/zhaoshuai/Library/Application Support/typora-user-images/image-20241015184254622.png)

<div style="page-break-after: always;"></div>

## b. derive symbolic functions of 𝐷(𝑞),𝑁(𝑞,𝑞̇) :

### (1) Result

<img src="/Users/zhaoshuai/Library/Application Support/typora-user-images/image-20241015194101425.png" alt="image-20241015194101425" style="zoom: 50%;" />

### (2) Code

```matlab
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

```

## c. simulate the system in 2 seconds

### (1) Plot

![2.c](/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_2/plots/2.c.png)



### (2) Video

https://drive.google.com/drive/folders/1am3pdWizw8bbAHBCgThMnKUuKT9bJgz0?usp=share_link

<div style="page-break-after: always;"></div>

### (3) Code

```matlab
clc;
clear;
close all;

% parameters
M = 1;         
m = 0.2;       
L = 0.3;       
g = 9.81;      

% I.C.
x0 = 0;                     
x_dot0 = 0;                 
theta0 = pi/6;              
theta_dot0 = 0;             

q0 = [x0; x_dot0; theta0; theta_dot0];

%% main
% ODE
tspan = [0 2];  
[t, q] = ode45(@(t, q) dynamics(t, q, M, m, L, g), tspan, q0);

% extract x & theta
x = q(:, 1);
theta = q(:, 3);

% plot and video
plot_x_theta(t, x, theta);
generate_video(t, x, theta, L);

%% dynamics
function q_dot = dynamics(~, q, M, m, L, g)
    x = q(1);
    x_dot = q(2);
    theta = q(3);
    theta_dot = q(4);

    % D & N
    D = [M + m, -L * m * cos(theta);
         -L * m * cos(theta), L^2 * m];
    N = [L * m * theta_dot^2 * sin(theta);
         -L * g * m * sin(theta)];

    a = D \ (-N);   % tau = 0

    % q_dot
    q_dot = [x_dot; a(1); theta_dot; a(2)];
end

%% plot x(t) & theta(t)
function plot_x_theta(t, x, theta)
    if ~exist('plots', 'dir')
        mkdir('plots');
    end

    figure;
    subplot(2, 1, 1);
    plot(t, x);
    xlabel('Time (s)');
    ylabel('Cart Position (x)');
    title('Cart Position');

    subplot(2, 1, 2);
    plot(t, theta);
    xlabel('Time (s)');
    ylabel('Pendulum Angle (theta)');
    title('Pendulum Angle');
    saveas(gcf, fullfile('plots', '2.c.png'));
end

%% video
function generate_video(t, x, theta, L)
    if ~exist('videos', 'dir')
        mkdir('videos');
    end
    
    figure;
    cart_width = 0.2; 
    cart_height = 0.1; 
    pendulum_radius = 0.03; 

    axis equal;
    set(gca, 'XLim', [-0.8, 0.5]);
    set(gca, 'YLim', [-0.4, 0.5]);
    xlabel('X Position');
    ylabel('Y Position');
    title('Cart-Pole System Animation');
    
    video = VideoWriter(fullfile('videos', 'animation_2_c.mp4'), 'MPEG-4');
    open(video);

    for i = 1:length(t)
        cla;

        hold on;
        % Draw cart
        rectangle('Position', [x(i) - cart_width / 2, -cart_height, cart_width, cart_height], ...
                  'FaceColor', [0 0 1], 'EdgeColor', [0 0 1]); 

        % Compute pendulum position
        pendulum_x = x(i) - L * sin(theta(i));
        pendulum_y = L * cos(theta(i));

        % Draw pendulum
        rectangle('Position', [pendulum_x - pendulum_radius, pendulum_y - pendulum_radius, ...
                  2 * pendulum_radius, 2 * pendulum_radius], ...
                  'Curvature', [1, 1], 'FaceColor', [1 0 0], 'EdgeColor', [1 0 0]); 

        % Draw rod
        plot([x(i), pendulum_x], [0, pendulum_y], 'r-', 'LineWidth', 2); 

        % Display information
        theta_deg = rad2deg(theta(i));
        text(-0.6, 0.4, sprintf('Time: %.2f s', t(i)), 'FontSize', 10, 'Color', 'k');
        text(-0.6, 0.35, sprintf('x: %.2f m', x(i)), 'FontSize', 10, 'Color', 'k');
        text(-0.6, 0.3, sprintf('\\theta: %.2f rad (%.2f°)', theta(i), theta_deg), 'FontSize', 10, 'Color', 'k');

        drawnow;

        % Capture and write frame
        frame = getframe(gcf);
        writeVideo(video, frame);
    end

    close(video);
end

```

<div style="page-break-after: always;"></div>

## d. Use MATLAB Simscape to solve problem 2.c.

### (1) Plot for 2.d.

<img src="/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_2/plots/2.d.png" alt="2.d" style="zoom: 40%;" />

### (2) Compare with 2.c.

<img src="/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_2/plots/compare.png" alt="compare" style="zoom: 40%;" />

### (3) Video

https://drive.google.com/drive/folders/1am3pdWizw8bbAHBCgThMnKUuKT9bJgz0?usp=share_link

<div style="page-break-after: always;"></div>

# Problem 3

## (1) Plot

* COM's positions

  <img src="/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_3/plots/Position.png" alt="Position" style="zoom:40%;" />

* Body orientation's Euler angles

  <img src="/Users/zhaoshuai/Desktop/AME 556/HW2/Problem_3/plots/Euler.png" alt="Euler" style="zoom:40%;" />

## (2) Video

https://drive.google.com/drive/folders/1am3pdWizw8bbAHBCgThMnKUuKT9bJgz0?usp=share_link

## (3) Code

```matlab
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

function generate_animation(t, state)
    position = state(:, 1:3); 
    R_matrices = reshape(state(:, 4:12)', 3, 3, []);

    % video open 
    v = VideoWriter('uav_simulation.mp4', 'MPEG-4'); 
    v.FrameRate = 30; 
    open(v);
    figure;

    % arms
    d = 0.2; 
    r = 0.05;
    arm_body = [
        d, 0, 0;    
        -d, 0, 0; 
        0, d, 0;   
        0, -d, 0  
    ]';
    colors = {'r', [0.6, 0.8, 1], 'g', 'b'}; 

    for i = 1:length(t)
        pos = position(i, :)';
        R = R_matrices(:, :, i);
        arm_world = R * arm_body + pos;

        % subplot for different view
        for subplot_index = 1:4
            subplot(2, 2, subplot_index);
            hold on;
            grid on;

            % arms
            for j = 1:4
                plot3([pos(1), arm_world(1, j)], ...
                      [pos(2), arm_world(2, j)], ...
                      [pos(3), arm_world(3, j)], ...
                      'Color', colors{j}, 'LineWidth', 2);
            end

            % circles
            for j = 1:4
                draw_circle(arm_world(:, j), r, R);
            end

            % set view
            switch subplot_index
                case 1
                    view(0, 0); % Front
                    title('Front View');
                case 2
                    view(-90, 0); % Left
                    title('Left View');
                case 3
                    view(0, 90); % Top
                    title('Top View');
                case 4
                    view(3); % 3D
                    title('3D View');
            end

            % change the xylim to keep uav at the center of the plot
            % or the uav would fall out from z-
            xlim([pos(1) - 0.5, pos(1) + 0.5]);
            ylim([pos(2) - 0.5, pos(2) + 0.5]);
            zlim([pos(3) - 0.5, pos(3) + 0.5]); 
            xlabel('X'); ylabel('Y'); zlabel('Z');
            hold off;
        end

        drawnow;
        frame = getframe(gcf);
        writeVideo(v, frame);

        if i < length(t)
            clf;
        end
    end

    close(v);
end

function draw_circle(center, radius, R)
    theta = linspace(0, 2*pi, 50);
    circle_body = radius * [cos(theta); sin(theta); zeros(1, length(theta))];
    circle_world = R * circle_body + center;
    plot3(circle_world(1, :), circle_world(2, :), circle_world(3, :), 'k', 'LineWidth', 1);
end

```

