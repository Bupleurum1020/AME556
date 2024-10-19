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
