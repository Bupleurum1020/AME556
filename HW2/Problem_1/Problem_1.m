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
