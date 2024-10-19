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
