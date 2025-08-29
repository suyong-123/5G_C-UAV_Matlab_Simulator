
%=======================================================================
% File: generateUniformWaypoints.m
%
% Description:
%   This script generates a predefined UAV flight trajectory and plots it 
%   along with the positions and coverage of 5G base stations (gNBs).
%   The trajectory waypoints are saved to a file named 'fixedpath.mat'.
%
%=======================================================================
function generateUniformWaypoints()
    % Base station positions [X, Y, Z]
    gNBPositions = [0 0 25; 700 0 25; 550 300 25; 350 0 25; 180 300 25];  
    
    % Generate trajectory waypoints (example route)
    waypoints = [
        400 200 150;
        200 0 150;        
        300 -100 150;       
       
        800 50 150;       
        520 100 150;       
        780 250 150;       
        500 500 150;      
        350 350 150;       
        200 200 150;       
        -100 100 150;     
        -100 -100 150;
        178 -102 150;
        524 -97 150;
    ];
    
    



    % Save waypoints
    save('fixedpath.mat', 'waypoints');

 
    % === Plotting ===
    figure;
    hold on;
    
    % Plot trajectory
    trajectoryPlot = plot(waypoints(:,1), waypoints(:,2), 'b-.', ...
        'LineWidth', 0.7, 'MarkerSize', 8, 'MarkerFaceColor', 'b', ...
        'DisplayName', 'Trajectory');

    selected_segments = [1, 3, 5, 7, 9];  % Select specific segments to add direction symbols
    for k = selected_segments
        if k < size(waypoints,1)
            % Compute the offset position (slightly forward on the segment)
            offset = 0.5; % Ratio from start to end point (0.5 = midpoint)
            arrow_pos = (1-offset)*waypoints(k,1:2) + offset*waypoints(k+1,1:2);
    
            % Calculate the precise rotation angle
            dx = waypoints(k+1,1) - waypoints(k,1);
            dy = waypoints(k+1,2) - waypoints(k,2);
            angle = atan2d(dy, dx); % Compute angle directly (in degrees)
    
            % Draw the direction symbol (rotated based on the angle)
            text(arrow_pos(1), arrow_pos(2), '\bf>', ...  % Use \bf for bold arrow
                'FontSize', 16, ...  % Increase font size for visibility
                'Color', 'b', ...
                'HorizontalAlignment', 'center', ...
                'VerticalAlignment', 'middle', ...
                'Rotation', angle, ...
                'Interpreter', 'tex'); % Use TeX interpreter to support bold formatting
        end
    end

        

    % Plot base stations
    bsPlot = plot(gNBPositions(:,1), gNBPositions(:,2), '^', ...
        'MarkerSize', 10, ...
        'MarkerEdgeColor', [0.5 0.5 0.5], ...
        'MarkerFaceColor', [0.7 0.7 0.7], ...
        'DisplayName', 'gNB');

    % Plot coverage circles (for each base station)
    theta = linspace(0, 2*pi, 100);
    radius = 200;
    coverageColor = [0.8 0.8 0.8];

    % Dummy plot for legend entry
    plot(NaN, NaN, '--', 'Color', coverageColor, ...
         'LineWidth', 1);

    for i = 1:size(gNBPositions,1)
        x_circle = gNBPositions(i,1) + radius * cos(theta);
        y_circle = gNBPositions(i,2) + radius * sin(theta);
        plot(x_circle, y_circle, '--', 'Color', coverageColor, 'LineWidth', 1);
    end

    % Final plot settings
    % grid on;
    % xlabel('X (m)');
    % ylabel('Y (m)');
    %title('5G Simulated Network Topology');
    legend([bsPlot,trajectoryPlot], 'Location', 'best','FontSize',8);
  
    xLimits = get(gca, 'XLim');
    yLimits = get(gca, 'YLim');
    
    
    xlabel('X (m)', 'Units', 'data', 'Position', [xLimits(2), yLimits(1) - 10, 0], 'HorizontalAlignment', 'right');
    ylabel('Y (m)', 'Units', 'data', 'Position', [xLimits(1) - 10, yLimits(2), 0], 'HorizontalAlignment', 'right', 'Rotation', 0);


    set(gca, 'FontSize', 12,'FontName', 'Times New Roman');

    %set(gcf, 'Units', 'inches', 'Position', [1, 1, 3.5, 2.5]);  % IEEE single-column size
    axis equal;
    hold off;
end
