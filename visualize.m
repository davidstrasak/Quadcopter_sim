function [] = visualize(input)
% Function that visualizes the outputs of the quadcopter calculations
arguments
    input (1,1) struct
end

% Extract all inputs from the structure
timeStep = input.timeStep;
deltaT = input.deltaT;
speedUp = input.speedUp;
currentState = input.currentState;
wayPoints = input.wayPoints;
targetPoint = input.targetPoint;
positionTolerance = input.positionTolerance;
forwardPos = input.forwardPos;
lateralPos = input.lateralPos;
altitudeCurrent = input.altitudeCurrent;

%% Visualization function
% Update plots at reduced rate for performance
if mod(timeStep,deltaT*speedUp) == 0
    % Set default font properties
    set(0, 'DefaultAxesFontName', 'Arial')
    set(0, 'DefaultAxesFontSize', 11)
    set(0, 'DefaultTextFontName', 'Arial')
    
    % Define visualization properties
    withinToleranceProps.MarkerSize = 8;
    withinToleranceProps.Color = [0 1 0];  % Darker green
    withinToleranceProps.Marker = 'x';        % Circle marker
    
    outsideToleranceProps.MarkerSize = 8;
    outsideToleranceProps.Color = [0 0 0];  % Light gray
    outsideToleranceProps.Marker = '.';           % Dot marker

    % 3D trajectory visualization
    subplot(2,2,1)
    hold off
    plot3(currentState.BodyXYZPosition.X,...
          currentState.BodyXYZPosition.Y,...
          currentState.BodyXYZPosition.Z,'x','Color',[0 0.7 0],'LineWidth',1.5)
    hold on
    
    % Create ground plane for reference
    [meshX,meshY] = meshgrid(-0.5:0.1:4,-0.5:0.1:1.5);
    meshZ = zeros(size(meshX));
    plot3(meshX,meshY,meshZ,'.','Color',[0.7 0.7 0.7],'MarkerSize',1)
    
    % Show waypoint path
    plot3(wayPoints(:,1),wayPoints(:,2),wayPoints(:,3),'r-o','LineWidth',1.5)
    
    title('Quadcopter 3D Trajectory','FontWeight','bold')
    xlabel('X Position (m)','FontWeight','bold')
    ylabel('Y Position (m)','FontWeight','bold')
    zlabel('Z Position (m)','FontWeight','bold')
    grid on
    box on
    
    disp(['Current time: ' num2str(timeStep, '%.2f') ' s'])
    clear meshX meshY meshZ

    % X Position
    subplot(2,2,2)
    if abs(forwardPos - targetPoint(1)) < positionTolerance
        plot(timeStep, forwardPos, withinToleranceProps.Marker, ...
            'Color', withinToleranceProps.Color, ...
            'MarkerSize', withinToleranceProps.MarkerSize, ...
            'LineWidth', 1.5)
    else
        plot(timeStep, forwardPos, outsideToleranceProps.Marker, ...
            'Color', outsideToleranceProps.Color, ...
            'MarkerSize', outsideToleranceProps.MarkerSize)
    end
    hold on
    title('X Position Tracking','FontWeight','bold')
    xlabel('Time (s)','FontWeight','bold')
    ylabel('X Position (m)','FontWeight','bold')
    grid on
    
    % Y Position
    subplot(2,2,3)
    if abs(lateralPos - targetPoint(2)) < positionTolerance
        plot(timeStep, lateralPos, withinToleranceProps.Marker, ...
            'Color', withinToleranceProps.Color, ...
            'MarkerSize', withinToleranceProps.MarkerSize, ...
            'LineWidth', 1.5)
    else
        plot(timeStep, lateralPos, outsideToleranceProps.Marker, ...
            'Color', outsideToleranceProps.Color, ...
            'MarkerSize', outsideToleranceProps.MarkerSize)
    end
    hold on
    title('Y Position Tracking','FontWeight','bold')
    xlabel('Time (s)','FontWeight','bold')
    ylabel('Y Position (m)','FontWeight','bold')
    grid on
    
    % Z Position
    subplot(2,2,4)
    if abs(altitudeCurrent - targetPoint(3)) < positionTolerance
        plot(timeStep, altitudeCurrent, withinToleranceProps.Marker, ...
            'Color', withinToleranceProps.Color, ...
            'MarkerSize', withinToleranceProps.MarkerSize, ...
            'LineWidth', 1.5)
    else
        plot(timeStep, altitudeCurrent, outsideToleranceProps.Marker, ...
            'Color', outsideToleranceProps.Color, ...
            'MarkerSize', outsideToleranceProps.MarkerSize)
    end
    hold on
    title('Z Position Tracking','FontWeight','bold')
    xlabel('Time (s)','FontWeight','bold')
    ylabel('Z Position (m)','FontWeight','bold')
    grid on
    
    % Adjust figure properties
    set(gcf, 'Color', 'white')
    
    pause(0.00001)
end
end

