clear; clc; close all;

robot = CreateModel();
robot.Gravity = [0 0 -9.81];
disp(robot);

% gui = interactiveRigidBodyTree(robot, MarkerScaleFactor = 1);

%% show workspace
if 0
    figure('Position', [100, 100, 800, 600]);
    h = show(robot);
    title("Workspace");
    hold on;

    % Sample configurations for workspace visualization
    numSamples = 20000;
    configSamples = cell(numSamples, 1);

    for i = 1:numSamples
        configSamples{i} = randomConfiguration(robot);
    end

    % Calculate end-effector positions for each configuration
    eePositions = zeros(numSamples, 3);

    for i = 1:numSamples
        T = getTransform(robot, configSamples{i}, 'end_effector');
        eePositions(i, :) = tform2trvec(T);
    end

    scatter3(eePositions(:, 1), eePositions(:, 2), eePositions(:, 3), 10, 'blue', 'filled', 'MarkerFaceAlpha', 0.3);
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    grid on;
    view(45, 30);

    axis([-0.3 0.3 -0.3 0.3 0 0.7]);
end

%% Inverse Kinematics
if 1
    targetPosition = [0.2, 0, 0.6];

    gik = generalizedInverseKinematics;
    gik.RigidBodyTree = robot;
    posConstraint = constraintPositionTarget("end_effector");
    posConstraint.TargetPosition = targetPosition;

    % Create aiming constraint to make the end effector point toward the robot base
    % aimConstraint = constraintAiming("end_effector");
    % aimConstraint.TargetPoint = [0, 0, 0]; % Robot base origin

    q0 = homeConfiguration(robot);

    % gik.ConstraintInputs = {"position", "aiming"};
    gik.ConstraintInputs = {"position"};
    % [q, solutionInfo] = gik(q0, posConstraint, aimConstraint);
    [q, solutionInfo] = gik(q0, posConstraint);

    % Visualize the solution in a new figure
    figure('Position', [100, 100, 800, 600]);
    show(robot, q);
    title(['IK Solution']);
    axis([-0.5 0.5 -0.5 0.5 0 1]);
    hold on;
    plot3(targetPosition(1), targetPosition(2), targetPosition(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

    % Plot a line from the end effector to the base to visualize the aiming constraint
    endEffectorTform = getTransform(robot, q, 'end_effector');
    endEffectorPosition = tform2trvec(endEffectorTform);
    line([endEffectorPosition(1), 0], [endEffectorPosition(2), 0], [endEffectorPosition(3), 0], 'Color', 'b', 'LineWidth', 2);

    hold off;

    % Display the solution configuration
    disp('关节配置 (弧度):');
    disp(q);

    %% draw trajectory animation
    if 1
        close all;

        % Validate the solution by forward kinematics
        actualEndEffectorTform = getTransform(robot, q, 'end_effector');
        actualEndEffectorPosition = tform2trvec(actualEndEffectorTform);
        disp('Actual EE Pose:');
        disp(actualEndEffectorPosition);
        disp('Target Pose:');
        disp(targetPosition);
        disp(['Pose Error: ', num2str(norm(actualEndEffectorPosition - targetPosition))]);

        fig = figure('Position', [100, 100, 900, 700]);
        ax = axes('Parent', fig);
        title('Robot Trajectory Animation', 'FontSize', 14);

        % Enhance lighting to match interactiveRigidBodyTree
        light('Position', [1 1 5], 'Style', 'infinite');
        light('Position', [-3 1 5], 'Style', 'infinite');
        light('Position', [0 -2 1], 'Style', 'infinite'); % Additional light from another angle

        view(ax, 3); % Default 3D view
        
        % Expand view field significantly - remove tight constraints
        axis([-1 1 -1 1 -0.5 1.5]); % Much wider viewing volume
        
        axis equal;
        grid on;
        
        % Adjust camera settings for better visibility
        camva(40); % Wider camera view angle (was 30)
        camtarget([0, 0, 0.5]); % Target center of the workspace
        campos([1.5, 1.5, 1.0]); % Position camera farther back
        
        % Enable camera toolbar for interactive adjustment
        cameratoolbar('Show');
        
        hold on;

        % Plot the target position in the animation
        plot3(targetPosition(1), targetPosition(2), targetPosition(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

        steps = 50;
        trajectory = zeros(length(q), steps);

        for i = 1:length(q)
            trajectory(i, :) = linspace(q0(i), q(i), steps);
        end

        % Precompute end effector positions for the entire trajectory
        eeTrajectory = zeros(3, steps);

        for i = 1:steps
            tform = getTransform(robot, trajectory(:, i), 'end_effector');
            eeTrajectory(:, i) = tform2trvec(tform)';
        end

        % Plot the complete end effector trajectory in advance
        traj_line = plot3(eeTrajectory(1, :), eeTrajectory(2, :), eeTrajectory(3, :), 'g-', 'LineWidth', 2);

        % Create initial robot visualization with better properties
        h = show(robot, trajectory(:, 1), 'PreservePlot', false, 'Frames', 'off');

        % Add a frameRate control for smoother animation
        frameRate = 30;

        % Add a colorbar for visual enhancement
        colormap(ax, jet);

        % Animate robot
        eeMarker = [];
        baseLine = [];

        for i = 1:steps
            config = trajectory(:, i);

            % Update robot visualization
            h = show(robot, config, 'PreservePlot', false, 'Parent', ax, 'Frames', 'off');

            % Delete previous markers if they exist
            if ~isempty(eeMarker) && isvalid(eeMarker)
                delete(eeMarker);
            end

            if ~isempty(baseLine) && isvalid(baseLine)
                delete(baseLine);
            end

            % Add current end effector position
            eeMarker = plot3(eeTrajectory(1, i), eeTrajectory(2, i), eeTrajectory(3, i), 'g.', 'MarkerSize', 20);

            % Add a line from end effector to base for the current frame
            baseLine = line([eeTrajectory(1, i), 0], [eeTrajectory(2, i), 0], [eeTrajectory(3, i), 0], ...
                'Color', [0.2 0.6 1.0], 'LineWidth', 1.5, 'LineStyle', '-.');

            % Add progress indicator
            progressText = sprintf('Frame: %d/%d', i, steps);
            sgtitle(progressText);

            drawnow limitrate;
            pause(1 / frameRate);
        end

        % Add legend at the end
        legend([h(1), traj_line, eeMarker, baseLine], {'Robot', 'Trajectory', 'End Effector', 'Base Link'}, 'Location', 'northeast');
    end

end

hold off;
