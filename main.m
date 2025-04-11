clear; clc; close all;

robot = CreateModel();
robot.Gravity = [0 0 -9.81];
disp(robot);

gui = interactiveRigidBodyTree(robot, MarkerScaleFactor = 1);

%% show workspace
if 1
    figure('Position', [100, 100, 800, 600]);
    h = show(robot);
    title("Workspace");
    hold on;

    % Sample configurations for workspace visualization
    numSamples = 20000;
    configSamples = cell(numSamples, 1);

    for i = 1:numSamples
        sample = randomConfiguration(robot);
        % limit joint 4 range
        if sample(4) > 0.5
            sample(4) = 0.5;
        elseif sample(4) < -0.5
            sample(4) = -0.5;
        end

        configSamples{i} = sample;
    end

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
if 0
    targetPosition = [0.2, 0, 0.6];

    gik = generalizedInverseKinematics;
    gik.RigidBodyTree = robot;
    posConstraint = constraintPositionTarget("end_effector");
    posConstraint.TargetPosition = targetPosition;

    q0 = homeConfiguration(robot);
    gik.ConstraintInputs = {"position"};
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

    disp('joint angle:');
    disp(q);

    %% draw trajectory animation
    if 0
        close all;

        % Validate the solution by forward kinematics
        actualEndEffectorTform = getTransform(robot, q, 'end_effector');
        actualEndEffectorPosition = tform2trvec(actualEndEffectorTform);
        disp('Actual EE Pose:');
        disp(actualEndEffectorPosition);
        disp('Target Pose:');
        disp(targetPosition);
        disp(['Pose Error: ', num2str(norm(actualEndEffectorPosition - targetPosition))]);

        % fig = figure('Position', [100, 100, 900, 700]);
        fig = figure();
        ax = axes('Parent', fig);
        title('Robot Trajectory Animation', 'FontSize', 14);

        light('Position', [1 1 5], 'Style', 'infinite');
        light('Position', [-3 1 5], 'Style', 'infinite');

        view(ax, 3); % Default 3D view
        axis([-0.8 0.8 -0.8 0.8 0 0.8]);
        grid on;

        % camva(30); % Camera view angle
        % camtarget([0.1, 0, 0.3]); % Target center of the robot
        % campos([1.2, 1.2, 0.8]); % Position camera similar to interactive view

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

        frameRate = 60;

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

            progressText = sprintf('Frame: %d/%d', i, steps);
            sgtitle(progressText);

            % drawnow limitrate;
            pause(1 / frameRate);
        end

    end

end

hold off;
