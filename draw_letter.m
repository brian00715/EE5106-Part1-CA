%% Create a more robust version of the main script
clear; clc; close all;

robot = CreateModel();
robot.Gravity = [0 0 -9.81];
disp(robot);

% gui = interactiveRigidBodyTree(robot, MarkerScaleFactor = 1);
% visualizeWorkspace(robot, 1000); % Reduced sample count for speed

writingPlane = 0.1; % X position for writing (fixed X value for YZ plane)
letterSize = 0.025; % Size of letters
letterSpacing = 0.05; % Spacing between letters (now in Y direction)
startPos = [writingPlane, -0.1, 0.5]; % Starting position for writing in YZ plane

textToWrite = 'HI';

% Create a letter-by-letter writing function (safer approach)
writeSingleLetters(robot, textToWrite, startPos, letterSize, letterSpacing, writingPlane);

%% Create a simpler function for writing letters one by one (more reliable)
function writeSingleLetters(robot, text, startPos, letterSize, letterSpacing, writingPlane)
    figure('Position', [100, 100, 900, 700]);
    show(robot);
    hold on;

    % Set view and lighting
    view(45, 30); % Keep this viewing angle for visualization
    light('Position', [1 1 5], 'Style', 'infinite');
    light('Position', [-3 1 5], 'Style', 'infinite');

    % Set axis limits based on workspace
    axis([-0.3 0.3 -0.3 0.3 0 0.7]);
    grid on;
    title('Robot Writing Letters on YZ Plane', 'FontSize', 14);

    % Display which text we're writing
    disp(['Writing text: "', text, '"']);

    for i = 1:length(text)
        currentLetter = text(i);
        disp(['Processing letter: ', currentLetter]);
        currentPos = startPos;
        currentPos(2) = startPos(2) + (i - 1) * letterSpacing; % Move in Y direction for letter spacing

        % Generate letter paths based on the letter
        switch upper(currentLetter)
            case 'H'
                % Define H shape in YZ plane (x position is fixed)
                pts = [
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2), currentPos(3) + letterSize / 2; % Middle left
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize / 2; % Middle right
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize; % Top right
                       writingPlane, currentPos(2) + letterSize, currentPos(3) % Bottom right
                       ];
                plot3(pts(:, 1), pts(:, 2), pts(:, 3), 'r-', 'LineWidth', 2);
                disp('Drawing letter H');

            case 'I'
                % Define I shape in YZ plane (x position is fixed)
                pts = [
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2) + letterSize, currentPos(3); % Bottom right
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3); % Bottom middle
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top middle
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize % Top right
                       ];
                plot3(pts(:, 1), pts(:, 2), pts(:, 3), 'g-', 'LineWidth', 2);
                disp('Drawing letter I');

            otherwise
                % Simple square for undefined letters (in YZ plane)
                pts = [
                       writingPlane, currentPos(2), currentPos(3);
                       writingPlane, currentPos(2) + letterSize, currentPos(3);
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize;
                       writingPlane, currentPos(2), currentPos(3) + letterSize;
                       writingPlane, currentPos(2), currentPos(3)
                       ];
                plot3(pts(:, 1), pts(:, 2), pts(:, 3), 'b-', 'LineWidth', 2);
                disp(['Drawing default shape for letter: ', currentLetter]);
        end

        % Now we have the letter path in pts, use it to move the robot
        moveRobotAlongPath(robot, pts);
    end

    hold off;
end

%% Function to move robot along a path
function moveRobotAlongPath(robot, path)
    % Start from home configuration
    q0 = homeConfiguration(robot);

    % Number of points in the path
    numPoints = size(path, 1);

    % Initialize IK solver
    gik = generalizedInverseKinematics;
    gik.RigidBodyTree = robot;
    posConstraint = constraintPositionTarget("end_effector");
    gik.ConstraintInputs = {"position"};

    % Compute initial solution for visualization
    posConstraint.TargetPosition = path(1, :);
    [q, solutionInfo] = gik(q0, posConstraint);

    % Show robot at first point
    show(robot, q, 'PreservePlot', false);
    pause(0.5);

    % Animation parameters
    frameRate = 30;
    stepsPerSegment = 5; % Number of steps between each path point

    % Initialize animation
    totalSteps = (numPoints - 1) * stepsPerSegment;
    eeTrajectory = zeros(totalSteps, 3);
    allConfigs = zeros(length(q0), totalSteps);

    % Generate smooth trajectory
    stepIdx = 1;

    for i = 1:(numPoints - 1)
        startPos = path(i, :);
        endPos = path(i + 1, :);

        for j = 1:stepsPerSegment
            t = j / stepsPerSegment;
            targetPos = (1 - t) * startPos + t * endPos;

            posConstraint.TargetPosition = targetPos;

            % Use previous configuration as initial guess
            if i == 1 && j == 1
                initialGuess = q0;
            else
                initialGuess = allConfigs(:, max(1, stepIdx - 1));
            end

            % Solve IK
            [q, solutionInfo] = gik(initialGuess, posConstraint);

            % Store configuration and end effector position
            allConfigs(:, stepIdx) = q;
            tform = getTransform(robot, q, 'end_effector');
            eeTrajectory(stepIdx, :) = tform2trvec(tform);

            stepIdx = stepIdx + 1;
        end

    end

    % Plot full trajectory
    plot3(eeTrajectory(:, 1), eeTrajectory(:, 2), eeTrajectory(:, 3), 'g-', 'LineWidth', 1.5);

    % Animate robot along trajectory
    eeMarker = [];
    currentTrace = [];

    for i = 1:size(allConfigs, 2)
        % Update robot visualization
        show(robot, allConfigs(:, i), 'PreservePlot', false);

        % Update end effector marker
        if ~isempty(eeMarker) && isvalid(eeMarker)
            delete(eeMarker);
        end

        eeMarker = plot3(eeTrajectory(i, 1), eeTrajectory(i, 2), eeTrajectory(i, 3), 'g.', 'MarkerSize', 15);

        % Update trace
        if ~isempty(currentTrace) && isvalid(currentTrace)
            delete(currentTrace);
        end

        currentTrace = plot3(eeTrajectory(1:i, 1), eeTrajectory(1:i, 2), eeTrajectory(1:i, 3), 'b-', 'LineWidth', 2);

        % Display progress
        title(sprintf('Writing progress: %.1f%%', 100 * i / size(allConfigs, 2)));

        drawnow;
        pause(1 / frameRate);
    end

end

%% Function to write text (multiple letters)
function writeText(robot, text, startPos, letterSize, letterSpacing, writingPlane)
    figure('Position', [100, 100, 900, 700]);
    h = show(robot);
    hold on;

    % Set view and lighting
    view(45, 30);
    light('Position', [1 1 5], 'Style', 'infinite');
    light('Position', [-3 1 5], 'Style', 'infinite');

    % Set axis limits based on workspace
    axis([-0.3 0.3 -0.3 0.3 0 0.7]);
    grid on;
    title('Robot Writing Text', 'FontSize', 14);

    currentPos = startPos;

    % Display which text we're writing
    disp(['Writing text: "', text, '"']);

    % Loop through each character in the text
    for i = 1:length(text)
        % Get current letter and display it
        currentLetter = text(i);
        disp(['Writing letter: "', currentLetter, '"']);

        % Get letter path
        letterPts = generateLetterPath(currentLetter, currentPos, letterSize, writingPlane);

        % Plot the letter path
        plot3(letterPts(:, 1), letterPts(:, 2), letterPts(:, 3), 'r-', 'LineWidth', 2);

        % Write the letter
        writeLetter(robot, letterPts, h);

        % Move to the next letter position
        currentPos(1) = currentPos(1) + letterSpacing;
    end

    hold off;
end

%% Function to write a single letter
function writeLetter(robot, letterPath, robotHandle)
    % Start from home configuration
    q0 = homeConfiguration(robot);

    % Number of points in the letter
    numPoints = size(letterPath, 1);

    % Preallocate arrays for joint configurations
    allConfigs = zeros(length(q0), numPoints);

    % Initialize IK solver
    gik = generalizedInverseKinematics;
    gik.RigidBodyTree = robot;
    posConstraint = constraintPositionTarget("end_effector");
    gik.ConstraintInputs = {"position"};

    % Compute IK for each point in the letter path
    for i = 1:numPoints
        targetPosition = letterPath(i, :);
        posConstraint.TargetPosition = targetPosition;

        % Use previous configuration as initial guess (if available)
        if i == 1
            initialGuess = q0;
        else
            initialGuess = allConfigs(:, i - 1);
        end

        % Solve IK
        [q, solutionInfo] = gik(initialGuess, posConstraint);

        % Check if solution is valid
        if solutionInfo.ExitFlag < 0
            warning('IK failed at point %d with error: %s', i, solutionInfo.Status);
            % Use previous solution if available, otherwise use initial guess
            if i > 1
                q = allConfigs(:, i - 1);
            else
                q = q0;
            end

        end

        % Store configuration
        allConfigs(:, i) = q;
    end

    % Animation parameters
    frameRate = 30;
    stepsPerSegment = 10; % Number of steps between each path point

    % Initialize animation
    totalSteps = (numPoints - 1) * stepsPerSegment;
    eeTrajectory = zeros(totalSteps, 3);
    allTrajectoryConfigs = zeros(length(q0), totalSteps);

    % Generate smooth trajectory between all points
    step = 1;

    for i = 1:(numPoints - 1)
        % Interpolate between current point and next point
        for j = 1:stepsPerSegment
            t = j / stepsPerSegment; % Interpolation parameter [0,1]

            % Linearly interpolate joint angles
            config = (1 - t) * allConfigs(:, i) + t * allConfigs(:, i + 1);
            allTrajectoryConfigs(:, step) = config;

            % Compute end effector position for this configuration
            tform = getTransform(robot, config, 'end_effector');
            eeTrajectory(step, :) = tform2trvec(tform);

            step = step + 1;
        end

    end

    % Plot full end effector trajectory
    traceHandle = plot3(eeTrajectory(:, 1), eeTrajectory(:, 2), eeTrajectory(:, 3), 'g-', 'LineWidth', 1.5);

    % Animate robot along the trajectory
    eeMarker = [];
    currentTrace = [];

    for i = 1:totalSteps
        % Update robot visualization
        show(robot, allTrajectoryConfigs(:, i), 'PreservePlot', false, 'Parent', gca, 'Frames', 'off');

        % Delete previous markers if they exist
        if ~isempty(eeMarker) && isvalid(eeMarker)
            delete(eeMarker);
        end

        % Add current end effector position marker
        eeMarker = plot3(eeTrajectory(i, 1), eeTrajectory(i, 2), eeTrajectory(i, 3), 'g.', 'MarkerSize', 20);

        % Update trace up to current point
        if ~isempty(currentTrace) && isvalid(currentTrace)
            delete(currentTrace);
        end

        currentTrace = plot3(eeTrajectory(1:i, 1), eeTrajectory(1:i, 2), eeTrajectory(1:i, 3), 'b-', 'LineWidth', 2);

        % Update progress
        progressText = sprintf('Writing progress: %.1f%%', 100 * i / totalSteps);
        title(progressText);

        drawnow;
        pause(1 / frameRate);
    end

end

%% Function to generate letter path
function points = generateLetterPath(letter, startPos, size, zHeight)
    % Default Z height if not specified
    if nargin < 4
        zHeight = startPos(3);
    end

    % Scale factor for letter size
    scale = size;

    % Base letter coordinates (normalized from 0 to 1)
    switch upper(letter)
        case 'A'
            rawPts = [
                      0.5, 0.0, 0; % Bottom point of the A
                      0.0, 1.0, 0; % Top left
                      1.0, 1.0, 0; % Top right
                      0.5, 0.0, 0; % Back to bottom
                      0.25, 0.5, 0; % Start of cross bar
                      0.75, 0.5, 0 % End of cross bar
                      ];
        case 'B'
            rawPts = [
                      0.0, 0.0, 0; % Bottom left
                      0.0, 1.0, 0; % Top left
                      0.75, 1.0, 0; % Top right
                      1.0, 0.85, 0; % Top curve
                      0.75, 0.7, 0; % Top middle right
                      0.0, 0.5, 0; % Middle left
                      0.75, 0.3, 0; % Bottom middle right
                      1.0, 0.15, 0; % Bottom curve
                      0.75, 0.0, 0; % Bottom right
                      0.0, 0.0, 0 % Back to bottom left
                      ];
        case 'C'
            rawPts = [
                      0.9, 0.8, 0; % Top right
                      0.7, 1.0, 0; % Top curve
                      0.3, 1.0, 0; % Top left curve
                      0.1, 0.8, 0; % Top left
                      0.0, 0.6, 0; % Left top
                      0.0, 0.4, 0; % Left bottom
                      0.1, 0.2, 0; % Bottom left
                      0.3, 0.0, 0; % Bottom left curve
                      0.7, 0.0, 0; % Bottom curve
                      0.9, 0.2, 0 % Bottom right
                      ];
        case 'H'
            rawPts = [
                      0.0, 0.0, 0; % Bottom left
                      0.0, 1.0, 0; % Top left
                      0.0, 0.5, 0; % Middle left
                      1.0, 0.5, 0; % Middle right
                      1.0, 1.0, 0; % Top right
                      1.0, 0.0, 0 % Bottom right
                      ];
        case 'I'
            rawPts = [
                      0.0, 0.0, 0; % Bottom left
                      1.0, 0.0, 0; % Bottom right
                      0.5, 0.0, 0; % Bottom middle
                      0.5, 1.0, 0; % Top middle
                      0.0, 1.0, 0; % Top left
                      1.0, 1.0, 0 % Top right
                      ];
        case 'O'
            theta = linspace(0, 2 * pi, 20);
            x = 0.5 + 0.5 * cos(theta);
            y = 0.5 + 0.5 * sin(theta);
            z = zeros(size(theta));
            rawPts = [x', y', z'];
        case 'T'
            rawPts = [
                      0.0, 1.0, 0; % Top left
                      1.0, 1.0, 0; % Top right
                      0.5, 1.0, 0; % Top middle
                      0.5, 0.0, 0 % Bottom middle
                      ];
        otherwise
            % Default to a simple square if letter not defined
            rawPts = [
                      0.0, 0.0, 0;
                      1.0, 0.0, 0;
                      1.0, 1.0, 0;
                      0.0, 1.0, 0;
                      0.0, 0.0, 0
                      ];
            warning('Letter "%s" not defined, using a square.', letter);
    end

    % Make sure rawPts is not empty to avoid indexing errors
    if (isempty(rawPts))
        rawPts = [0.0, 0.0, 0; 1.0, 1.0, 0]; % Default to diagonal line if empty
    end

    % Scale and position the points
    points = rawPts;
    points(:, 1) = startPos(1) + scale * rawPts(:, 1);
    points(:, 2) = startPos(2) + scale * rawPts(:, 2);
    points(:, 3) = zHeight * ones(size(rawPts, 1), 1); % Set Z height
end

%% Function to visualize the robot workspace
function visualizeWorkspace(robot, numSamples)
    figure('Position', [100, 100, 800, 600]);
    h = show(robot);
    title("Robot Workspace");
    hold on;

    % Sample configurations for workspace visualization
    configSamples = cell(numSamples, 1);

    for i = 1:numSamples
        sample = randomConfiguration(robot);
        % limit joint 4 range (assuming this is necessary from original code)
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

    % Draw a writing plane to visualize where letters will be written
    [X, Y] = meshgrid([-0.3:0.1:0.3], [-0.3:0.1:0.3]);
    Z = ones(size(X)) * 0.3; % Writing plane height
    mesh(X, Y, Z, 'FaceAlpha', 0.2, 'EdgeColor', [0.7 0.7 0.7], 'FaceColor', [0.9 0.9 1]);

    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    grid on;
    view(45, 30);

    axis([-0.3 0.3 -0.3 0.3 0 0.7]);

    % Add a title explaining the workspace
    title('Robot Workspace with Writing Plane at Z=0.3');
end
