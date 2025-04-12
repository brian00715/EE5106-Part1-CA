function MoveRobotAlongPath(robot, path, displayTrajectory, frames)

    if nargin < 3
        displayTrajectory = true; % Default to displaying trajectory
    end

    if nargin < 4
        frames = 'on'; % Default to showing frames
    end

    % Start from home configuration
    q0 = homeConfiguration(robot);

    % Initialize IK solver
    gik = generalizedInverseKinematics;
    gik.RigidBodyTree = robot;
    posConstraint = constraintPositionTarget("end_effector");
    gik.ConstraintInputs = {"position"};

    % Animation parameters
    frameRate = 120;
    stepsPerSegment = 5;

    % Find segments separated by NaN points
    nanIdx = find(any(isnan(path), 2));
    segments = {};
    startIdx = 1;

    % Split path into segments at NaN points
    for i = 1:length(nanIdx)

        if nanIdx(i) > startIdx
            segments{end + 1} = path(startIdx:nanIdx(i) - 1, :);
        end

        startIdx = nanIdx(i) + 1;
    end

    % Add final segment if it exists
    if startIdx <= size(path, 1)
        segments{end + 1} = path(startIdx:end, :);
    end

    % If no NaN points found, treat entire path as one segment
    if isempty(segments)
        segments{1} = path;
    end

    % Process each segment
    for segIdx = 1:length(segments)
        currentSegment = segments{segIdx};

        if size(currentSegment, 1) < 2
            continue; % Skip segments with less than 2 points
        end

        % Calculate total steps needed for this segment
        numPoints = size(currentSegment, 1);
        totalSteps = (numPoints - 1) * stepsPerSegment;

        % Initialize arrays for trajectory
        allConfigs = zeros(length(q0), totalSteps);
        eeTrajectory = zeros(totalSteps, 3);

        % Generate trajectory for this segment
        stepIdx = 1;

        for i = 1:(numPoints - 1)
            startPos = currentSegment(i, :);
            endPos = currentSegment(i + 1, :);

            for j = 1:stepsPerSegment
                t = j / stepsPerSegment;
                targetPos = (1 - t) * startPos + t * endPos;

                posConstraint.TargetPosition = targetPos;

                % Use previous configuration as initial guess
                if stepIdx == 1 && segIdx == 1
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

        % Animate the motion for this segment
        for i = 1:totalSteps
            % Update robot visualization with frames parameter
            show(robot, allConfigs(:, i), 'PreservePlot', false, 'Frames', frames);

            if displayTrajectory
                % Only display trajectory points that are on the writing plane
                if i > 1 % We need at least 2 points to draw a line
                    % Get current and previous points
                    currentPoint = eeTrajectory(i, :);
                    prevPoint = eeTrajectory(i - 1, :);

                    % Only draw line if both points are on the writing plane
                    if abs(currentPoint(1) - currentSegment(1, 1)) < 0.001 && ...
                            abs(prevPoint(1) - currentSegment(1, 1)) < 0.001
                        plot3([prevPoint(1), currentPoint(1)], ...
                            [prevPoint(2), currentPoint(2)], ...
                            [prevPoint(3), currentPoint(3)], ...
                            'b-', 'LineWidth', 2.8);
                    end

                end

            end

            drawnow;
            pause(1 / frameRate);
        end

        % Small pause between segments
        pause(0.05);
    end

end
