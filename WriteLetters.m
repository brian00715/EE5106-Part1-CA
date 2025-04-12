function WriteLetters(robot, text, startPos, letterSize, letterSpacing, writingPlane, varargin)
    % Parse optional parameters
    p = inputParser;
    addParameter(p, 'Frames', 'on');
    parse(p, varargin{:});
    
    figure('Position', [100, 100, 900, 700]);
    show(robot, 'Frames', p.Results.Frames);
    hold on;

    % Set view and lighting
    % rview(45, 30); % Keep this viewing angle for visualization
    light('Position', [1 1 5], 'Style', 'infinite');
    light('Position', [-3 1 5], 'Style', 'infinite');

    % Set axis limits based on workspace
    axis([-0.3 0.3 -0.3 0.3 0 0.9]);
    grid on;
    title('Robot Writing Letters on YZ Plane', 'FontSize', 14);

    % Display which text we're writing
    disp(['Writing text: "', text, '"']);

    % Store last end point for transition path
    lastEndPoint = [];

    for i = 1:length(text)
        currentLetter = text(i);
        disp(['Processing letter: ', currentLetter]);
        currentPos = startPos;
        currentPos(2) = startPos(2) + (i - 1) * letterSpacing; % Move in Y direction for letter spacing

        switch upper(currentLetter)
            case 'H'
                pts = [
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2), currentPos(3) + letterSize / 2; % Middle left
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize / 2; % Middle right
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize; % Top right
                       writingPlane, currentPos(2) + letterSize, currentPos(3) % Bottom right
                       ];
                disp('Drawing letter H');

            case 'I'
                pts = [
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2) + letterSize, currentPos(3); % Bottom right
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3); % Bottom middle
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top middle
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize % Top right
                       ];
                disp('Drawing letter I');

            case 'E'
                pts = [
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top right
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3); % Bottom right
                       NaN, NaN, NaN; % Pen up
                       writingPlane, currentPos(2), currentPos(3) + letterSize / 2; % Middle left
                       writingPlane, currentPos(2) + letterSize / 2 * 0.8, currentPos(3) + letterSize / 2; % Middle right
                       ];
                disp('Drawing letter E');

            case '2'
                pts = [
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top right
                       NaN, NaN, NaN; % Pen up
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top right
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize / 2; % Middle right
                       NaN, NaN, NaN; % Pen up
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize / 2; % Middle right
                       writingPlane, currentPos(2), currentPos(3) + letterSize / 2; % Middle left
                       NaN, NaN, NaN; % Pen up
                       writingPlane, currentPos(2), currentPos(3) + letterSize / 2; % Middle left
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       NaN, NaN, NaN; % Pen up
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) % Bottom right
                       ];
                validPoints = ~any(isnan(pts), 2);
                disp('Drawing number 2 in digital display style');

            case '0'
                % Define 0 shape in YZ plane as a rectangle
                pts = [
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3); % Bottom right
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top right
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2), currentPos(3) % Back to bottom left
                       ];
                disp('Drawing number 0 as rectangle');

            case '1'
                % Define 1 shape in YZ plane
                pts = [
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3); % Bottom middle
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top middle
                       writingPlane, currentPos(2), currentPos(3) + letterSize * 0.8 % Top left slant
                       ];
                disp('Drawing number 1');

            case '6'
                % Define 6 shape in YZ plane with digital display style (6 strokes)
                pts = [
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize; % Top right
                       writingPlane, currentPos(2), currentPos(3) + letterSize; % Top left
                       writingPlane, currentPos(2), currentPos(3); % Bottom left
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3); % Bottom right
                       writingPlane, currentPos(2) + letterSize / 2, currentPos(3) + letterSize / 2; % Middle right
                       writingPlane, currentPos(2), currentPos(3) + letterSize / 2; % Middle left
                       ];
                validPoints = ~any(isnan(pts), 2);
                disp('Drawing number 6');

            case '-'
                % Define dash shape in YZ plane
                pts = [
                       writingPlane, currentPos(2), currentPos(3) + letterSize / 2; % Left
                       writingPlane, currentPos(2) + letterSize / 4, currentPos(3) + letterSize / 2 % Right
                       ];
                disp('Drawing dash');

            otherwise
                % Simple square for undefined letters (in YZ plane)
                pts = [
                       writingPlane, currentPos(2), currentPos(3);
                       writingPlane, currentPos(2) + letterSize, currentPos(3);
                       writingPlane, currentPos(2) + letterSize, currentPos(3) + letterSize;
                       writingPlane, currentPos(2), currentPos(3) + letterSize;
                       writingPlane, currentPos(2), currentPos(3)
                       ];
                disp(['Drawing default shape for letter: ', currentLetter]);
        end

        % If we have a previous end point, create transition path without displaying it
        if ~isempty(lastEndPoint)
            % Create transition path with retraction in x direction
            liftDepth = 0.05; % How far to retract in x direction
            transitionPoints = [
                                lastEndPoint; % Start from last letter's end
                                [lastEndPoint(1) - liftDepth, lastEndPoint(2), lastEndPoint(3)];
            % Move to above the start of next letter while retracted
                                [pts(1, 1) - liftDepth, pts(1, 2), pts(1, 3)];
                                pts(1, :) % Move forward to start of next letter
                                ];

            % Move robot along transition path
            MoveRobotAlongPath(robot, transitionPoints, false, p.Results.Frames); % false means don't display trajectory
        end

        % Draw the letter and its trajectory
        plot3(pts(:, 1), pts(:, 2), pts(:, 3), 'r-', 'LineWidth', 2);
        MoveRobotAlongPath(robot, pts, true, p.Results.Frames); % true means display trajectory

        % Store the end point of current letter for next transition
        lastEndPoint = pts(end, :);
    end

    hold off;
end
