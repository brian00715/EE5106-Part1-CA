%% Robot and Work Plane Visualization (No Toolbox Required)
close all; clear; clc;

% Create figure
figure('Position', [100, 100, 1000, 800]);
hold on;
grid on;
axis equal;

%% Define the robot parameters
% DH parameters
% Link 1: α₁ = -90°, a₁ = 0, d₁ = 400 mm, θ₁ = variable
% Link 2: α₂ = 90°, a₂ = 0, d₂ = 0, θ₂ = variable
% Link 3: α₃ = 0, a₃ = 0, d₃ = 100 mm, θ₃ = variable
% Link 4: α₄ = 0, a₄ = 100 mm, d₄ = 50 mm, θ₄ = 0

% a, alpha, d, theta
dhparams = [
            0, -pi / 2, 400, 0; % Link 1
            0, pi / 2, 0, 0; % Link 2
            0, 0, 100, 0; % Link 3
            100, 0, 50, 0 % Link 4
            ];

% Set joint angles (in radians)
theta1 = pi / 6; % 30 degrees
theta2 = pi / 4; % 45 degrees
theta3 = pi / 3; % 60 degrees

% Forward kinematics function
joint_positions = forwardKinematics(theta1, theta2, theta3);

%% Plot the robot
% Base
plot3(0, 0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(5, 5, 5, 'Base', 'FontSize', 12);

% Links
colors = {'r', 'g', 'b', 'm'};

for i = 1:size(joint_positions, 1) - 1
    plot3([joint_positions(i, 1), joint_positions(i + 1, 1)], ...
        [joint_positions(i, 2), joint_positions(i + 1, 2)], ...
        [joint_positions(i, 3), joint_positions(i + 1, 3)], ...
        colors{i}, 'LineWidth', 3);

    plot3(joint_positions(i + 1, 1), joint_positions(i + 1, 2), joint_positions(i + 1, 3), ...
        'ko', 'MarkerSize', 8, 'MarkerFaceColor', colors{i});

    text(joint_positions(i + 1, 1) + 10, joint_positions(i + 1, 2) + 10, joint_positions(i + 1, 3) + 10, ...
        ['Joint ', num2str(i)], 'FontSize', 10);
end

%% Draw work plane - Changed to YZ plane (vertical plane)
% Transform matrix for YZ plane at X = 100mm
T_plane_to_base = [0, 0, -1, 100;  % X fixed at 100mm, plane normal along X
                   0, 1, 0, 0;      % Y axis stays as Y axis
                   1, 0, 0, 200;    % Z axis position
                   0, 0, 0, 1];
plane_origin = T_plane_to_base(1:3, 4)';
plane_width = 600; % mm (width along Y axis)
plane_height = 600; % mm (height along Z axis)

% Create the plane vertices for YZ plane (X is fixed, Y and Z vary)
plane_vertices = [plane_origin;
                  plane_origin + [0, plane_width, 0];
                  plane_origin + [0, plane_width, plane_height];
                  plane_origin + [0, 0, plane_height]];

% Create patches for the plane
patch('Vertices', plane_vertices, 'Faces', [1 2 3 4], ...
    'FaceColor', [0.8 0.8 0.9], 'FaceAlpha', 0.5);

% Draw coordinate frames
% Robot Base Frame
drawFrame([0 0 0], eye(3), 100, 'b');

% Work Plane Frame - Rotation matrix for YZ plane
R_plane = [0 0 1;   % X points into the board (normal to YZ plane)
           0 1 0;   % Y stays as Y
           1 0 0];  % Z points up along original X

% Draw the writing plane frame
drawFrame(plane_origin, R_plane, 100, 'r');

% Add text for the frames
text(0, 0, 50, 'Robot Base Frame', 'FontSize', 12);
text(plane_origin(1) + 10, plane_origin(2), plane_origin(3) + 50, 'YZ Writing Plane', 'FontSize', 12);

% Set view
view(30, 30);
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('3DOF Robot with Vertical YZ Writing Plane');
xlim([-400 400]);
ylim([-200 600]);
zlim([-200 600]);

% Draw a grid on the YZ plane to make it more visible
[Y, Z] = meshgrid(0:100:plane_width, 0:100:plane_height);
X = ones(size(Y)) * plane_origin(1);
mesh(X, Y + plane_origin(2), Z + plane_origin(3), 'FaceAlpha', 0, 'EdgeColor', [0.5 0.5 0.7], 'LineWidth', 1);

%% Helper function to draw coordinate frames
function drawFrame(origin, R, length, color)
    % X-axis
    quiver3(origin(1), origin(2), origin(3), ...
        R(1, 1) * length, R(2, 1) * length, R(3, 1) * length, color, 'LineWidth', 2);
    % Y-axis
    quiver3(origin(1), origin(2), origin(3), ...
        R(1, 2) * length, R(2, 2) * length, R(3, 2) * length, color, 'LineWidth', 2);
    % Z-axis
    quiver3(origin(1), origin(2), origin(3), ...
        R(1, 3) * length, R(2, 3) * length, R(3, 3) * length, color, 'LineWidth', 2);

    % Add axis labels
    text(origin(1) + R(1, 1) * length * 1.1, origin(2) + R(2, 1) * length * 1.1, origin(3) + R(3, 1) * length * 1.1, 'X', 'Color', color);
    text(origin(1) + R(1, 2) * length * 1.1, origin(2) + R(2, 2) * length * 1.1, origin(3) + R(3, 2) * length * 1.1, 'Y', 'Color', color);
    text(origin(1) + R(1, 3) * length * 1.1, origin(2) + R(2, 3) * length * 1.1, origin(3) + R(3, 3) * length * 1.1, 'Z', 'Color', color);
end
