function joint_positions = forwardKinematics(theta1, theta2, theta3)
    % DH transformation matrix function
    function T = DHTransform(alpha, a, d, theta)
        T = [cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta);
             sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta);
             0, sin(alpha), cos(alpha), d;
             0, 0, 0, 1];
    end

    % Compute transformation matrices
    T01 = DHTransform(-pi / 2, 0, 400, theta1);
    T12 = DHTransform(pi / 2, 0, 0, theta2);
    T23 = DHTransform(0, 0, 100, theta3);
    T34 = DHTransform(0, 100, 50, 0); % θ₄ = 0 (fixed)

    % Compute cumulative transformations
    T02 = T01 * T12;
    T03 = T02 * T23;
    T04 = T03 * T34;

    % Extract joint positions
    joint_positions = zeros(5, 3); % 5 points: base + 4 joints

    % Base position
    joint_positions(1, :) = [0, 0, 0];

    % Joint positions
    joint_positions(2, :) = T01(1:3, 4)';
    joint_positions(3, :) = T02(1:3, 4)';
    joint_positions(4, :) = T03(1:3, 4)';
    joint_positions(5, :) = T04(1:3, 4)';
end