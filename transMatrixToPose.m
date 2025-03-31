function [pos, ori] = transMatrixToPose(T)
    % transMatrixToPose - Convert a 4x4 transformation matrix to a position and orientation
    pos = T(1:3, 4)';

    % Extract orientation (roll, pitch, yaw) from rotation matrix
    R = T(1:3, 1:3);

    % Convert rotation matrix to Euler angles (ZYX convention)
    pitch = atan2(-R(3, 1), sqrt(R(3, 2) ^ 2 + R(3, 3) ^ 2));
    yaw = atan2(R(2, 1) / cos(pitch), R(1, 1) / cos(pitch));
    roll = atan2(R(3, 2) / cos(pitch), R(3, 3) / cos(pitch));

    ori = [roll, pitch, yaw];
end
