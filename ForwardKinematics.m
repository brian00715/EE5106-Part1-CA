function T_rst = forwardKinematics(dh, q)
    % Generic forward kinematics using DH parameters
    % Input:
    %   dh: DH parameter matrix [n x 4] (alpha, a, d, theta_offset)
    %   q: Joint positions vector [n x 1]
    % Output:
    %   T_rst: Transformation matrix [4 x 4] of the end effector
    
    % Get number of joints
    n = size(dh, 1);
    
    % Initialize transformation matrix
    T = eye(4);
    
    % DH transformation matrix function
    function T = DHTransform(alpha, a, d, theta)
        T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
             sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
             0, sin(alpha), cos(alpha), d;
             0, 0, 0, 1];
    end
    
    % Compute cumulative transformation
    for i = 1:n
        alpha = dh(i,1);
        a = dh(i,2);
        d = dh(i,3);
        theta = q(i) + dh(i,4);  % Add theta offset from DH parameters
        
        Ti = DHTransform(alpha, a, d, theta);
        T = T * Ti;
    end
    
    if 0
        % Extract position
        position = T(1:3,4)';
        
        % Extract orientation (roll, pitch, yaw) from rotation matrix
        R = T(1:3,1:3);
        
        % Convert rotation matrix to Euler angles (ZYX convention)
        pitch = atan2(-R(3,1), sqrt(R(3,2)^2 + R(3,3)^2));
        yaw = atan2(R(2,1)/cos(pitch), R(1,1)/cos(pitch));
        roll = atan2(R(3,2)/cos(pitch), R(3,3)/cos(pitch));
        
        % Combine position and orientation
        ee_pose = [position, roll, pitch, yaw];
    end
    T_rst = T;
end

