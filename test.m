% Example DH parameters for a 3-DOF robot
dh = [
    -pi/2, 0, 400, 0;
    pi/2,  0, 0,   0;
    0,     0, 100, 0
];

% Joint positions
q = [0.5, 0.7, 0.3];

% Calculate forward kinematics
ee_pose = forwardKinematics(dh, q);