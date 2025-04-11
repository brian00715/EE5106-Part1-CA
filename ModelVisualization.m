function ModelVisualization(robot)

    addVisual(robot.Base, 'box', [0.08 0.08 0.3], trvec2tform([0 0 0.15]));
    addVisual(getBody(robot, 'link1'), 'box', [0.08 0.1 0.08], trvec2tform([0 0.05 0]));
    addVisual(getBody(robot, 'link2'), 'cylinder', [0.05 0.08], rotm2tform(rotx(90)));
    addVisual(getBody(robot, 'link3'), 'cylinder', [0.015 0.1], trvec2tform([0 0 -0.02]));
    addVisual(getBody(robot, 'link3'), 'cylinder', [0.03 0.02]);
    addVisual(getBody(robot, 'link4'), 'cylinder', [0.015 0.3], trvec2tform([0 0 -0.15]));
    addVisual(getBody(robot, 'link4'), 'box', [0.03 0.06 0.01], trvec2tform([0 0 0.0]));
    addVisual(getBody(robot, 'link4'), 'box', [0.03 0.02 0.05], trvec2tform([0 0.02 0.0]));
    addVisual(getBody(robot, 'link4'), 'box', [0.03 0.02 0.05], trvec2tform([0 -0.02 0.0]));

    T = rotm2tform(roty(90)) * trvec2tform([0 0 -0.08]);
    addVisual(getBody(robot, 'end_effector'), 'cylinder', [0.01, 0.15], T);
    addVisual(getBody(robot, 'end_effector'), 'sphere', 0.01, trvec2tform([-0.005 0 0]));
    addCollision(getBody(robot, "end_effector"), "sphere", 0.01, trvec2tform([-0.005 0 0]));

end
