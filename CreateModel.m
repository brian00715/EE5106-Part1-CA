function robot = CreateModel()

    robot = rigidBodyTree('DataFormat', 'column');

    % a alpha d theta
    dhparams = [
                0, -pi / 2, 0.4, 0; % Link 1
                0, pi / 2, 0, 0; % Link 2
                0, 0, 0.1, 0; % Link 3
                0, 0, 0, 0 % Link 4
                0.1, 0, 0, 0 % end_effector
                ];

    joint1 = rigidBodyJoint('joint1', 'revolute');
    setFixedTransform(joint1, dhparams(1, :), "dh");
    joint2 = rigidBodyJoint('joint2', 'revolute');
    setFixedTransform(joint2, dhparams(2, :), "dh");
    joint3 = rigidBodyJoint('joint3', 'revolute');
    setFixedTransform(joint3, dhparams(3, :), "dh");
    joint4 = rigidBodyJoint('joint4', 'prismatic');
    setFixedTransform(joint4, dhparams(4, :), "dh");
    joint5 = rigidBodyJoint('joint5', 'fixed');
    setFixedTransform(joint5, dhparams(5, :), "dh");

    link1 = rigidBody('link1');
    link1.Joint = joint1;
    addBody(robot, link1, robot.BaseName);

    link2 = rigidBody('link2');
    link2.Joint = joint2;
    addBody(robot, link2, 'link1');

    link3 = rigidBody('link3');
    link3.Joint = joint3;
    addBody(robot, link3, 'link2');

    link4 = rigidBody('link4');
    link4.Joint = joint4;
    addBody(robot, link4, 'link3');

    end_effector = rigidBody('end_effector');
    end_effector.Joint = joint5;
    addBody(robot, end_effector, 'link4');

    ModelVisualization(robot);
end
