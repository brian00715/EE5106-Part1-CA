clear; clc; close all;
% DH parameters
% Link 1: α₁ = -90°, a₁ = 0, d₁ = 400 mm, θ₁ = variable
% Link 2: α₂ = 90°, a₂ = 0, d₂ = 0, θ₂ = variable
% Link 3: α₃ = 0, a₃ = 0, d₃ = 100 mm, θ₃ = variable
% Link 4: α₄ = 0, a₄ = 100 mm, d₄ = 50 mm, θ₄ = 0

% a, alpha, d, theta
dhparams = [
            0, -pi / 2, 0.4, 0; % Link 1
            0, pi / 2, 0, 0; % Link 2
            0, 0, 0.1, 0; % Link 3
            0.1, 0, 0.05, 0 % Link 4
            ];

robot = rigidBodyTree;

bodies = cell(1, size(dhparams, 1));
joints = cell(1, size(dhparams, 1));

for i = 1:size(dhparams, 1)
    bodies{i} = rigidBody(['body' num2str(i)]);

    if i ~= 4
        joints{i} = rigidBodyJoint(['joint' num2str(i)], 'revolute');
    else
        joints{i} = rigidBody(['tool' num2str(i)], 'fixed');
    end

    setFixedTransform(joints{i}, dhparams(i, :), 'dh');

    bodies{i}.Joint = joints{i};

    if i == 1
        addBody(robot, bodies{i}, 'base');
    else
        addBody(robot, bodies{i}, ['body' num2str(i - 1)]);
    end

end

% collsions
collBase = collisionBox(0.1, 0.1, 0.05);
collBase.Pose = trvec2tform([0, 0, 0.025]);
collLink1 = collisionCylinder(0.1, 0.1, 0.4);
collLink1.Pose = trvec2tform([0, 0, 0.2]);
collLink2 = collisionCylinder(0.1, 0.1, 0.1);
collLink2.Pose = trvec2tform([0, 0, 0.05]);
collLink3 = collisionCylinder(0.1, 0.1, 0.1);
collLink3.Pose = trvec2tform([0, 0, 0.05]);
collLink4 = collisionCylinder(0.1, 0.1, 0.05);
collLink4.Pose = trvec2tform([0, 0, 0.025]);
addCollision(bodies{1}, collBase);
addCollision(bodies{2}, collLink1);
addCollision(bodies{3}, collLink2);
addCollision(bodies{4}, collLink3);
addCollision(bodies{4}, collLink4);

showdetails(robot)

q = [0, 0, 0];
config = homeConfiguration(robot);

for i = 1:length(config)
    config(i).JointPosition = q(i);
end

T = getTransform(robot, config, 'body4', 'base');

figure(Name = "PUMA Robot Model")
show(robot);

% figure(Name="Interactive GUI")
% gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);
