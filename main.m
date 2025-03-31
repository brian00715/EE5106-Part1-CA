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
        joints{i} = rigidBodyJoint(['joint' num2str(i)], 'fixed');
    end

    % Set the joint's DH parameters
    setFixedTransform(joints{i}, dhparams(i, :), 'dh');

    % Attach the joint to the body
    bodies{i}.Joint = joints{i};

    if i == 1
        addBody(robot, bodies{i}, 'base');
    else
        addBody(robot, bodies{i}, ['body' num2str(i - 1)]);
    end

end

showdetails(robot)

% figure(Name = "PUMA Robot Model")
% show(robot);

figure(Name="Interactive GUI")
gui = interactiveRigidBodyTree(robot,MarkerScaleFactor=0.5);
