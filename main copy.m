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

robot = CreateModel();
ModelVisualization(robot);

showdetails(robot)

q = [0, 0, 0, 0];
config = homeConfiguration(robot);

for i = 1:length(config)
    config(i) = q(i);
end

T = getTransform(robot, config, 'end_effector', 'base');


% figure(Name="Interactive GUI")
% gui = interactiveRigidBodyTree(robot, MarkerScaleFactor = 0.5);
