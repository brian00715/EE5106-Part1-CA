clc; clear all;

robot = CreateModel();
ModelVisualization(robot);
show(robot);

%theta1_range = linspace(-pi, pi, 20);
%theta2_range = linspace(-pi, pi, 20);
%theta3_range = linspace(-pi, pi, 20);

%for t1 = theta1_range
%    for t2 = theta2_range
%        for t3 = theta3_range
%            config = [t1; t2; t3];
%            eeTform = getTransform(robot, config, 'end_effector');
%            pos = tform2trvec(eeTform);
%            plot3(pos(1), pos(2), pos(3), 'r.');
%        end
%    end
%end

%xlabel('X'); ylabel('Y'); zlabel('Z');
%title('End Effector Workspace Points');
%axis equal; grid on;
