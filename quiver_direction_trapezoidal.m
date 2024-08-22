clc;
clear all;
close all;

x = 1; 
y = 2; 
z = 3; 

pitch = deg2rad(45);
q = [cos(pitch/2), 0, sin(pitch/2), 0];

% directions = [2 * (q(1) * q(4) + q(2) * q(3));
%               2 * (q(2) * q(4) - q(1) * q(3));
%               1 - 2 * (q(4)^2 + q(2)^2)];


qw = q(1);
qx = q(2);
qy = q(3);
qz = q(4);

% Compute the direction vector from the quaternion
directions = [2 * (qx .* qz + qw .* qy);
              2 * (qy .* qz - qw .* qx);
              1 - 2 * (qx.^2 + qy.^2)];

scale_factor = 10;
directions_scaled = scale_factor * directions;

r = 2; 
R = 5; 
h = 10; 

theta = linspace(0, 2*pi, 50);

% Calculate the rotation matrix from the direction vector
v = directions_scaled / norm(directions_scaled);
v0 = [0; 0; 1];  % Original direction (along x-axis)

% Calculate the axis of rotation
axis_of_rotation = cross(v0, v);
axis_of_rotation = axis_of_rotation / norm(axis_of_rotation);

% Calculate the angle of rotation
angle_of_rotation = acos(dot(v0, v));

% Rotation matrix using Rodrigues' rotation formula
K = [0 -axis_of_rotation(3) axis_of_rotation(2);
     axis_of_rotation(3) 0 -axis_of_rotation(1);
     -axis_of_rotation(2) axis_of_rotation(1) 0];

R_matrix = eye(3) + sin(angle_of_rotation) * K + (1 - cos(angle_of_rotation)) * (K^2);

% Rotate the circle points
yc1_rot = R_matrix * [zeros(1, length(theta)); r * cos(theta); r * sin(theta)];
yc2_rot = R_matrix * [h * ones(1, length(theta)); R * cos(theta); R * sin(theta)];

% Create the 3D plot
figure;
quiver3(x, y, z, directions_scaled(1), directions_scaled(2), directions_scaled(3), 'r', 'LineWidth', 2);

hold on;

% Plot the circles at the two ends
plot3(x + yc1_rot(1, :), y + yc1_rot(2, :), z + yc1_rot(3, :), 'b-', 'LineWidth', 2); % Smaller circle
plot3(x + yc2_rot(1, :), y + yc2_rot(2, :), z + yc2_rot(3, :), 'g-', 'LineWidth', 2); % Larger circle

% Plot the sides of the trapezoidal cylinder
for i = 1:length(theta)
    plot3([x + yc1_rot(1, i), x + yc2_rot(1, i)], [y + yc1_rot(2, i), y + yc2_rot(2, i)], [z + yc1_rot(3, i), z + yc2_rot(3, i)], 'k-', 'LineWidth', 1);
end

% Plot the center points for0000000000000000000000000000000000000000000000000000000 reference
plot3(x, y, z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Center point of smaller circle
plot3(x + h * v(1), y + h * v(2), z + h * v(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Center point of larger circle

% Axis labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('UAV downward wind simulation (Trapezoidal Cylinder)');
legend("UAV flight direction")
grid on;
axis equal;
view(3);
hold off;