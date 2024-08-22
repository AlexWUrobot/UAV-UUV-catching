% rotatable


% Given point coordinates, quaternion, and dimensions
x = 1; % example value for x
y = 2; % example value for y
z = 3; % example value for z
qw = 1; % example quaternion w-component
qx = 0; % example quaternion x-component
qy = 0; % example quaternion y-component
qz = 0; % example quaternion z-component
r = 2; % radius of the circle

% Direction vector from the quaternion
directions = [2 * (qx .* qz + qw .* qy);
              2 * (qy .* qz - qw .* qx);
              1 - 2 * (qx.^2 + qy.^2)];

% Define the angle range for the circle
theta = linspace(0, 2*pi, 100);

% Calculate the circle in the yz-plane (before rotation)
yc = r * cos(theta);
zc = r * sin(theta);
xc = zeros(size(theta)); % Circle is initially in the yz-plane

% Combine into a 3xN matrix of points (before rotation)
circle_points = [xc; yc; zc];

% Rotation matrix derived from quaternion
R = [1 - 2*qy^2 - 2*qz^2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw;
     2*qx*qy + 2*qz*qw, 1 - 2*qx^2 - 2*qz^2, 2*qy*qz - 2*qx*qw;
     2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx^2 - 2*qy^2];

% Apply the rotation to the circle points
rotated_circle_points = R * circle_points;

% Translate the circle to the given point (x, y, z)
xc_rotated = rotated_circle_points(1, :) + x;
yc_rotated = rotated_circle_points(2, :) + y;
zc_rotated = rotated_circle_points(3, :) + z;

% Create the 3D plot
figure;
hold on;

% Plot the rotated circle
plot3(xc_rotated, yc_rotated, zc_rotated, 'b-', 'LineWidth', 2); % Circle

% Plot the direction vector
quiver3(x, y, z, directions(1), directions(2), directions(3), 'r', 'LineWidth', 2);

% Plot the center point
plot3(x, y, z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Axis labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Rotated Circle Perpendicular to Given Direction');
grid on;
axis equal;
view(3);
hold off;