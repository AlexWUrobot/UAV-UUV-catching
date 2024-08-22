% Given point coordinates and radius
x = 1; % example value for x
y = 2; % example value for y
z = 3; % example value for z
qw = 1; % example quaternion w-component
qx = 0; % example quaternion x-component
qy = 0; % quaternion y-component (zero for vertical to x-z plane)
qz = 0; % example quaternion z-component
r = 5; % example radius

% Define the angle range for the circle
theta = linspace(0, 2*pi, 100);

% Calculate the points on the circle in the y-z plane
yc = y + r * cos(theta);
zc = z + r * sin(theta);

% Create the 3D plot
figure;
hold on;
plot3(x * ones(size(theta)), yc, zc, 'b-', 'LineWidth', 2); % Circle
plot3(x, y, z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Center point

% Axis labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Circle Perpendicular to X-Z Plane');
grid on;
axis equal;
view(3);
hold off;