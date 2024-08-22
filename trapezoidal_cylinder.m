% Given point coordinates and dimensions
x = 1; % example value for x
y = 2; % example value for y
z = 3; % example value for z

pq = pitch_to_quaternion(45);
qw = pq(1);
qx = pq(2);
qy = pq(3);
qz = pq(4);
% qw = 1; % example quaternion w-component
% qx = 0; % example quaternion x-component
% qy = 0; % quaternion y-component (zero for vertical to x-z plane)
% qz = 0; % example quaternion z-component



r = 2; % radius of the smaller circle
R = 5; % radius of the larger circle
h = 10; % height of the cylinder along the x-axis

% Define the angle range for the circles
theta = linspace(0, 2*pi, 50);

% Calculate the points for the two circles
yc1 = y + r * cos(theta);
zc1 = z + r * sin(theta);
yc2 = y + R * cos(theta);
zc2 = z + R * sin(theta);

% Create the 3D plot
figure;
hold on;

% Plot the circles at the two ends
plot3(x * ones(size(theta)), yc1, zc1, 'b-', 'LineWidth', 2); % Smaller circle
plot3((x + h) * ones(size(theta)), yc2, zc2, 'g-', 'LineWidth', 2); % Larger circle

% Plot the sides of the trapezoidal cylinder
for i = 1:length(theta)
    plot3([x, x + h], [yc1(i), yc2(i)], [zc1(i), zc2(i)], 'k-', 'LineWidth', 1);
end

% Plot the center points for reference
plot3(x, y, z, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r'); % Center point of smaller circle
plot3(x + h, y, z, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g'); % Center point of larger circle

% Axis labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Trapezoidal Cylinder Perpendicular to X-Z Plane');
grid on;
axis equal;
view(3);
hold off;