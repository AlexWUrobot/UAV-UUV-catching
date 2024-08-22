% Example data (replace with your actual data)
x = 1;
y = 2;
z = 3;
qw = 0.8;
qx = 0.6;
qy = 0.4;
qz = 0.2;

% Scatter plot
scatter3(x, y, z, 'filled');
hold on;

% Compute the direction vector from the quaternion
direction = [2 * (qx*qz + qw*qy), 2 * (qy*qz - qw*qx), 1 - 2 * (qx^2 + qy^2)];

% Scale the direction vector (adjust the scaling factor as needed)
scale_factor = 0.1;
direction_scaled = scale_factor * direction;

% Add an arrow from the point (x, y, z) in the direction of the quaternion
quiver3(x, y, z, direction_scaled(1), direction_scaled(2), direction_scaled(3), 'r', 'LineWidth', 1);

% Set axis labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Orientation of Point in 3D Scatter Plot');

% Adjust view for better visibility
view(30, 30);
grid on;
hold off;