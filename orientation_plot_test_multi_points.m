% Example data (replace with your actual data)
num_points = 5;  % Number of points
x = rand(1, num_points);  % Random x-coordinates
y = rand(1, num_points);  % Random y-coordinates
z = rand(1, num_points);  % Random z-coordinates
qw = rand(1, num_points); % Random quaternion components
qx = rand(1, num_points);
qy = rand(1, num_points);
qz = rand(1, num_points);
% Scatter plot
scatter3(x, y, z, 'filled');
hold on;

% Compute the direction vector from the quaternion
directions = [2 * (qx .* qz + qw .* qy);
              2 * (qy .* qz - qw .* qx);
              1 - 2 * (qx.^2 + qy.^2)];

% Scale the direction vector (adjust the scaling factor as needed)
scale_factor = 0.1;
directions_scaled = scale_factor * directions;



% Add an arrow from the point (x, y, z) in the direction of the quaternion
quiver3(x, y, z, directions_scaled(1, :), directions_scaled(2, :), directions_scaled(3, :), 'r', 'LineWidth', 2);

% Set axis labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Orientation of Points in 3D Scatter Plot');

% Adjust view for better visibility
view(30, 30);
grid on;
hold off;


