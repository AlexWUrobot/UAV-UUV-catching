function count = points_inside_trapezoid(points, directions_scaled, x, y, z)
r = 2; 
R = 5; 
h = 15; 

theta = linspace(0, 2*pi, 25);

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

circle2_center = R_matrix * [h;0;0] + [x;y;z]; 

vector_between_2_circles = R_matrix * [h;0;0];
vector_between_2_circles_norm = vector_between_2_circles/norm(vector_between_2_circles);

%% Check each point

count = 0; % Initialize count of points inside the trapezoid
for i = 1:size(points, 1)
    point = points(i, :)'; % Get the current point

    % Project the point onto the trapezoid's axis
    relative_point = point - [x; y; z];
    %projection = dot(relative_point, v);
    projection = dot(relative_point, vector_between_2_circles_norm);
    


    % Check if the point is between the two circles (height condition)
    if projection < 0 || projection > h
        continue;
    end

    % Find the radius of the circle at the projected height
    interpolated_radius = r + (R - r) * (projection / h);

    % Calculate the perpendicular distance from the point to the axis
    distance_to_axis = norm(cross(relative_point, vector_between_2_circles_norm)) / norm(vector_between_2_circles_norm);

    % Check if the point is within the circle at this height
    if distance_to_axis <= interpolated_radius
        count = count + 1; % Increment the count
    end
end


%% Plot the circles at the two ends
plot3(x + yc1_rot(1, :), y + yc1_rot(2, :), z + yc1_rot(3, :), 'b-', 'LineWidth', 2,'HandleVisibility','off'); % Smaller circle
plot3(x + yc2_rot(1, :), y + yc2_rot(2, :), z + yc2_rot(3, :), 'g-', 'LineWidth', 2,'HandleVisibility','off'); % Larger circle

% Plot the sides of the trapezoidal cylinder
for i = 1:length(theta)
    plot3([x + yc1_rot(1, i), x + yc2_rot(1, i)], [y + yc1_rot(2, i), y + yc2_rot(2, i)], [z + yc1_rot(3, i), z + yc2_rot(3, i)], 'c-', 'LineWidth', 1,'HandleVisibility','off');
end

% Plot the center points for reference
plot3(x, y, z, 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'r','HandleVisibility','off'); % Center point of smaller circle
%plot3(x + h * v(1), y + h * v(2), z + h * v(3), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g','HandleVisibility','off'); % Center point of larger circle

plot3(circle2_center(1),circle2_center(2),circle2_center(3),'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'g','HandleVisibility','off')

% Plot the points
for i = 1:length(points)
    plot3(points(i,1),points(i,2),points(i,3),'ro');
end

xlabel('X'); ylabel('Y'); zlabel('Z'); 
axis equal; % Ensure equal scaling on all axes
view(3)
grid on;


