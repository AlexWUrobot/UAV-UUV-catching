% Parameters
g = 9.81; % Acceleration due to gravity (m/s^2)
mass_drone = 1.0; % Mass of the drone (kg)
mass_box = 0.5; % Mass of the box (kg)
thrust_max = 10.0; % Maximum thrust (N)

% Initial conditions
start_pos_drone = [0; 2]; % Drone's start position (x, y)
start_pos_box = [4; 0]; % Box's start position (x, y)
target_pos_box = [4; 0.1]; % Box's final position slightly above ground

% Time settings
dt = 0.05; % Time step (s)
total_time = 5; % Total simulation time (s)
num_steps = total_time / dt;

% Compute minimum jerk trajectory
t = linspace(0, total_time, num_steps);
x_traj = start_pos_drone(1) + (target_pos_box(1) - start_pos_drone(1)) * (10 * (t / total_time).^3 - 15 * (t / total_time).^4 + 6 * (t / total_time).^5);
y_traj = start_pos_drone(2) + (target_pos_box(2) - start_pos_drone(2)) * (10 * (t / total_time).^3 - 15 * (t / total_time).^4 + 6 * (t / total_time).^5);

% Main simulation loop
for i = 1:num_steps
    % Calculate thrust (opposing gravity)
    thrust = mass_box * g + mass_drone * g;
    
    % Update position
    position_drone = [x_traj(i); y_traj(i)];
    
    % Calculate velocity
    if i > 1
        velocity_drone = sqrt((x_traj(i) - x_traj(i-1))^2 + (y_traj(i) - y_traj(i-1))^2) / dt;
    else
        velocity_drone = 0;
    end
    
    % Plot the animation
    plot(position_drone(1), position_drone(2), 'bo', 'MarkerSize', 10);
    hold on;
    plot(start_pos_box(1), start_pos_box(2), 'ro', 'MarkerSize', 10);
    hold on;
    plot([position_drone(1), start_pos_box(1)], [position_drone(2), start_pos_box(2)], 'r--');
    xlim([-1, 6]);
    ylim([-1, 6]);
    xlabel('X position (m)');
    ylabel('Y position (m)');
    title(sprintf('Drone Approaching the AUV\nTime: %.2f s, Velocity: %.2f m/s', t(i), velocity_drone));
    %title('Drone Approaching the AUV');
    grid on;
    pause(0.01); % Animation speed
    if i < num_steps
        cla; % Clear previous frame
    end
end
%hold off;

% Initial conditions
start_pos_drone = [4; 0.1]; % Drone's start position (x, y)
start_pos_box = [4; 0]; % Box's start position (x, y)
target_pos_box = [6; 4]; % Box's final position slightly above ground

% Time settings
%dt = 0.02; % Time step (s)
total_time = 5; % Total simulation time (s)
num_steps = total_time / dt;

% Compute minimum jerk trajectory
t = linspace(0, total_time, num_steps);
x_traj = start_pos_drone(1) + (target_pos_box(1) - start_pos_drone(1)) * (10 * (t / total_time).^3 - 15 * (t / total_time).^4 + 6 * (t / total_time).^5);
y_traj = start_pos_drone(2) + (target_pos_box(2) - start_pos_drone(2)) * (10 * (t / total_time).^3 - 15 * (t / total_time).^4 + 6 * (t / total_time).^5);

hold on;
% Main simulation loop
for i = 1:num_steps
    % Calculate thrust (opposing gravity)
    thrust = mass_box * g + mass_drone * g;
    
    % Update position
    position_drone = [x_traj(i); y_traj(i)];

    % Calculate velocity
    if i > 1
        velocity_drone = sqrt((x_traj(i) - x_traj(i-1))^2 + (y_traj(i) - y_traj(i-1))^2) / dt;
    else
        velocity_drone = 0;
    end
    
    % Plot the animation
    plot(position_drone(1), position_drone(2), 'bo', 'MarkerSize', 10);
    hold on;
    plot(start_pos_box(1), start_pos_box(2), 'ro', 'MarkerSize', 10);
    hold on;
    plot([position_drone(1), start_pos_box(1)], [position_drone(2), start_pos_box(2)], 'r--');
    xlim([-1, 6]);
    ylim([-1, 6]);
    xlabel('X position (m)');
    ylabel('Y position (m)');
    title(sprintf('Drone Approaching the AUV\nTime: %.2f s, Velocity: %.2f m/s', t(i)+5, velocity_drone));
    %title('Drone Approaching the AUV');
    grid on;
    pause(0.01); % Animation speed
    if i < num_steps
        cla; % Clear previous frame
    end
end
hold off;