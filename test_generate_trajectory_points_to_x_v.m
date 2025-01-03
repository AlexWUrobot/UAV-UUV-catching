
% Define sample points and timestamps
points = [
    0.0, 0.0, 0.0, 0, 0, 0;
    1.0, 0.5, 1.0, 10, 5, 45;
    2.0, 1.0, 2.0, 20, 10, 90;
    3.0, 1.5, 1.5, 15, -5, 135;
    4.0, 2.0, 1.0, 0, -10, 180
];

times = [0.0, 1.0, 2.0, 3.0, 4.0];

% Generate trajectories
[x_d, v_d] = generate_trajectory_points_to_x_v(points, times);

% Example usage
t = linspace(0, 4, 100); % Sample time points
positions = arrayfun(x_d, t, 'UniformOutput', false);
velocities = arrayfun(v_d, t, 'UniformOutput', false);

% Convert cell arrays to matrices for plotting
positions = cell2mat(positions');
velocities = cell2mat(velocities');