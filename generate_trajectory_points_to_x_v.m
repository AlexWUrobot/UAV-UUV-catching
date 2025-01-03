function [x_d, v_d] = generate_trajectory_points_to_x_v(points, times)
    % Inputs:
    %   points: n x 6 matrix where each row contains [x, y, z, roll, pitch, yaw]
    %   times: 1 x n vector of timestamps for each point
    % Outputs:
    %   x_d: function handle for position trajectory [x(t); y(t); z(t)]
    %   v_d: function handle for velocity trajectory [vx(t); vy(t); vz(t)]

    %% Extract Position and Orientation Data
    positions = points(:, 1:3); % [x, y, z]
    rolls = points(:, 4);      % Roll angles
    pitches = points(:, 5);    % Pitch angles
    yaws = points(:, 6);       % Yaw angles

    %% Interpolate Position
    % Use cubic spline interpolation for smooth position trajectory
    x_spline = spline(times, positions(:, 1));
    y_spline = spline(times, positions(:, 2));
    z_spline = spline(times, positions(:, 3));

    % Position trajectory as a function of time
    x_d = @(t) [ppval(x_spline, t); ppval(y_spline, t); ppval(z_spline, t)];

    % Velocity trajectory as a function of time (derivative of position)
    x_dot_spline = fnder(x_spline);
    y_dot_spline = fnder(y_spline);
    z_dot_spline = fnder(z_spline);
    v_d = @(t) [ppval(x_dot_spline, t); ppval(y_dot_spline, t); ppval(z_dot_spline, t)];

    %% Interpolate Orientation (Yaw/Heading) to Generate b1d
    % Convert Roll, Pitch, Yaw to Rotation Matrices
    n = size(points, 1);
    rotations = zeros(3, 3, n);
    for i = 1:n
        rotations(:, :, i) = eul2rotm([yaws(i), pitches(i), rolls(i)], 'ZYX');
    end

    % Extract b1d (first column of rotation matrix) at each point
    b1d_points = squeeze(rotations(:, 1, :))';

    % Use SLERP for smooth interpolation of heading vectors
    b1d_interp = @(t) slerp_vectors(b1d_points, times, t);

    %% Interpolate Roll, Pitch, Yaw
    roll_spline = spline(times, rolls);
    pitch_spline = spline(times, pitches);
    yaw_spline = spline(times, yaws);

    roll_d = @(t) ppval(roll_spline, t);
    pitch_d = @(t) ppval(pitch_spline, t);
    yaw_d = @(t) ppval(yaw_spline, t);

    
    
    %% Interpolate Orientation (Yaw/Heading) to Generate b1d
    % Convert Roll, Pitch, Yaw to Rotation Matrices
    n = size(points, 1);
    rotations = zeros(3, 3, n);
    for i = 1:n
        rotations(:, :, i) = eul2rotm([yaws(i), pitches(i), rolls(i)], 'ZYX');
    end

    % Extract b1d (first column of rotation matrix) at each point
    b1d_points = squeeze(rotations(:, 1, :))';

    % Use SLERP for smooth interpolation of heading vectors
    b1d_interp = @(t) slerp_vectors(b1d_points, times, t);

    
    
    %% Display Results (Optional Visualization)
    t_sample = linspace(times(1), times(end), 100);
    positions_sample = arrayfun(@(t) x_d(t), t_sample, 'UniformOutput', false);
    positions_sample = cell2mat(positions_sample); % Convert to matrix for plotting

    velocities_sample = arrayfun(@(t) v_d(t), t_sample, 'UniformOutput', false);
    velocities_sample = cell2mat(velocities_sample); % Convert to matrix for plotting

    % Compute b1d trajectory for display
    b1d_sample = arrayfun(@(t) b1d_interp(t), t_sample, 'UniformOutput', false);
    b1d_sample = cell2mat(b1d_sample); % Convert to matrix for plotting

    % Compute roll, pitch, yaw for display
    roll_sample = arrayfun(roll_d, t_sample);
    pitch_sample = arrayfun(pitch_d, t_sample);
    yaw_sample = arrayfun(yaw_d, t_sample);
    
    % Plot Position Trajectory
    figure;
    subplot(4, 1, 1);
    plot(t_sample, positions_sample');
    title('Position Trajectory');
    legend('x', 'y', 'z');
    grid on;

    % Plot Velocity Trajectory
    subplot(4, 1, 2);
    plot(t_sample, velocities_sample');
    title('Velocity Trajectory');
    legend('vx', 'vy', 'vz');
    grid on;

    % Compute b1d trajectory for display
    b1d_sample = arrayfun(@(t) b1d_interp(t), t_sample, 'UniformOutput', false);
    b1d_sample = cell2mat(b1d_sample); % Convert to matrix for plotting

    subplot(4, 1, 3);
    plot(t_sample, b1d_sample');
    title('Heading Trajectory (b1d)');
    legend('b1d_x', 'b1d_y', 'b1d_z');
    grid on;
    
    % Plot Roll, Pitch, Yaw
    subplot(4, 1, 4);
    plot(t_sample, roll_sample, 'r', t_sample, pitch_sample, 'g', t_sample, yaw_sample, 'b');
    title('Orientation (Roll, Pitch, Yaw)');
    legend('Roll', 'Pitch', 'Yaw');
    grid on;
%     %% Display Results (Optional Visualization)
%     t_sample = linspace(times(1), times(end), 100);
%     positions_sample = arrayfun(x_d, t_sample, 'UniformOutput', false);
%     velocities_sample = arrayfun(v_d, t_sample, 'UniformOutput', false);
%     b1d_sample = arrayfun(b1d_interp, t_sample, 'UniformOutput', false);
% 
%     % Convert cell arrays to matrices for plotting
%     positions_sample = cell2mat(positions_sample');
%     velocities_sample = cell2mat(velocities_sample');
%     b1d_sample = cell2mat(b1d_sample');
% 
%     figure;
%     subplot(3, 1, 1);
%     plot(t_sample, positions_sample);
%     title('Position Trajectory');
%     legend('x', 'y', 'z');
%     grid on;
% 
%     subplot(3, 1, 2);
%     plot(t_sample, velocities_sample);
%     title('Velocity Trajectory');
%     legend('vx', 'vy', 'vz');
%     grid on;
% 
%     subplot(3, 1, 3);
%     plot(t_sample, b1d_sample);
%     title('Heading Trajectory (b1d)');
%     legend('b1d_x', 'b1d_y', 'b1d_z');
%     grid on;
end

function b1d_t = slerp_vectors(vectors, times, t)
    % Perform SLERP interpolation on heading vectors
    % Inputs:
    %   vectors: n x 3 matrix of unit vectors
    %   times: 1 x n vector of timestamps
    %   t: scalar or vector of query times
    % Outputs:
    %   b1d_t: interpolated unit vector(s) at time t

    n = size(vectors, 1);
    t = min(max(t, times(1)), times(end)); % Clamp t within time bounds

    % Find interval for each t
    indices = arrayfun(@(ti) find(ti >= times, 1, 'last'), t);
    indices = min(indices, n - 1); % Avoid out-of-bound access

    b1d_t = zeros(3, length(t));
    for i = 1:length(t)
        t1 = times(indices(i));
        t2 = times(indices(i) + 1);
        v1 = vectors(indices(i), :);
        v2 = vectors(indices(i) + 1, :);

        % Compute SLERP interpolation
        alpha = (t(i) - t1) / (t2 - t1);
        omega = acos(dot(v1, v2));
        if omega > 1e-6
            b1d_t(:, i) = (sin((1 - alpha) * omega) * v1 + sin(alpha * omega) * v2) / sin(omega);
        else
            b1d_t(:, i) = v1; % Handle collinear vectors
        end
    end
end
