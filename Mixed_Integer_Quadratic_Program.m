% MATLAB Code for MIQP-based Trajectory Optimization for Fig. 4
% Requires IBM CPLEX installed and configured with MATLAB.

clc;
clear;
close all;

% Add CPLEX to MATLAB Path
addpath(genpath('C:\Users\Reed\Downloads\YALMIP-master'));
addpath(genpath('/path/to/cplex/matlab'));

%% Parameters
n_basis = 11;        % Number of basis functions
m_segments = 5;      % Number of trajectory segments
cable_length = 0.34; % Cable length (meters)
g = 9.81;            % Gravitational acceleration (m/s^2)

% Time vector
t_total = 2.9;                 % Total time (seconds)
t_des = [0, 0.6, 1.2, 1.8, 2.4, 2.9]; % Time waypoints for segments

% Waypoints (x, y, z positions at t_des)
waypoints = [
    -1, 0, 1;   % Start position
     0, 1, 0.8; % Mid-point 1
     1, 0, 0.6; % Mid-point 2
     1, 0, 0.8; % Pre-release
     2, 0, 0.8; % End position
];

%% Define Basis Functions and Decision Variables
syms t;
P = sym('P', [n_basis, 1]); % Basis functions
for i = 1:n_basis
    P(i) = t^(i-1); % Polynomial basis functions (t^0, t^1, ..., t^(n_basis-1))
end

% Decision Variables
c = sdpvar(3 * n_basis * m_segments, 1); % Coefficients for trajectory (x, y, z)

% Binary Variables for Obstacle Avoidance
b = binvar(4 * m_segments, 1);

%% Cost Function (Minimize Snap - 6th Derivative)
J = 0;
for j = 1:m_segments
    for i = 6:n_basis % Minimize the 6th derivative coefficients
        J = J + c(3 * (j-1) * n_basis + i)^2;
    end
end

%% Constraints
constraints = [];

% Waypoint Constraints
for i = 1:length(t_des)
    t_i = t_des(i);
    for dim = 1:3 % x, y, z
        idx = (dim-1) * n_basis + 1:dim * n_basis; % Index for each dimension
        p_val = double(subs(P, t, t_i)); % Evaluate basis functions at time t_i
        
        constraints = [constraints, p_val * c(idx) == waypoints(i, dim)];
    end
end

% Continuity Constraints
for j = 1:m_segments-1
    t_i = t_des(j+1);
    for dim = 1:3 % x, y, z
        idx1 = (dim-1) * n_basis + 1:dim * n_basis + (j-1) * n_basis * 3;
        idx2 = idx1 + n_basis * 3;
        p_val = double(subs(P, t, t_i)); % Evaluate basis functions
        constraints = [constraints, p_val * c(idx1) == p_val * c(idx2)];
        
    end
end

% Obstacle Avoidance (if needed, add binary variables for collision avoidance)

%% Solve MIQP Using CPLEX
options = sdpsettings('solver', 'cplex', 'verbose', 1);
sol = optimize(constraints, J, options);

% Check Solution
if sol.problem == 0
    disp('Optimal solution found!');
else
    disp('Problem in optimization.');
    disp(sol.info);
end

% Extract Solution
c_opt = value(c);

%% Generate Trajectories
t_fine = linspace(0, t_total, 300);
x_traj = zeros(1, length(t_fine));
y_traj = zeros(1, length(t_fine));
z_traj = zeros(1, length(t_fine));

for i = 1:length(t_fine)
    t_i = t_fine(i);
    for dim = 1:3
        idx = (dim-1) * n_basis + 1:dim * n_basis;
        p_val = double(subs(P, t, t_i));
        if dim == 1, x_traj(i) = p_val * c_opt(idx); end
        if dim == 2, y_traj(i) = p_val * c_opt(idx); end
        if dim == 3, z_traj(i) = p_val * c_opt(idx); end
    end
end

%% Plot Trajectories
figure;
plot3(x_traj, y_traj, z_traj, 'b', 'LineWidth', 2); hold on;
scatter3(waypoints(:, 1), waypoints(:, 2), waypoints(:, 3), 100, 'r', 'filled'); % Waypoints
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Load Trajectory');
grid on;
view(3);
