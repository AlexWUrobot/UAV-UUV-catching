
clc;
clear;
close all;

% Add CPLEX to MATLAB Path
addpath(genpath('C:\Users\Reed\Downloads\YALMIP-master'));
addpath(genpath('/path/to/cplex/matlab'));


% Example with decision variables and constraints
n_basis = 11;
m_segments = 5;

% Define decision variables
c = sdpvar(3 * n_basis * m_segments, 1); % Coefficients for trajectory

% Example index range for x, y, z dimensions
dim = 1; % For x-dimension
idx = (dim-1) * n_basis + 1 : dim * n_basis;

% Add constraints (dummy example)
Constraints = [c(idx) >= 0]; % Dummy constraint
Objective = sum(c(idx));    % Dummy objective

% Solve the problem
options = sdpsettings('solver', 'cplex', 'verbose', 1);
optimize(Constraints, Objective, options);

% Extract solution
c_numeric = value(c);       % Full numerical solution
c_idx_numeric = c_numeric(idx); % Subset corresponding to idx
disp('Numeric values of c(idx):');
disp(c_idx_numeric);