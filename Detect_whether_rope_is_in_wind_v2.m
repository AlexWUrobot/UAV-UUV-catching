% Detect whether point in the wind trapezoidal
clc;
clear all;
close all;

points = [1, 0, 0; 3, 0, 7; 0, 0, 20; -1, -1, 5]; % Example points
directions_scaled = [-1; -0.4; 0];
x = 0; y = 0; z = 0;

hold on;

inside_count = points_inside_trapezoid(points, directions_scaled, x, y, z);
disp(['Number of points inside the trapezoid: ', num2str(inside_count)]);