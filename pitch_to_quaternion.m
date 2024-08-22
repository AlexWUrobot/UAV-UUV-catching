function q = pitch_to_quaternion(pitch)


%pitch = -45;
% Convert pitch angle from degrees to radians
pitch = deg2rad(pitch);
% Define the quaternion for the given pitch angle
q = [cos(pitch/2), 0, sin(pitch/2), 0];
%q = quaternion([cos(pitch/2), 0, sin(pitch/2), 0]);
