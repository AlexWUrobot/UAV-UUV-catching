clc;
clear all; 
close all;
rng(100,"twister")

mapData = load("C:\Users\Reed\Documents\MATLAB\Examples\R2024a\uav\MotionPlanningWithRRTForAFixedWingUAVExample\uavMapCityBlock.mat");
omap = mapData.omap;

% Consider unknown spaces to be unoccupied
omap.FreeThreshold = omap.OccupiedThreshold;
show(omap)

%startPose = [12 22 25 0.7 0.2 0 0.1];  % [x y z qw qx qy qz]
%goalPose = [150 180 35 0.3 0 0.1 0.6];



startPose = [40, 20, 100, pitch_to_quaternion(135)];  % [x y z qw qx qy qz]
waypt1 = [80 20 35, pitch_to_quaternion(130)];
waypt2 = [90 20 35, pitch_to_quaternion(125)];
goalPose = [100 20 100, pitch_to_quaternion(90)];

% Plot the start and goal poses
hold on
scatter3(startPose(1),startPose(2),startPose(3),100,".r")
scatter3(goalPose(1),goalPose(2),goalPose(3),100,".g")
view([-31 63])
legend("","Start Position","Goal Position")
hold off

%% RRT
% ss = stateSpaceSE3([-20 220;
%                     -20 220;
%                     -10 100;
%                     inf inf;
%                     inf inf;
%                     inf inf;
%                     inf inf]);
% 
% sv = validatorOccupancyMap3D(ss);
% sv.Map = omap;
% sv.ValidationDistance = 0.1;
% 
% 
% planner = plannerRRTStar(ss,sv);
% planner.MaxConnectionDistance = 50;
% planner.GoalBias = 0.8;
% planner.MaxIterations = 1000;
% planner.ContinueAfterGoalReached = true;
% planner.MaxNumTreeNodes = 10000;
% 
% 
% [pthObj,solnInfo] = plan(planner,startPose,goalPose);

%% Visualize Path

% if (~solnInfo.IsPathFound)
%     disp("No Path Found by the RRT, terminating example")
%     return
% end

% Plot map, start pose, and goal pose
show(omap)
hold on
scatter3(startPose(1),startPose(2),startPose(3),100,".r")
scatter3(goalPose(1),goalPose(2),goalPose(3),100,".g")

% Plot path computed by path planner
% plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"-g")
% hold on



%%
view([-31 63])
legend("","Start Position","Goal Position","Planned Path")
%hold off



%%
waypoints = [startPose; waypt1; waypt2; goalPose]; 
nWayPoints = height(waypoints);

% waypoints = pthObj.States; 
% nWayPoints = pthObj.NumStates;

% Calculate the distance between waypoints
distance = zeros(1,nWayPoints);
for i = 2:nWayPoints
    distance(i) = norm(waypoints(i,1:3) - waypoints(i-1,1:3));
end

% Assume a UAV speed of 3 m/s and calculate time taken to reach each waypoint
% UAVspeed = 3;
% timepoints = cumsum(distance/UAVspeed);

UAVspeed = [3, 3, 5, 3];   % velocity should control real UAV attitude, "pitch_to_quaternion" cannot !
%UAVspeed = 2;  
timepoints = cumsum(distance./UAVspeed); % time stamp, when to arrive the point
nSamples = 100;  % output number of points

total_time_of_motion = timepoints(end);
sample_time_interval = total_time_of_motion/nSamples;
timeline = ((1:1:nSamples)-1)*sample_time_interval; % start from 0 sec


% Compute states along the trajectory
initialStates = minsnappolytraj(waypoints',timepoints,nSamples,MinSegmentTime=1,MaxSegmentTime=75,TimeAllocation=true,TimeWeight=100)';


%%
% Plot map, start pose, and goal pose
% show(omap)
% hold on
% scatter3(startPose(1),startPose(2),startPose(3),30,".r")
% scatter3(goalPose(1),goalPose(2),goalPose(3),30,".g")

% Plot the waypoints
% plot3(pthObj.States(:,1),pthObj.States(:,2),pthObj.States(:,3),"-g")
% plot3(waypoints(:,1),waypoints(:,2),waypoints(:,3),"-g")


% Plot the minimum snap trajectory
plot3(initialStates(:,1),initialStates(:,2),initialStates(:,3),"-y")




%%



view([-31 63])
legend("","Start Position","Goal Position","Planned Path","Initial Trajectory")
hold on

x = initialStates(:,1)';
y = initialStates(:,2)';
z = initialStates(:,3)';
qw = initialStates(:,4)';
qx = initialStates(:,5)';
qy = initialStates(:,6)';
qz = initialStates(:,7)';

% Compute the direction vector from the quaternion
directions = [2 * (qx .* qz + qw .* qy);
              2 * (qy .* qz - qw .* qx);
              1 - 2 * (qx.^2 + qy.^2)];


% Scale the direction vector (adjust the scaling factor as needed)
scale_factor = 0.1;
directions_scaled = scale_factor * directions;

% Add an arrow from the point (x, y, z) in the direction of the quaternion
%quiver3(x, y, z, directions_scaled(1, :), directions_scaled(2, :), directions_scaled(3, :), 'r', 'LineWidth', 2);
gap = 10;
quiver3(x(1:gap:end), y(1:gap:end), z(1:gap:end), directions_scaled(1, 1:gap:end), directions_scaled(2, 1:gap:end), directions_scaled(3, 1:gap:end), 'r', 'LineWidth', 2);



hold on;
for i=1:gap:length(x)
    direction_x = directions_scaled(1, i);
    direction_y = directions_scaled(2, i);
    direction_z = directions_scaled(3, i);

    direction_xyz = [direction_x, direction_y, direction_z];
    K = wind_trapezoidal(x(i),y(i),z(i),direction_xyz);
end


title("UAV catching UUV motion planning")

figure()
subplot(3,1,1)
plot(timeline,x)
title("UAV poisiton")
ylabel("x(m)")
subplot(3,1,2)
plot(timeline,round(y))
ylabel("y(m)")
subplot(3,1,3)
plot(timeline,z)
ylabel("z(m)")
xlabel("time(sec)")


v_x = diff(x)/sample_time_interval;
v_y = diff(round(y))/sample_time_interval;
v_z = diff(z)/sample_time_interval;
%timeline = ((1:1:nSamples-1)-1)*sample_time_interval; % start from 0 sec
timeline = ((1:1:nSamples-1))*sample_time_interval-sample_time_interval/2; % start from 0 sec, 
% -sample_time_interval/2, because we want to get the average speed between two sample points

figure()
subplot(3,1,1)
plot(timeline,v_x)
title("UAV velocity")
ylabel("x(m/s)")
subplot(3,1,2)
plot(timeline,v_y)
ylabel("y(m/s)")
subplot(3,1,3)
plot(timeline,v_z)
ylabel("z(m/s)")
xlabel("time(sec)")

