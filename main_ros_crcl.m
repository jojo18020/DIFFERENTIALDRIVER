% MAIN_tb3_ros2_plain.m
% Minimal, bullet-proof ROS2 loop that uses *your* fcn_controller_dd as-is.
%
% Before running (MATLAB, once per session):
%   setenv('ROS_DOMAIN_ID','123');         % must match WSL
%
% In WSL:
%   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
%   export ROS_DOMAIN_ID=123
%   unset ROS_LOCALHOST_ONLY
%   source /opt/ros/humble/setup.bash
%   export TURTLEBOT3_MODEL=burger
%   ros2 launch turtlebot3_fake_node turtlebot3_fake_node.launch.py
%
% Needs on MATLAB path:
%   make_geom_from_waypoints.m, eval_path.m, poly_traj_coeffs_vdes.m
%   fcn_traj_dd.m, fcn_controller_dd.m

clear; clc; close all

%% ---------------- Inputs ----------------
% waypoints = [
%              0 0;
%              -0.1 0.3;
%              -0.2 0.4;
%              -0.5 0.5;
%             -0.8 0.4;
%             -0.9 0.3;
%             -1.0 0;
%             -0.9 -0.3;
%             -0.8 -0.4;
%              -0.5 -0.5;
%              -0.2 -0.4;
%              -0.1 -0.3;
%              0 0
%              -0.1 0.3];
waypoints = [
             0 0;
             0.3 -0.1;
             0.4 -0.2;
             0.5 -0.5;
             0.4 -0.8;
             0.3 -0.9;
             0 -1.0;
            -0.3 -0.9;
            -0.4 -0.8;
            -0.5 -0.5;
            -0.4 -0.2;
            -0.3 -0.1;
             0 0;
             ];

T  = 60;          % total path time [s]
dt = 0.02;        % control step [s] (50 Hz)

% TB3 geometry (for wheel-speed plot)
wheel_radius = 0.033;   % m
half_track   = 0.0805;  % m  (L/2)

% Hard command limits (no shaping/"help")
v_max = 0.22;    % m/s
w_max = 1.20;    % rad/s

%% ----------- Trajectory (identical strategy to your non-ROS) ----------
geom = make_geom_from_waypoints(waypoints);
segL = diff(geom.S);   L = geom.L;
ts   = [0; cumsum(segL/L * T)];
v_des = L / T;

traj = fcn_traj_dd(geom, ts, v_des, dt);
[~,~,~,~,kappa] = eval_path(geom, traj.s);
traj.kappa = kappa;

fprintf('\n[TB3 tracker]\nWaypoints=%d | L=%.3f m | T=%.2f s | v_des=%.3f m/s\n', ...
        size(waypoints,1), L, T, v_des);

%% ---------------- ROS 2 I/O ----------------
node    = ros2node("/tb3_tracker");
subOdom = ros2subscriber(node, "/odom", "nav_msgs/Odometry", ...
                         "Reliability","reliable", "History","keeplast","Depth",20);
pubCmd  = ros2publisher(node, "/cmd_vel", "geometry_msgs/Twist");
msgCmd  = ros2message(pubCmd);

%% ---------------- Initial state (blocking up to 3 s) -------------------
odom = [];
t0 = tic;
while isempty(odom) && toc(t0) < 3
    try, odom = receive(subOdom, 0.25); catch, end
end
if isempty(odom)
    warning('No /odom received yet. Starting from first segment heading.');
    th0 = atan2(waypoints(2,2)-waypoints(1,2), waypoints(2,1)-waypoints(1,1));
    X   = [waypoints(1,1); waypoints(1,2); th0];
else
    p = odom.pose.pose.position; q = odom.pose.pose.orientation;
    X = [p.x; p.y; quat2yaw([q.w q.x q.y q.z])];
end

%% ---------------- Control loop (publish every tick) --------------------
N = numel(traj.t);
r = ros2rate(node, 1/dt);

X_log  = nan(N,3);
v_cmd  = zeros(N,1);
w_cmd  = zeros(N,1);
ey_arr = zeros(N,1);
e_th   = zeros(N,1);

sat_v_count = 0;
sat_w_count = 0;

for k = 1:N
    % get latest odom (small non-zero timeout to actually fetch new data)
    od = [];
    try, od = receive(subOdom, 0.01); catch, end
    if ~isempty(od)
        p = od.pose.pose.position; q = od.pose.pose.orientation;
        X = [p.x; p.y; quat2yaw([q.w q.x q.y q.z])];
    end

    % your controller — NO edits, NO shaping
    [v, w, idx] = fcn_controller_dd(X, traj);

    % hard saturation only
    v = min(max(v, -v_max), v_max);
    w = min(max(w, -w_max), w_max);

    sat_v_count = sat_v_count + (abs(v) >= v_max*0.999);
    sat_w_count = sat_w_count + (abs(w) >= w_max*0.999);

    % publish
    msgCmd.linear.x  = v;
    msgCmd.angular.z = w;
    send(pubCmd, msgCmd);

    % log
    X_log(k,:) = X.';
    v_cmd(k)   = v;
    w_cmd(k)   = w;

    % same error metrics as your non-ROS script
    dx = traj.x(idx) - X(1);
    dy = traj.y(idx) - X(2);
    ey_arr(k) = -sin(X(3))*dx + cos(X(3))*dy;
    e_th(k)   = wrapToPiLocal(traj.th(idx) - X(3));

    waitfor(r);
end

% stop
msgCmd.linear.x = 0; msgCmd.angular.z = 0; send(pubCmd, msgCmd);

%% ---------------- Wheel speeds from commanded (v,w) --------------------
wL = (v_cmd - half_track*w_cmd)/wheel_radius;
wR = (v_cmd + half_track*w_cmd)/wheel_radius;

%% --------------------------- Plots ------------------------------------
figure('Name','Path & Trajectory','Color','w');
subplot(2,2,1);
plot(waypoints(:,1), waypoints(:,2), 'ko--','LineWidth',1.2); hold on
plot(traj.x, traj.y, 'b-', 'LineWidth',1.6);
valid = ~isnan(X_log(:,1));
plot(X_log(valid,1), X_log(valid,2), 'r-', 'LineWidth',1.2);
quiver(X_log(1:10:end,1), X_log(1:10:end,2), ...
       cos(X_log(1:10:end,3))*0.15, sin(X_log(1:10:end,3))*0.15, 0, 'r');
axis equal; grid on;
legend('waypoints','reference path','robot path','robot heading','Location','best');
title('2D Path'); xlabel('x [m]'); ylabel('y [m]');

subplot(2,2,2);
plot(traj.t, traj.v_ff, 'b','LineWidth',1.5); hold on
plot(traj.t, v_cmd, 'r--','LineWidth',1.3);
grid on; xlabel('t [s]'); ylabel('v [m/s]');
legend('v_{ff}','v_{cmd}','Location','best'); title('Linear Speed');

subplot(2,2,3);
plot(traj.t, traj.w_ff, 'b','LineWidth',1.5); hold on
plot(traj.t, w_cmd, 'r--','LineWidth',1.3);
grid on; xlabel('t [s]'); ylabel('\omega [rad/s]');
legend('\omega_{ff}','\omega_{cmd}','Location','best'); title('Angular Speed');

subplot(2,2,4);
th_ref = traj.th;
plot(traj.t, th_ref, 'b','LineWidth',1.5); hold on
plot(traj.t, X_log(:,3), 'r--','LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('\theta [rad]');
legend('\theta_{ref}','\theta','Location','best'); title('Heading vs Time');

figure('Name','Errors, Curvature, Arc-length','Color','w');
subplot(2,2,1); plot(traj.t, ey_arr, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('e_y [m]'); title('Cross-track Error');

subplot(2,2,2); plot(traj.t, e_th, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('e_\theta [rad]'); title('Heading Error');

subplot(2,2,3); plot(traj.t, traj.kappa, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('\kappa [1/m]'); title('Path Curvature');

subplot(2,2,4); plot(traj.t, traj.s, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('s [m]'); title('Arc-length vs Time');

figure('Name','Wheel Angular Speeds','Color','w');
plot(traj.t, wL, 'LineWidth',1.4); hold on
plot(traj.t, wR, 'LineWidth',1.4);
yline(0,'k:'); grid on
xlabel('t [s]'); ylabel('\omega_{wheel} [rad/s]');
legend('\omega_L','\omega_R','Location','best');
title('Left/Right Wheel Speeds');

%% ---------------- Animation from odom logs -----------------------------
figure('Name','TB3 Animation (Odom logs)','Color','w');
axis equal; grid on; hold on
xlabel('x [m]'); ylabel('y [m]');
title('Robot Path Animation');
plot(traj.x, traj.y, 'b-','LineWidth',1.5);
plot(waypoints(:,1), waypoints(:,2), 'ko','MarkerFaceColor','k');

Lr = 0.30;  Wr = 0.20;
robotPatch = patch([-Lr/2 Lr/2 Lr/2 -Lr/2], [-Wr/2 -Wr/2 Wr/2 Wr/2], [0.8 0.1 0.1], 'FaceAlpha', 0.8);
trail = animatedline('Color',[0.9 0 0],'LineWidth',1.5);

xlim([min(traj.x)-0.5, max(traj.x)+0.5]);
ylim([min(traj.y)-0.5, max(traj.y)+0.5]);

skip = max(1, floor(N/400));
for k = 1:skip:N
    if ~valid(k), continue; end
    x = X_log(k,1); y = X_log(k,2); th = X_log(k,3);
    Rm = [cos(th) -sin(th); sin(th) cos(th)];
    body = Rm * [ -Lr/2 Lr/2 Lr/2 -Lr/2; -Wr/2 -Wr/2 Wr/2 Wr/2 ];
    set(robotPatch, 'XData', body(1,:)+x, 'YData', body(2,:)+y);
    addpoints(trail, x, y);
    drawnow limitrate nocallbacks
end

%% ---------------- Diag summary ----------------------------------------
fprintf('\n[v_cmd] min/max: [%.3f, %.3f]  |  [w_cmd] min/max: [%.3f, %.3f]\n', ...
         min(v_cmd), max(v_cmd), min(w_cmd), max(w_cmd));
fprintf('Saturations: v: %d / %d  (%.1f%%)   w: %d / %d  (%.1f%%)\n', ...
        sat_v_count, N, 100*sat_v_count/N, sat_w_count, N, 100*sat_w_count/N);

%% ---------------- Helpers ---------------------------------------------
function yaw = quat2yaw(qwxyz)
    w=qwxyz(1); x=qwxyz(2); y=qwxyz(3); z=qwxyz(4);
    yaw = atan2(2*(w*z + x*y), 1 - 2*(y*y + z*z));
end
function y = wrapToPiLocal(a), y = mod(a + pi, 2*pi) - pi; end
