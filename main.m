% MAIN_diffdrive.m — Differential-drive trajectory + animation
% Only inputs: waypoints and total time T

clear; clc; close all

%% ---------------- User Inputs ----------------
% Waypoints (edit for your assignment)
waypoints = [0 0;
             1 0.2;
             2 0.8;
             3 0.8;
             4 0.5;
             6 0.9;
             7 0.15;
             8 0.9];

T = 18;        % total travel time [s]
dt = 0.01;     % timestep [s]

% Robot geometry (for wheel-speed plots)
wheel_radius = 0.05;  % m
half_track   = 0.15;  % m

% Command limits
v_max = 0.8;  w_max = 3.0;

%% --------------- Build Geometry & Time Allocation ---------------
geom = make_geom_from_waypoints(waypoints);

% Segment lengths and total path length
segment_lengths = diff(geom.S);
total_length = geom.L;

% Distribute total time proportionally to segment lengths
ts = [0; cumsum(segment_lengths / total_length * T)];
v_des = total_length / T;   % nominal desired speed (constant along path)

fprintf('\n--- Trajectory Setup ---\n');
fprintf('Number of waypoints: %d\n', size(waypoints,1));
fprintf('Total path length: %.3f m\n', total_length);
fprintf('Nominal v_des: %.3f m/s\n', v_des);
fprintf('Total time: %.2f s\n\n', T);

% Build trajectory
traj = fcn_traj_dd(geom, ts, v_des, dt);
[~,~,~,~,kappa] = eval_path(geom, traj.s);
traj.kappa = kappa;

%% ------------------- Initial Conditions --------------------
th0 = atan2(waypoints(2,2)-waypoints(1,2), waypoints(2,1)-waypoints(1,1));
X0  = [waypoints(1,1); waypoints(1,2); th0];

%% ---------------------- Simulate ---------------------------
[t_ode, X_ode] = ode45(@(t,X) dyn_diffdrive(t, X, [], traj), ...
                       [traj.t(1) traj.t(end)], X0);

N = numel(t_ode);
v_cmd  = zeros(N,1);
w_cmd  = zeros(N,1);
ey_arr = zeros(N,1);
e_th   = zeros(N,1);

for k = 1:N
    [v_cmd(k), w_cmd(k), idx] = fcn_controller_dd(X_ode(k,:).', traj);
    dx = traj.x(idx) - X_ode(k,1);
    dy = traj.y(idx) - X_ode(k,2);
    ey_arr(k) = -sin(X_ode(k,3))*dx + cos(X_ode(k,3))*dy;
    e_th(k)   = wrapToPiLocal(traj.th(idx) - X_ode(k,3));
end

% Wheel speeds
wL = (v_cmd - half_track*w_cmd)/wheel_radius;
wR = (v_cmd + half_track*w_cmd)/wheel_radius;

%% ----------------------- Plots -----------------------------
figure('Name','Path & Trajectory','Color','w'); 
subplot(2,2,1);
plot(waypoints(:,1), waypoints(:,2), 'ko--', 'LineWidth',1.2); hold on
plot(traj.x, traj.y, 'b-', 'LineWidth',1.6);
plot(X_ode(:,1), X_ode(:,2), 'r-', 'LineWidth',1.2);
quiver(X_ode(1:10:end,1), X_ode(1:10:end,2), ...
       cos(X_ode(1:10:end,3))*0.15, sin(X_ode(1:10:end,3))*0.15, 0, 'r');
axis equal; grid on;
legend('waypoints','reference path','robot path','robot heading','Location','best');
title('2D Path'); xlabel('x [m]'); ylabel('y [m]');

subplot(2,2,2);
plot(traj.t, traj.v_ff, 'b','LineWidth',1.5); hold on
plot(t_ode, clipLocal(v_cmd,-v_max,v_max), 'r--','LineWidth',1.3);
grid on; xlabel('t [s]'); ylabel('v [m/s]');
legend('v_{ff}','v_{cmd}','Location','best'); title('Linear Speed');

subplot(2,2,3);
plot(traj.t, traj.w_ff, 'b','LineWidth',1.5); hold on
plot(t_ode, clipLocal(w_cmd,-w_max,w_max), 'r--','LineWidth',1.3);
grid on; xlabel('t [s]'); ylabel('\omega [rad/s]');
legend('\omega_{ff}','\omega_{cmd}','Location','best'); title('Angular Speed');

subplot(2,2,4);
th_ref = interp1(traj.t, traj.th, t_ode, 'linear', 'extrap');
plot(t_ode, th_ref, 'b','LineWidth',1.5); hold on
plot(t_ode, X_ode(:,3), 'r--','LineWidth',1.2);
grid on; xlabel('t [s]'); ylabel('\theta [rad]');
legend('\theta_{ref}','\theta','Location','best'); title('Heading vs Time');

% Errors, curvature, arc-length
figure('Name','Errors, Curvature, Arc-length','Color','w');
subplot(2,2,1); plot(t_ode, ey_arr, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('e_y [m]'); title('Cross-track Error');

subplot(2,2,2); plot(t_ode, e_th, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('e_\theta [rad]'); title('Heading Error');

subplot(2,2,3); plot(traj.t, traj.kappa, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('\kappa [1/m]'); title('Path Curvature');

subplot(2,2,4); plot(traj.t, traj.s, 'LineWidth',1.4); grid on
xlabel('t [s]'); ylabel('s [m]'); title('Arc-length vs Time');

% Wheel speeds
figure('Name','Wheel Angular Speeds','Color','w');
plot(t_ode, wL, 'LineWidth',1.4); hold on
plot(t_ode, wR, 'LineWidth',1.4);
yline(0,'k:'); grid on
xlabel('t [s]'); ylabel('\omega_{wheel} [rad/s]');
legend('\omega_L','\omega_R','Location','best');
title('Left/Right Wheel Speeds');

%% ---------------------- Animation ----------------------
figure('Name','Differential Drive Animation','Color','w');
axis equal; grid on; hold on;
xlabel('x [m]'); ylabel('y [m]');
title('Robot Path Animation');

plot(traj.x, traj.y, 'b-', 'LineWidth', 1.5);
plot(waypoints(:,1), waypoints(:,2), 'ko','MarkerFaceColor','k');

L = 0.3; W = 0.2;
robotPatch = patch([-L/2 L/2 L/2 -L/2], [-W/2 -W/2 W/2 W/2], [0.8 0.1 0.1], 'FaceAlpha', 0.8);
trail = animatedline('Color',[0.9 0 0],'LineWidth',1.5);

xlim([min(traj.x)-0.5, max(traj.x)+0.5]);
ylim([min(traj.y)-0.5, max(traj.y)+0.5]);

skip = max(1, floor(length(t_ode)/400));

for k = 1:skip:length(t_ode)
    x = X_ode(k,1);
    y = X_ode(k,2);
    th = X_ode(k,3);
    R = [cos(th) -sin(th); sin(th) cos(th)];
    body = R * [ -L/2 L/2 L/2 -L/2; -W/2 -W/2 W/2 W/2 ];
    set(robotPatch, 'XData', body(1,:)+x, 'YData', body(2,:)+y);
    addpoints(trail, x, y);
    drawnow limitrate nocallbacks
end

fprintf('\n--- Simulation Summary ---\n');
fprintf('Total path length: %.3f m\n', total_length);
fprintf('Total time: %.2f s\n', T);
fprintf('Nominal v_des: %.3f m/s\n', v_des);
fprintf('Simulated samples: %d\n', N);
fprintf('Mean |ey|: %.3f m, Max |ey|: %.3f m\n', mean(abs(ey_arr)), max(abs(ey_arr)));
fprintf('Mean |e_theta|: %.3f rad, Max |e_theta|: %.3f rad\n\n', mean(abs(e_th)), max(abs(e_th)));

%% ---------------------- Local Helpers ----------------------
function y = wrapToPiLocal(a)
    y = mod(a + pi, 2*pi) - pi;
end

function y = clipLocal(x, lo, hi)
    y = min(max(x, lo), hi);
end
