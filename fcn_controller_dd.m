function [v_cmd, w_cmd, idx] = fcn_controller_dd(state, traj)
% state = [x; y; th]
% Nearest point index (simple nearest; you can add lookahead later)
[~,idx] = min( (traj.x - state(1)).^2 + (traj.y - state(2)).^2 );

% Errors in robot frame
dx = traj.x(idx) - state(1);
dy = traj.y(idx) - state(2);
ey = -sin(state(3))*dx + cos(state(3))*dy;    % cross-track error
etheta = wrapToPi(traj.th(idx) - state(3));   % heading error

% Gains (tune as needed)
k_y  = 1.5;
k_th = 2.0;

v_cmd = traj.v_ff(idx);
w_cmd = traj.w_ff(idx) + k_y*ey + k_th*etheta;

% Saturation (edit to your platform)
v_cmd = clip(v_cmd, -0.8, 0.8);
w_cmd = clip(w_cmd, -2.0, 2.0);
end

function y = clip(x, lo, hi)
y = min(max(x, lo), hi);
end

function ang = wrapToPi(a)
% Lightweight wrapToPi (avoid toolbox dependency)
ang = mod(a + pi, 2*pi) - pi;
end
