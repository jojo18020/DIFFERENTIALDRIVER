function xdot = dyn_diffdrive(~, X, ~, traj)
% X = [x; y; th]
[v_cmd, w_cmd] = fcn_controller_dd(X, traj);
xdot = [ v_cmd*cos(X(3));
         v_cmd*sin(X(3));
         w_cmd ];
end
