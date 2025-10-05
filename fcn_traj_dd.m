function traj = fcn_traj_dd(geom, ts, v_des, dt)
% ts: waypoint times (size matches geom.S), v_des: scalar, dt: step
coeff_s = poly_traj_coeffs_vdes(geom.S, ts, v_des);
t = (ts(1):dt:ts(end))';
% Evaluate s(t)
s = zeros(size(t));
seg = 1;
for k = 1:numel(t)
    while seg < numel(ts) && t(k) > ts(seg+1), seg = seg+1; end
    seg = min(seg, numel(ts)-1);
    h = t(k) - ts(seg);
    a = coeff_s(:,seg);
    s(k)  = a(1) + a(2)*h + a(3)*h^2 + a(4)*h^3;
end
% Path->pose and feedforward velocities
[x,y,dxds,dyds,kappa] = eval_path(geom, s);
th   = unwrap(atan2(dyds, dxds));
v_ff = v_des * ones(size(t));          % desired linear speed
w_ff = kappa .* v_ff;                  % ω = κ v
traj.t=t; traj.s=s; traj.x=x; traj.y=y; traj.th=th; traj.v_ff=v_ff; traj.w_ff=w_ff;
end
