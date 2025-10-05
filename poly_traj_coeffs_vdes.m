function coeffs = poly_traj_coeffs_vdes(Ss, ts, v_des)
% Ss  : [S0 S1 ... SN] cumulative arc-length at waypoints
% ts  : [t0 t1 ... tN] times at waypoints
% v_des: scalar desired linear speed at every knot (use sign for direction)
%
% Returns coeffs: 4 x N matrix, each column [a0;a1;a2;a3] for a cubic on [ti, ti+1]

N = numel(ts)-1;
coeffs = zeros(4, N);
for i = 1:N
    t0 = ts(i);  tf = ts(i+1);  h = tf - t0;
    S0 = Ss(i);  Sf = Ss(i+1);
    % Basis with tau = t - t0
    % s(t) = a0 + a1*tau + a2*tau^2 + a3*tau^3
    % Constraints:
    % s(0)=S0
    % s(h)=Sf
    % s'(0)=v_des
    % s'(h)=v_des
    A = [ 1,   0,   0,     0;
          1,   h, h^2,   h^3;
          0,   1,   0,     0;
          0,   1, 2*h, 3*h^2 ];
    b = [S0; Sf; v_des; v_des];
    coeffs(:,i) = A\b;
end
end
