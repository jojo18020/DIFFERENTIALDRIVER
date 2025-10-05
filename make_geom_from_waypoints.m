function geom = make_geom_from_waypoints(waypoints)
% waypoints: Nx2 [x y]
xy = waypoints;
ds = sqrt(sum(diff(xy).^2,2));
S = [0; cumsum(ds)];        % cumulative arc-length at knots
xx = xy(:,1); yy = xy(:,2);
ppx = spline(S, xx);        % x(s)
ppy = spline(S, yy);        % y(s)
% Derivatives for curvature
ppx1 = fnder(ppx,1); ppy1 = fnder(ppy,1);
ppx2 = fnder(ppx,2); ppy2 = fnder(ppy,2);
geom.S  = S;             % knot s
geom.L  = S(end);        % total length
geom.ppx  = ppx; geom.ppy = ppy;
geom.ppx1 = ppx1; geom.ppy1 = ppy1;
geom.ppx2 = ppx2; geom.ppy2 = ppy2;
end

function [x,y,dxds,dyds,kappa] = eval_path(geom, s)
x    = ppval(geom.ppx,  s);
y    = ppval(geom.ppy,  s);
dxds = ppval(geom.ppx1, s);
dyds = ppval(geom.ppy1, s);
d2x  = ppval(geom.ppx2, s);
d2y  = ppval(geom.ppy2, s);
kappa = (dxds.*d2y - dyds.*d2x) ./ max((dxds.^2 + dyds.^2), 1e-9).^(3/2);
end
