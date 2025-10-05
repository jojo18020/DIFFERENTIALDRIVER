function [x,y,dxds,dyds,kappa] = eval_path(geom, s)
x    = ppval(geom.ppx,  s);
y    = ppval(geom.ppy,  s);
dxds = ppval(geom.ppx1, s);
dyds = ppval(geom.ppy1, s);
d2x  = ppval(geom.ppx2, s);
d2y  = ppval(geom.ppy2, s);

den  = max((dxds.^2 + dyds.^2), 1e-9).^(3/2);
kappa = (dxds.*d2y - dyds.*d2x) ./ den;   % curvature κ(s)
end
