function d = distance2curveGlobal(u)

global SplinePoints

size(SplinePoints)
if length(SplinePoints) > 1
 d = SplinePoints(1,1);
else
  d=0;
end
    
%[xy, d] = distance2curve(SplinePoints, u', 'spline');
