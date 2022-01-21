function d = distance2curveKH(curvexyandu)

[xy, d] = distance2curve(curvexyandu(:,1:end-2)', curvexyandu(:,end-1:end)', 'spline');
