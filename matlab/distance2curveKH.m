function d = distance2curveKH(curvexyandu)

sensorpos = curvexyandu(:,1:2)';
curvexy = curvexyandu(:,3:end)';


[xy, d] = distance2curve(curvexy, sensorpos, 'spline');
