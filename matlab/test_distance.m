% Test distance to spline curve function
a=3;
SplinePoints=[-a, -a
    -0.904*a, -1.019*a,
    -0.8*a, -a
    0, -0.9*a
    a, -a
    0.9*a, 0
    a, a
    0, 0.9*a
    -a, a
    -0.9*a, 0
    -a, -0.8*a
    -1.019*a, -0.904*a];
npts = length(SplinePoints(:,1));
%pp = csape(0:(npts-1), SplinePoints', 'periodic');
pp = cscvn(SplinePoints');
splpnts = fnplt(pp);
figure(1)
clf
plot(splpnts(1,:), splpnts(2,:), 'r', 'LineWidth', 2);
hold on

[xxyy, dd, tt] = distance2curve(SplinePoints, splpnts', 'spline');
plot(xxyy(:,1), xxyy(:,2), 'mo', 'LineWidth', 1);

figure(2)
clf
plot(dd)

mean(dd)


