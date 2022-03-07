function pb = transform_point(u)
% Transform world coordinate to robot-centric coordinate
%
% Inputs
% -------
% u(1) : The direction of the robot in rad
% u(2) : The x-position of the robot in m
% u(3) : The y-position of the robot in m
% u(4) : The x-coordinate of the point to transform
% u(5) : The y-coordinate of the point to transform

%
% Outputs
% pb(1) : The x-coordinate of the point in the robot frame
% pb(2) : The y-coordinate of the point in the robot frame


if nargin == 0
    run_unit_tests()
    return
end


th = u(1);
xy = u(2:3);
ps = u(4:5);


% Your implementation here


end

function run_unit_tests()
% A number of tests to check the correctness of the implementation

% Rotation of 90 degrees of robot, robot in origin
th = pi/2;
xy = zeros(2,1);
ps = [1;0];

u = [th;xy;ps];
pb = transform_point(u);
pb_expected = [0;-1];
assert(norm(pb-pb_expected) < 1e-10, 'Transformed point not as expected')

% Rotation of -90 degrees of robot, robot at [1;1]
th = -pi/2;
xy = [1;1];
ps = [0;1];

u = [th;xy;ps];
pb = transform_point(u);
pb_expected = [0;-1];
assert(norm(pb-pb_expected) < 1e-10, 'Transformed point not as expected')
end
