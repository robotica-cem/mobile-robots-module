function wangvel = unicycle_inverse_kinematics(u)
% Inverse kinematics for the unicycle model
%
% Inputs
% -------
% u(1) : The linear velocity in m/s
% u(2) : The angular velocity in rad/s
% u(3) : The distance between the wheels in m
% u(4) : The radius of the wheels in m
%
% Outputs
% wangvel(1) : The left wheel angular velocity in rad/s
% wangvel(2) : The right wheel angular velocity in rad/s


if nargin == 0
    run_unit_tests()
    return
end

v = u(1);
w = u(2);
d = u(3);
r = u(4);

wR = (v + (w*d/2))/r;
wL = (v - w*d/2)/r;


wangvel = [wL;wR];
% Your implementation here




end

function run_unit_tests()
% A number of tests to check the correctness of the implementation

% Forward velocity only should give same angular velocity of both
% wheels
v = 1;
w = 0;
d = 0.2;
r = 0.1;

u = [v
    w
    d
    r];

wLwR = unicycle_inverse_kinematics(u);
assert(abs(wLwR(1)-wLwR(2)) < 1e-10, 'The velocities should be equal')

% Angular velocity only should give same angular velocity but of
% opposite sign for the two wheels
v = 0;
w = 1;
d = 0.2;
r = 0.1;

u = [v
    w
    d
    r];

wLwR = unicycle_inverse_kinematics(u);
assert(wLwR(1) == -wLwR(2), 'The velocities should be equal in magnitude but opposite sign')


end
