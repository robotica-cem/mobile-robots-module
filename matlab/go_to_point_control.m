function wv = go_to_point_control(u)
% Transform world coordinate to robot-centric coordinate
%
% Inputs
% -------
% u(1) : The heading of the robot in rad
% u(2) : The x-position of the robot in m
% u(3) : The y-position of the robot in m
% u(4) : The desired x-position
% u(5) : The desired y-position

%
% Outputs
% wv(1) : The angular velocity
% wv(2) : The linear velocity


vlim = 0.4; % Maximum allowed linear vel
Kv = 1.0; % Gain for setting linear velocity (m/s) / m

th = u(1);
xy_s = u(2:3);
p_s = u(4:5);

v2_s = p_s - xy_s; % Vector to desired position

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Your code here
%
% v =
% w = 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

wv = [w;v];

end
