function y = follow_trajectory(u)
% Implementation of the pure pursuit algorithm. 
% See Craig Coulter 1990 "Implementation of the pure pursuit path tracking
% algorithm"
%
% Arguments
% u(1)  -  The simulation time
% u(2)  -  The heading angle theta
% u(3)  -  The robot position x-coordinate
% u(4)  -  The robot position y-coordinate
% u(5)  -  The look-ahead distance l
% u(6)  -  The default linear velocity. Except for at start and end, this
%          will be the linear velocity
% u(7:end)  -  Waypoints defining the path
%
% Returns
% y(1)  -  The angular velocity
% y(2)  -  The linear velocity


% Unit tests
if nargin == 0
    find_goal_point();
    return
end

% Simulation time
tsim = u(1);
% Current state
th = u(2);
x = u(3);
y = u(4);

% Parameters
l = u(5); % The look-ahead distance
vd = u(6); % The defailt speed. 

% Waypoints. The path.
waypoints = reshape(u(7:end), 2, []);
nwp = size(waypoints,2);

persistent currentWP
if isempty(currentWP)
    currentWP = 1;
end
if tsim < 0.1 % Avoids having to reset the function before new simulation
    currentWP = 1;
end
    
% TODO: Transform points to robot-centric frame
%waypoints_b = 
Rbs = [cos(th), sin(th)
    -sin(th), cos(th)];
vwp_s = waypoints - [x;y];
waypoints_b = Rbs *vwp_s;


if currentWP == nwp
    % Current waypoint is goal. Go to point
    display('Last waypoint. Go to point')
    y = go_to_point(cwp, vd);
else
    % Pure pursuit
    cwp2 = waypoints_b(:, currentWP+1);
    [pg, beta] = find_goal_point(cwp, cwp2, l);
    
    % TODO: Steer according to the pure pursuit algorithm. Change
    % waypoint if needed.
    if isnan(beta)
        y = go_to_point(cwp, vd);
    else
    
        if beta > 1
        
        else
            
        end
    end
    
    
end
end % function

function y = go_to_point(cwp, vd)
    
    % Arguments
    % cwp  -  A waypoint to go to, in the robot-centric reference frame
    % vd   -  Upper limit on linear velocity
    % 
    % Returns
    % y(1)  -  The angular velocity
    % y(2)  -  The linear velocity
 
    % TODO: Implement function to steer the robot towards the point cwp
    
    dx = cwp(1)
    dy = cwp(2);
    dist = sqrt(dx^2 + dy^2);
    errang = atan2(dy, dx);
    
    v = min(vd, dist);
    w = errang*dist*4;
        
    y = [w;v];
end % function

function [pg, beta] = find_goal_point(p1, p2, l)
    % Find goal point: Point on line segment p1 and p2 at look-ahead
    % distance. 
    %
    % Arguments
    %   p1  - Current waypoint (2x1)
    %   p2  - Next waypoint (2x1)
    %   l1  - the look-ahead distance.
    %
    % Returns
    %   pg  - the goal point (2x1)
    %   beta - scalar. pg = p1 + beta*(p2-p1). 0 <= beta <= 1, if the 
    %          goal point is on the line segment. If the goal point is 
    %          beyond the line segment, beta > 1, and if the line is
    %          beyond the look-ahead distance, then beta = inf.
    
    % TODO: Implement the function
    
    % Unit tests
    if nargin == 0
        display(' ')
        display(' ')
        display('Unit tests for function find_goal_point')
        tol = 1e-8;
        [p, b] = find_goal_point([0;0], [2;0], 1)
        if (abs(b-0.5) > tol)
            display('   FAILED test 1.1')
        else
            display('   Passed test 1.1')
        end
        if (norm(p-[1;0]) > tol)
            display('   FAILED test 1.2')
        else
            display('   Passed test 1.2')
        end
        if (abs(norm(p)-1) > tol)
            display('   FAILED test 1.3')
        else
            display('   Passed test 1.3')
        end
        
        [p, b] = find_goal_point([0;1], [2;1], sqrt(2))
        if (abs(b-0.5) > tol)
            display('   FAILED test 2.1')
        else
            display('   Passed test 2.1')
        end
        if (norm(p-[1;1]) > tol)
            display('   FAILED test 2.2')
        else
            display('   Passed test 2.2')
        end
        if (abs(norm(p)-sqrt(2)) > tol)
            display('   FAILED test 2.3')
        else
            display('   Passed test 2.3')
        end
        
        [p, b] = find_goal_point([0;1], [2;1], 0.8)
        if (~isnan(b))
            display('   FAILED test 3')
        else
            display('   Passed test 3')
        end
        return
    end
  
    v = p2-p1;
    a = v'*v;
    b = 2*v'*p1;
    c = p1'*p1 - l^2;
    
    sqrtpart = 1/(2*a)*sqrt(b^2 -4*a*c)
    if isreal(sqrtpart)
        beta = -b/(2*a) + sqrtpart;
        pg = p1 + beta*v;
    else
        beta = nan;
        pg = p1; 
    end
    
    
end
    