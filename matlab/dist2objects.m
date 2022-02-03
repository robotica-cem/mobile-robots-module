function d = dist2objects(u)
%
% Calcualates the distance to objects in the x-y plane from a central point
% representing a lidar sensor.
%
% Arguments
% ---------
% u  : [theta, x, y, p1x, p1y, p2x, p2y, ...]
%   theta is direction of sensor, x and y its position.
%   The rest of the elements are split into groups of four elements
%   where each group defines a line segement from (p1x, p1y) to (p2x, p2y)
%   The lidar ray will bounce back when hitting this line segment
%
% Returns
% ---------
% d  : distance to nearest object in 180 sectors of angle 2 degrees
%      starting from straight ahead and going in positive angular 
%      direction

% Kjartan Halvorsen
% 2022-02-02




if nargin == 0
    run_tests();
    return
else
    
    sect = 2;
    a = (1:sect:360)'/180*pi; % angles of each sector
    d = ones(size(a))*realmax; % The distances
    
    % Unit vectors in the direction of the lidar rays
    v = [cos(a'); sin(a')];
    
    th = u(1);
    xy = u(2:3);
    corners = reshape(u(4:end), 2, []) - xy;
    [two, nc] = size(corners);
    
    % Transform corners so that the lidar is in the origin
    R = [cos(th), sin(th)
        -sin(th), cos(th)];
    corners = R*corners;
    
    % Find the angle to each corner
    angles = atan2(corners(2,:), corners(1,:)); % in (-pi, pi]

    % Convert to index into the distance vector d
    negatives = find(angles < 0);
    angles(negatives) = angles(negatives) + 2*pi; 
    angles = floor( angles/pi*180/sect ) + 1;
  
    
    for i=1:2:nc
        p1 = corners(1:2, i);
        p2 = corners(1:2, i+1);
        ang1 = angles(i);
        ang2 = angles(i+1);
        
        % Determine the angles (indices) to check for intersection 
        % between ray and line segment.
        % Carful when the segment is in front
        % Handle 
        if ang1 < ang2
            rays = ang1:ang2;
        else
            rays = ang2:ang1;
        end
        if (rays(end)-rays(1))*sect > 180
            rays = cat(2, 1:rays(1), rays(end):180);
        end
        
        
        % Find intersection of lidar ray with line segment
        p1minp2 = p1-p2;
        for ind = rays
            ts = [v(:,ind), p1minp2]\p1;
            if ts(2) < 0 | ts(2) > 1
                % Intersection not on line segment. Ignore
                continue
            end
            dst = ts(1);
            if d(ind) > dst
                % The segment is closer
                d(ind) = dst;
            end
        end
    end
    
end

function run_tests()
% Unit tests
tol = 1e-2; % High tolerance due to discretization of space 

% 1)
x = 0;
y = 0;
th = pi/2;

corners = [1;-1;1;1];
d = dist2objects(cat(1, th, x, y, corners));

[md, minind] = min(d);
assert(abs(md - 1) < tol, 'Minimum distance incorrect')
display('Test 1.a OK')

minangle = minind*360/180;
assert( abs(minangle - 270) < 3 , 'Angle to minimum distance incorrect')
display('Test 1.b OK')


% 2)
tol = 1e-1; % High tolerance due to discretization of space 
x = 1;
y = 0.9;
corners = [3
    1
    5
    2
    5
    2
    5
    1
    5
    1
    3
    1];

for th = (1:10)*pi/5
    
    d = dist2objects(cat(1, th, x, y, corners));
    [md, minind] = min(d);
    minangle = minind/180*2*pi;
    expangle = -th;
    if expangle < 0
        expangle = expangle + 2*pi;
    end
    assert(abs(md - 2) < tol, 'Minimum distance incorrect')
    assert( abs(minangle - expangle) < 3 , 'Angle to minimum distance incorrect')
end

display('Test 2 OK')

% 3) Object straight ahead
tol = 1e-3; % High tolerance due to discretization of space 
x = 1;
y = 1;
th = 0;

corners = [3;-1;3;3];
d = dist2objects(cat(1, th, x, y, corners));

[md, minind] = min(d);
assert(abs(md - 2) < tol, 'Minimum distance incorrect')
display('Test 3.a OK')

minangle = minind*360/180;
assert( abs(minangle - 0) < 3 , 'Angle to minimum distance incorrect')
display('Test 3.b OK')


% 4) Object almost parallel to ray
tol = 1e-3; % High tolerance due to discretization of space 
x = 1;
y = 1;
th = pi;

corners = [3;0.99;5;1.001];
d = dist2objects(cat(1, th, x, y, corners));

[md, minind] = min(d);
assert(md == realmax, 'Minimum distance incorrect')
display('Test 4 OK')

            