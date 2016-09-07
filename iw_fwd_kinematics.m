function [ iw_htm ] = iw_fwd_kinematics( l , l_back , r_i )
%iw_fwd_kinematics Using Ian Walker's 2006 on Kinematics of Continuum Robots to
%                  present a clear pathway from input tendon lengths to output
%                  coordinates.
%
% Inputs:
%    l      : l is a vector with 3 dimensons 
%             (I expect a row, but I think it should work either way)
%             l(1) = l1, l(2) = l2, etc.
%    l_back : The length of the flexible backbone
%    r_i    : Distance from the central backbone to any of the tendons
%
% Assumes: 
%   - All tendons are spaced an equal distance (r_i) from the central
%     backbone.
%   - Backbone length (l_back) does not change.
%   - Backbone length (l_back) is not zero.
%   - This is a three tendon robot where the three tendon lengths are
%     stored in vector l.
%

%% CONSTANTS

%The paper treats each trunk as being split into n segments, while having
%only 1 curvature overall. So we will assume that each trunk = our segment.
%Therefore each trunk is a 1 segment long affair.

n = 1;
d = r_i;

% Note that the positions of cables 1, 2 and 3 on the circumference
% of the trunk with respect to the x axis are 90�, 210�
% and -30�

%% ACTUATOR SPACE TO CONFIGURATION SPACE

% We have our actuator variables (l1, l2, l3).
% Now, let's move to the configuration space (s,k, phi)

% S is defined by equation (19) [ and the limit of equation (19) in rare
% cases]

if (l(1) == l(2)) && (l(1) == l(3)) && (l(2)==l(3))
    %When the robot is straight, the HTM can be quickly calculated.
    s = sum(l)/3;
    
    A = [ eye(3) [0;0;l_back] ; zeros(1,3) 1 ];
    
else
    s = (n*d*sum(l))/sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3));
    s = s * asin( sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3))/(3*n*d));

    % Kappa and phi are defined by equations (15) and (16)
    % WARNING: phi turns into NaN when l1=l2=l3. (it becomes atan( 0 / 0 ), i
    % think. So, DO NOT INPUT STRAIGHT CONFIGURATIONS TO THIS FUNCTION.

    kappa = 2 * sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3)) / ( d* sum(l) );

    phi   = atan( (sqrt(3)/3) * ( l(3) + l(2) - 2*l(1))/(l(2)-l(3)) );

    if isnan( phi )
        disp('Somethings wrong with phi!!!')
        % I don't believe this model was intended to help in situations where
        % the robot is straight.
        disp('Setting phi to 0?')
        phi = 0;
    end

    %% HTM

    % Equation 8 in the paper directly creates an htm for the system using
    % simple expressions for each entry.

    %Included in this matrix are the orientation steps which make the robot
    %point in the "z" direction. No additional rotations are needed.

    A = zeros(4);
    A(4,4) = 1;

    A(1,1) = (cos(phi))^2 * ( cos(kappa*s) - 1) + 1;

    A(1,2) = sin(phi)*cos(phi)*( cos(kappa*s) - 1);

    A(1,3) = -cos(phi)*sin(kappa*s);

    A(1,4) = cos(phi)*( cos(kappa*s) - 1)/kappa;

    A(2,1) = sin(phi)*cos(phi)*( cos(kappa*s) - 1);

    A(2,2) = (cos(phi))^2 * ( 1 - cos(kappa*s) ) + cos( kappa * s );

    A(2,3) = -sin(phi)*sin(kappa*s);

    A(2,4) = sin(phi)*( cos(kappa*s) - 1)/kappa;

    A(3,1) = cos(phi)*sin(kappa*s);

    A(3,2) = sin(phi)*sin(kappa*s);

    A(3,3) = cos(kappa*s);

    A(3,4) = sin(kappa*s)/kappa;

    end

%Save result
iw_htm = A;
end
