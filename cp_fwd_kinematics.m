function [ res ] = cp_fwd_kinematics( varargin )
%cp_fwd_kinematics Using Ian Walker's 2006 paper on Kinematics of Continuum Robots to
%                  present a clear pathway from input tendon lengths to output
%                  coordinates.
%
% Possible Usages:
%    iw_htm = cp_fwd_kinematics( 'f1,f2 (HTM)', l , l_back , d )
%    iw_htm = cp_fwd_kinematics( 'f1,f2 (HTM)', l , l_back , d , n )
%    iw_htm = cp_fwd_kinematics( 'f1 (HTM)'   , s , kappa , phi )
%    [   s   ]
%    [ kappa ] = cp_fwd_kinematics('f2', l , l_back , d , n )
%    [  phi  ]
%   
%    Use the '(f1,f2 (HTM)' version with 4 arguments when you want to define the number of "segments"
%    (use )that make up your continuum arm.    
%
% Inputs:
%    l      : l is a vector with 3 dimensons 
%             (I expect a row, but I think it should work either way)
%             l(1) = l1, l(2) = l2, etc.
%    l_back : The length of the flexible backbone
%    d      : Distance from the central backbone to any of the tendons
%    n      : The number of "segments" on the manipulators backbone.
%
% Assumes: 
%   - All tendons are spaced an equal distance (d) from the central
%     backbone.
%   - Backbone length (l_back) does not change.
%   - Backbone length (l_back) is not zero.
%   - This is a three tendon robot where the three tendon lengths are
%     stored in vector l.
%
% Modified:
%   - 9/7/2016: + to accomodate 2 different styles of calling this function.
%   - 9/21/2016: Changed name and hoping to accomodate different parts of the fwd kinematics chain 
%
% Current Issues:
%   - Despite my directly copying relationships from the paper, it appears
%     that directions are flipped and that the tendons are actually mapped to
%     

%% Constants
correction_angle = pi;  %For some reason, a rotation of 180 degrees (pi) is needed
                        %after the full calculation is complete.

switch( varargin{1} )
    case 'f1,f2 (HTM)'
        %This indicates that we want to transform a point in the actuator space (l) to 
        %task/world space (x,orientation)
        %Diagram Explanation:
        %   l = (l1, l2, l3) -> (x,y,z)

        %% PROCESS INPUTS

        if( (nargin ~= 4) && ( nargin ~=5) )
            disp([ 'Improper number of arguments to the function. 4 or 5 acceptable, ' num2str(nargin) ' received.' ])
        end

        if( nargin ~= 5 )
            disp(['Error: Expected 5 arguments, received ' num2str(nargin) '.' ])
            %Exit program
            res = -1; return;
        end

        l      = varargin{2};
        l_back = varargin{3};
        d      = varargin{4};

        if( nargin == 5 )
            n = varargin{5};
        else
            n = 1;
        end

        if( sum( l <= 0 ) > 0 )
            disp([ 'Error: One of the input tendon lengths is negative!' ])
            %Exit program
            res = -1; return;
        end

        if( l_back <= 0 )
            disp( [ 'Error: Expect length of backbone to be nonnegative. Received ' num2str(l_back) '.' ])
            %Exit program
            res = -1; return;
        end

        if( n <= 0 )
            disp( [ 'n must be a nonnegative integer. Received ' num2str(n) '.' ] );
            %Exit Program early.
            res = -1; return;
        end

        if( d <= 0 )
            disp( [ 'd must be a positive number. Received ' num2str(d) '.' ] );
            %Exit program early.
            res = -1; return;
        end

        

        %% CONSTANTS

        %The paper treats each trunk as being split into n segments, while having
        %only 1 curvature overall. So we will assume that each trunk = our segment.

        % Note that the positions of cables 1, 2 and 3 on the circumference
        % of the trunk with respect to the x axis are 90°, 210°
        % and -30°

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
            s = (n*d*sum(l))/(sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3)));
            s = s * asin( sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3))/(3*n*d));

            % Kappa and phi are defined by equations (15) and (16)
            % WARNING: phi turns into NaN when l1=l2=l3. (it becomes atan( 0 / 0 ), i
            % think. So, DO NOT INPUT STRAIGHT CONFIGURATIONS TO THIS FUNCTION.

            kappa = 2 * sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3)) / ( d* sum(l) );
            
            phi = atan2( (sqrt(3)/3) * ( l(3) + l(2) - 2*l(1)) , (l(2)-l(3)) );

            %Debugging
            %disp([ 'phi: ' num2str(phi) ', l: ' num2str(l(1)) ', ' num2str(l(2)) ', ' num2str(l(3)) ])

            if isnan( phi )
                disp('Somethings wrong with phi!!!')
                % I don't believe this model was intended to help in situations where
                % the robot is straight.
                disp('Setting phi to -pi/2?')
                phi = -pi/2;
            end

            %% HTM

            % Equation 8 in the paper directly creates an htm for the system using
            % simple expressions for each entry.

            %Included in this matrix are the orientation steps which make the robot
            %point in the "z" direction. No additional rotations are needed.

            A = zeros(4);
            A(4,4) = 1;

            %Row 1
            A(1,1) = (cos(phi))^2 * ( cos(kappa*s) - 1) + 1;
            A(1,2) = sin(phi)*cos(phi)*( cos(kappa*s) - 1);
            A(1,3) = -cos(phi)*sin(kappa*s);
            A(1,4) = cos(phi)*( cos(kappa*s) - 1)/kappa;

            %Row 2
            A(2,1) = sin(phi)*cos(phi)*( cos(kappa*s) - 1);
            A(2,2) = (cos(phi))^2 * ( 1 - cos(kappa*s) ) + cos( kappa * s );
            A(2,3) = -sin(phi)*sin(kappa*s);
            A(2,4) = sin(phi)*( cos(kappa*s) - 1)/kappa;

            %Row 3
            A(3,1) = cos(phi)*sin(kappa*s);
            A(3,2) = sin(phi)*sin(kappa*s);
            A(3,3) = cos(kappa*s);
            A(3,4) = sin(kappa*s)/kappa;

        end

        %Rotate and then save result
        r = [ cos( correction_angle ) -sin(correction_angle) ;
              sin( correction_angle ) cos(correction_angle) ];

        htm_rot_correction = [ r zeros(2) ;
                               0 0 1 0 ;
                               0 0 0 1 ];

        iw_htm = (htm_rot_correction) * A ;
        res = iw_htm;

    case 'f1 (HTM)'
        %This indicates that we want to transform a point in the continuum robot configuration space
        %(s,kappa,phi) to task/world space (x,orientation)
        %Diagram Explanation:
        %   (s,kappa,phi) -> (x,y,z)

        %Process inputs

        if( nargin ~= 4 )
            disp([ 'Error. Received ' num2str(nargin) ' arguments; expected 4.' ]);
        end

        s     = varargin{2};
        kappa = varargin{3};
        phi   = varargin{4};

        if( s <= 0 )
            disp(['Expecting nonnegative number for s. Received ' num2str(s) '.' ])
            %Exit program.
            res = -1; return;
        end

        if( (kappa <= 0) || (kappa == inf) )
            disp([ 'Expecting nonnegative number for kappa. Received ' num2str(kappa) '.' ])
            %Exit Program.
            res = -1; return;
        end

        % Calculate HTM

        A = zeros(4);
        A(4,4) = 1;

        %Row 1
        A(1,1) = (cos(phi))^2 * ( cos(kappa*s) - 1) + 1;
        A(1,2) = sin(phi)*cos(phi)*( cos(kappa*s) - 1);
        A(1,3) = -cos(phi)*sin(kappa*s);
        A(1,4) = cos(phi)*( cos(kappa*s) - 1)/kappa;

        %Row 2
        A(2,1) = sin(phi)*cos(phi)*( cos(kappa*s) - 1);
        A(2,2) = (cos(phi))^2 * ( 1 - cos(kappa*s) ) + cos( kappa * s );
        A(2,3) = -sin(phi)*sin(kappa*s);
        A(2,4) = sin(phi)*( cos(kappa*s) - 1)/kappa;

        %Row 3
        A(3,1) = cos(phi)*sin(kappa*s);
        A(3,2) = sin(phi)*sin(kappa*s);
        A(3,3) = cos(kappa*s);
        A(3,4) = sin(kappa*s)/kappa;

        %Rotate and then save result
        r = [ cos( correction_angle ) -sin(correction_angle) ;
              sin( correction_angle ) cos(correction_angle) ];

        htm_rot_correction = [ r zeros(2) ;
                               0 0 1 0 ;
                               0 0 0 1 ];

        iw_htm = (htm_rot_correction) * A ;
        res = iw_htm;

    case 'f2'
        %This indicates that we want to transform a point in the actuator space (l) to 
        %configuration space (s,kappa,phi)
        %Diagram Explanation:
        %   l = (l1, l2, l3) -> (s,kappa,phi)

        %Process Inputs

        if( nargin ~= 5 )
            disp(['Error: Expected 5 arguments, received ' num2str(nargin) '.' ])
            %Exit program
            res = -1; return;
        end

        l      = varargin{2};
        l_back = varargin{3};
        d      = varargin{4};
        n      = varargin{5};

        if( sum( l <= 0 ) > 0 )
            disp([ 'Error: One of the input tendon lengths is negative!' ])
            %Exit program
            res = -1; return;
        end

        if( l_back <= 0 )
            disp( [ 'Error: Expect length of backbone to be nonnegative. Received ' num2str(l_back) '.' ])
            %Exit program
            res = -1; return;
        end

        if( n <= 0 )
            disp( [ 'n must be a nonnegative integer. Received ' num2str(n) '.' ] );
            %Exit Program early.
            res = -1; return;
        end

        if( d <= 0 )
            disp( [ 'd must be a positive number. Received ' num2str(d) '.' ] );
            %Exit program early.
            res = -1; return;
        end

        % Calculate Config. Space Variables

        if (l(1) == l(2)) && (l(1) == l(3)) && (l(2)==l(3))
            %When the robot is straight, the HTM can be quickly calculated.
            s = sum(l)/3;
            kappa = inf;
            phi = 0;        %Phi could be anything when the robot is straight. Just choose this
                            %So the program has a defined behavior.
                        
        else
            s = (n*d*sum(l))/(sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3)));
            s = s * asin( sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3))/(3*n*d));

            % Kappa and phi are defined by equations (15) and (16)
            % WARNING: phi turns into NaN when l1=l2=l3. (it becomes atan( 0 / 0 ), i
            % think. So, DO NOT INPUT STRAIGHT CONFIGURATIONS TO THIS FUNCTION.

            kappa = 2 * sqrt( sum( l.^2 ) - l(1)*l(2) - l(2)*l(3) - l(1)*l(3)) / ( d* sum(l) );
            
            phi = atan2( (sqrt(3)/3) * ( l(3) + l(2) - 2*l(1)) , (l(2)-l(3)) );

            %Debugging
            %disp([ 'phi: ' num2str(phi) ', l: ' num2str(l(1)) ', ' num2str(l(2)) ', ' num2str(l(3)) ])

            if isnan( phi )
                disp('Somethings wrong with phi!!!')
                % I don't believe this model was intended to help in situations where
                % the robot is straight.
                disp('Setting phi to -pi/2?')
                phi = -pi/2;
            end

        end

        res = [ s ; kappa ; phi ];

    otherwise
        disp([ 'First argument (' varargin{1} ') not currently recognized.' ]);
end

end

