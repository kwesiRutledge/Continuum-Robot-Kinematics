function [ J ] = get_cp_jacobian( varargin )
%get_cp_jacobian Purpose is to calculate the "Curve Parametric" jacobian for a continuum manipulator segment
% 			  	 using the relationships outlined in Jones and Walker's 2006 Paper entitled Kinematics of
% 			  	 Continuum Robots.
%
% Possible Usages:
% 	get_iw_jacobian( 'D-H,f1' , s , kappa , phi )
%   get_iw_jacobian( 'f2'     , l , l0    , d   , n )
%
% Assumptions:
% 	- 'f2' option is not tested for the condition when l1 == l2 == l3. There should be a singularity here
%	  according to the CP model. Do not trust results when this occurs.
%
% Inputs:
% 	s 		, s is the arc length of the segment (the length of its curved central backbone)
% 	kappa 	, kappa is the curvature of the robot (1/"radius of curvature")
% 	phi 	, phi is the angle the segmetn creates relative to the axis
%			  (measured from the axis and contained in the xy-plane that the segment creates)
%   l       , l is a vector of the three tendon lengths (has 3 entries)
%             (I expect a row, but I think it should work either way)
%             l(1) = l1, l(2) = l2, etc. 
%   l0 		, The length of the flexible/central backbone
%   d      	, Distance from the central backbone to any of the tendons
%   n      	, The number of "segments" on the manipulators backbone.
%
% Requirements of Spaces:
%	The following spaces determine what variables you need to input into the program or manipulate.
%	
%	Configuration Space 		- (s,kappa,phi)
%	Actuator Space (tendons) 	- l , l0 , d , n

switch( varargin{1} )
	case 'D-H,f1'
		%This means to calculate the Jacobian between the position/orientation to configuration space variables
		% List of Transformations:
		% 	x -> Denavit-Hartenberg parameters -> (s,kappa, phi)

		%Process inputs
		s 	  = varargin{2};
		kappa = varargin{3};
		phi   = varargin{4};

		%Create constants
		J = zeros(6,3);
	
		%Rows copied directly from the 2006 Paper
		%First Row (x?)
		J(1,1) = -sin( phi )*(cos(kappa*s) - 1)/kappa;
		J(1,2) = -cos( phi )*( kappa*s*sin(kappa*s) + cos(kappa*s) - 1 )/(kappa.^2);
		J(1,3) = -cos( phi )*sin( kappa*s );

		%Second Row (y?)
		J(2,1) = 0;
		J(2,2) = ( kappa*s*cos(kappa*s) - sin(kappa*s) )/(kappa.^2);
		J(2,3) = cos( kappa*s );

		%Third Row (z?)
		J(3,1) = cos( phi )*(cos(kappa*s) - 1)/kappa;
		J(3,2) = sin( phi )*( kappa*s*sin(kappa*s) + cos(kappa*s) - 1 )/(kappa.^2);
		J(3,3) = sin( phi )*sin(kappa*s);

		%Fourth Row (Phi?)
		J(4,1) = cos( phi )*sin( kappa*s );
		J(4,2) = s*sin(phi);
		J(4,3) = kappa*sin(phi);

		%Fifth Row (Theta?)
		J(5,1) = 1 - cos(kappa*s);
		J(5,2) = 0;
		J(5,3) = 0;

		%Sixth Row (Psi?)
		J(6,1) = -sin( phi )*sin(kappa*s);
		J(6,2) = s*cos(phi);
		J(6,3) = kappa*cos(phi);


	case 'f2'
		%This means to calculate the Jacobian between the configuration space variables and tendon lengths
		% List of Transformations:
		% 	(s, kappa, phi) -> l = ( l1 , l2 , l3)

		%Process inputs
		l  = varargin{2};
		l0 = varargin{3};
		d  = varargin{4};
		n  = varargin{5};

		%Create constants
		J = zeros(3);

		%Derivative Performed in MATLAB. Expressions copied directly from that result.
		%First Row
		J(1,1) = (d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n))) ...
				/(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2) + ...
				 ((l(2) - 2*l(1) + l(3))*(l(1) + l(2) + l(3)))/(6*((- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) ...
				 - l(3)^2)/(9*d^2*n^2) + 1)^(1/2)*(- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) - l(3)^2)) + ...
				 (d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n)) ...
				 *(l(2) - 2*l(1) + l(3))*(l(1) + l(2) + l(3)))/(2*(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + ...
				  l(3)^2)^(3/2));

		J(1,2) = (d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n)))...
				/(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2) + ((l(1) - 2*l(2) + l(3))*...
				(l(1) + l(2) + l(3)))/(6*((- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) - l(3)^2)...
				/(9*d^2*n^2) + 1)^(1/2)*(- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) - l(3)^2)) + ...
				(d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n))*(l(1) - 2*l(2) + l(3))...
				*(l(1) + l(2) + l(3)))/(2*(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(3/2));

		J(1,3) = (d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n)))...
				/(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2) + ((l(1) + l(2) - 2*l(3))*...
				(l(1) + l(2) + l(3)))/(6*((- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) - l(3)^2)...
				/(9*d^2*n^2) + 1)^(1/2)*(- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) - l(3)^2)) + ...
				(d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n))*...
				(l(1) + l(2) - 2*l(3))*(l(1) + l(2) + l(3)))/(2*(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(3/2));

		%Second Row
		J(2,1) = - (2*(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)) ...
				/(d*(l(1) + l(2) + l(3))^2) - (l(2) - 2*l(1) + l(3))/(d*(l(1) + l(2) + l(3))* ...
				(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2));

		J(2,2) = - (2*(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)) ...
				/(d*(l(1) + l(2) + l(3))^2) - (l(1) - 2*l(2) + l(3))/(d*(l(1) + l(2) + l(3))* ...
				(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2));

		J(2,3) = - (2*(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)) ...
				/(d*(l(1) + l(2) + l(3))^2) - (l(1) + l(2) - 2*l(3))/(d*(l(1) + l(2) + l(3))* ...
				(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2));

		%Third Row
		J(3,1) = -(2*3^(1/2))/(3*((l(2) - 2*l(1) + l(3))^2/(3*(l(2) - l(3))^2) + 1)*(l(2) - l(3)));

		J(3,2) = (3^(1/2)/(3*(l(2) - l(3))) - (3^(1/2)*(l(2) - 2*l(1) + l(3)))/(3*(l(2) - l(3))^2)) ...
				/((l(2) - 2*l(1) + l(3))^2/(3*(l(2) - l(3))^2) + 1);

		J(3,3) = (3^(1/2)/(3*(l(2) - l(3))) + (3^(1/2)*(l(2) - 2*l(1) + l(3)))/(3*(l(2) - l(3))^2)) ...
				/((l(2) - 2*l(1) + l(3))^2/(3*(l(2) - l(3))^2) + 1);
		
	case 'D-H,f1,f2'
		%This means to calculate the Jacobian between the position/orientation to tendon lengths
		% List of Transformations:
		% 	x -> Denavit-Hartenberg parameters -> (s,kappa, phi)

		disp('This Jacobian feature is not yet available.')
		J = -1;
		return;

		J(1,1) = (d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n)))/(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2) + ((l(2) - 2*l(1) + l(3))*(l(1) + l(2) + l(3)))/(6*((- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) - l(3)^2)/(9*d^2*n^2) + 1)^(1/2)*(- l(1)^2 + l(1)*l(2) + l(1)*l(3) - l(2)^2 + l(2)*l(3) - l(3)^2)) + (d*n*asin((l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(1/2)/(3*d*n))*(l(2) - 2*l(1) + l(3))*(l(1) + l(2) + l(3)))/(2*(l(1)^2 - l(1)*l(2) - l(1)*l(3) + l(2)^2 - l(2)*l(3) + l(3)^2)^(3/2));


	otherwise
		disp('Unrecognized first input to the function:')
		disp(varargin{1})

end

end