function [ res ] = cp_inv_kinematics( varargin )
%cp_inv_kinematics	This function uses the Curve Parametric Kinematics developed in Jones and Walkers's
%					2006 Paper entitled "Kinematics for Multisection Continuum Robots."
%
% Possible usages:
% l = cp_inv_kinematics( 'f2b' , s , kappa , phi , d , n )
% [   s   ]   
% [ kappa ] = cp_inv_kinematics( 'f1 (P)' , x )
% [  phi  ]
%

switch(varargin{1})
	case 'f2b'
		% f2b relates the configuration variables to the tendon lengths
		% This means that for the inverse kinematics calculation we will be moving in this direction:
		%  (s,kappa,phi) -> l = (l1,l2,l3)

		%Process Inputs
		if nargin ~= 6
			disp([ 'Inappropriate number of arguments. Expect 6, received ' num2str(nargin) '.'])
			%Exit program early.
			res = -1; return;
		end

		s     = varargin{2};
		kappa = varargin{3};
		phi   = varargin{4};
		d     = varargin{5};
		n     = varargin{6};

		if( s <= 0 )
			disp( [ 's must be a positive number. Received ' num2str(s) '.' ]);
			%Exit program early.
			res = -1; return;
		end

		if( kappa <= 0 )
			disp( [ 'Kappa must be a positive number. Received ' num2str(kappa) '.' ]);
			%Exit program early.
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

		% Calculate tendon lengths.
		l = [];

		l(1) = 2*n*sin( (kappa*s)/(2*n) ) * (1/kappa - d*sin(phi));
		l(2) = 2*n*sin( (kappa*s)/(2*n) ) * (1/kappa + d*sin(phi+pi/3) );
		l(3) = 2*n*sin( (kappa*s)/(2*n) ) * (1/kappa - d*cos(phi+pi/6) );

		res = l';

	case 'f1 (P)'
		% f1 relates the endpoint coordinate to configuration variables
		% This means that for the inverse kinematics calculation we will be moving in this direction:
		%  (x,y,z) -> (s,kappa,phi)

		%Process inputs

		if nargin ~= 2
			disp([ 'Inappropriate number of arguments. Expecting 2, received ' num2str(nargin) '.' ]);
			%Exit PRogram early.
			res = -1; return;
		end

		x = varargin{2};

		phi   = atan2( x(2) , x(1) );
		kappa = ( 2*norm(x([1 2])) )/( norm(x).^2 );

		% s calculation
		if x(3) > 0
			theta = acos( 1 - kappa * norm( x([1 2]) ) );
		else %if x(3) = z <= 0
			theta = 2*pi - acos( 1 - kappa * norm( x([1 2]) ) );
		end

		s = (1/kappa)*theta;

		res = [ s , kappa , phi ]';

	otherwise
		disp([ 'Unexpected first argument: ' varargin{1} ]);

end

end