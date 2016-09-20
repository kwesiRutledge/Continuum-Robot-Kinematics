%This script will perform the Jacobian calculation for f_2b from Jones and
%Walker's 2006 paper, "Kinematics for Multisection Continuum Robots."
%Written By Kwesi Rutledge

clear all;
close all;
clc;

%% Defining Symbolic Variables necessary

syms n d l_1 l_2 l_3 real

%% Testing the Jacobian function

l = [ l_1 ; l_2 ; l_3 ]

jacobian(l,l)

%% Jacobian of f_2b

f_2b = [ ...
			((n * d * ( l_1 + l_2 + l_3 ) )/sqrt( l_1.^2 + l_2.^2 + l_3.^2 - l_1 * l_2 - l_2 * l_3 - l_1 * l_3) )* ...
			asin( sqrt( l_1.^2 + l_2.^2 + l_3.^2 - l_1 * l_2 - l_2 * l_3 - l_1 * l_3)/(3*n*d) ) ;
		2 * sqrt( l_1.^2 + l_2.^2 + l_3.^2 - l_1 * l_2 - l_2 * l_3 - l_1 * l_3)/( d * ( l_1 + l_2 + l_3 ) ) ;
		atan( (1/sqrt(3)) * (l_3+l_2-2*l_1)/(l_2-l_3) ) ...
]

J_f2b = jacobian(f_2b,l)

latex( J_f2b )