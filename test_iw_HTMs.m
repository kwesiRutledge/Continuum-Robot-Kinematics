clear all;
close all;
clc;

%% Constants

l0 = 42.5;	%Assume backbone is initially 42.5 mm.
n  = 4  ;	%Assume segment can bend within 4 distinct regions.
d  = 5  ;	%5 mm is the distance between the center of the manipulator and any of the motors/tendons.
			%Obtained by looking at CAD.

testing_fwd_kin_directions = false;

num_pts_on_circle = 8;

%% Define the circle we are interested in by using iw_fwd_kinematics

%Start by defining a random point.
itd = 0.9*l0; 			%initial tendon deflection

%Because of the design of our robot we can easily find the length of the other 2 tendons.
first_tendons      = [ itd ones(1,2)*(( 3*l0 - itd )/2) ]
second_tendon_test = [ first_tendons(2) itd first_tendons(3) ]
third_tendon_test  = [ ones(1,2)*(( 3*l0 - itd )/2) itd ]

first_HTM  = iw_fwd_kinematics( first_tendons      , l0 , d )
second_HTM = iw_fwd_kinematics( second_tendon_test , l0 , d )
third_HTM  = iw_fwd_kinematics( third_tendon_test  , l0 , d )
