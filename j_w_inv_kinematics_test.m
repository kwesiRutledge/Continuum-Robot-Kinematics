clear all;
close all;
clc;

% This script will verify how well the inverse kinematics from "Closed-Form Inverse Kinematics for Continuum Manipulators" works with simulated data and the forward Kinematics produced by Jones & Walker's 2006 Paper.

%% CONSTANTS

l0 = 42.5;	%Assume backbone is initially 42.5 mm.
n  = 4  ;	%Assume segment can bend within 4 distinct regions.
d  = 5  ;	%5 mm is the distance between the center of the manipulator and any of the motors/tendons.
			%Obtained by looking at CAD.

%% CREATE FAKE DATA

%Create fake tendon trajectory data.
itd = 0.9*l0;
matching_itd = (3*l0 - itd )/2;

num_steps = 100;
if( matching_itd > itd )
	step_sign = 1;
else
	step_sign = -1;
end

tendon_traj = [ [ itd : step_sign*abs(matching_itd - itd)/(num_steps-1) : matching_itd ]  ;
				[ matching_itd : -step_sign*abs(matching_itd - itd)/(num_steps-1) : itd ] ;
				ones(1,num_steps)*matching_itd ];

tendon_traj(:,:,2) = [ ones(1,num_steps)*matching_itd ;
					   [ itd : step_sign*abs(matching_itd - itd)/(num_steps-1) : matching_itd ] ;
					   [ matching_itd : -step_sign*abs(matching_itd - itd)/(num_steps-1) : itd ] ];

tendon_traj(:,:,3) = [ [ matching_itd : -step_sign*abs(matching_itd - itd)/(num_steps-1) : itd ] ;
					   ones(1,num_steps)*matching_itd ;
					   [ itd : step_sign*abs(matching_itd - itd)/(num_steps-1) : matching_itd ] ] ;

%Append all of the previous blocks together into a final tendon trajectory.
des_tendon_traj = [ ]
for k = 1:size(tendon_traj,3)

	des_tendon_traj = [ des_tendon_traj tendon_traj(:,:,k) ];

end

%Now that the tendon trajectory data is complete, compute the positions that these tendon lengths map to.
des_pos_traj = [];
for j = 1:size( des_tendon_traj , 2 )

	%store HTM in a temporary variable
	temp_HTM = iw_fwd_kinematics( des_tendon_traj(:,j) , l0 , n , d );
	des_pos_traj = [ des_pos_traj temp_HTM([1:3],4) ];

end

%% Test Inverse Kinematics.

%Now use inverse kinematics to try to tease out the tendon lengths/configuration variables.
for m = 1 : size( des_pos_traj , 2)

	phi(m) = atan2( des_pos_traj(2,m) , des_pos_traj(1,m) );

	kappa(m) = 2 * sqrt( sum( des_pos_traj([1:2],m ).^2 ) )/( sum( des_pos_traj(:,m).^2 ) );

end

figure;
subplot(1,2,1)
plot(phi)
title('\phi')

subplot(1,2,2)
plot(kappa)
title('\kappa')