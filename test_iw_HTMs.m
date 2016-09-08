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

%% Use a gif to verify how these kinematics vary when directed OFF the direction of the actuators.

steps_in_l  = 100;
generated_ls = [ [ itd : (l0-itd)/(steps_in_l-1) : l0 ]  l0*ones(1,steps_in_l) l0*ones(1,steps_in_l) ;
				 l0*ones(1,steps_in_l)  [ itd : (l0-itd)/(steps_in_l-1) : l0 ] l0*ones(1,steps_in_l) ;
				 l0*ones(1,steps_in_l) l0*ones(1,steps_in_l) [ itd : (l0-itd)/(steps_in_l-1) : l0 ] ];

generated_ls2 = [ [ itd    :  (1.1*l0-itd)/(steps_in_l) : 1.1*l0 ] ;
			      [ 1.2*l0 : -(1.2*l0-itd)/(steps_in_l) : itd ] ] ;
generated_ls2 = [ generated_ls2 ;
				  (3*l0 - generated_ls2(1,:) - generated_ls2(2,:)) ];

generated_ls3 = [ [ itd    :  (1.1*l0-itd)/(steps_in_l) : 1.1*l0 ] ;
			      [ itd    :  (1.1*l0-itd)/(steps_in_l) : 1.1*l0 ] ] ;
generated_ls3 = [ generated_ls3 ;
				  (3*l0 - generated_ls3(1,:) - generated_ls3(2,:)) ];

generated_ls4 = [ [ itd    :  (l0-itd)/(steps_in_l) : l0 ] ;
			      [ itd    :  (l0-itd)/(steps_in_l) : l0 ] ] ;
generated_ls4 = [ generated_ls4 ;
				  (3*l0 - generated_ls4(1,:) - generated_ls4(2,:)) ];

%Choose data set
tendon_list = [ generated_ls3 generated_ls2 ];

%Prep GIF Stuff

num_iters = size(tendon_list,2);

figure(1)
axis([-100 100 -100 100 0 100])
filename = 'images/example_dynamics.gif';

saveGifImageEvery = 5; %Save Gif Image every ___ frames
dispFigureEvery   = 2;

htm_list = [];

for i = 1 : num_iters

    %Update Scatter of Points
    htm_list(:,:,i) = iw_fwd_kinematics( tendon_list(:,i) , l0 , d , n );
    
    if  ( (i == 1) || mod(i,dispFigureEvery) == 0 ) %This helps the plotting go by quicker.
        
    	%plot
    	scatter3( htm_list(1,4,:) , htm_list(2,4,:) , htm_list(3,4,:) )
    	hold on;
    	scatter3( 0 , 0 , l0 , 'r+' )
        %axis([-50 50 -50 50 30 50])
		xlabel('x (mm)')
		ylabel('y (mm)')
		zlabel('z (mm)')

        drawnow
        
        hold off;
        
        frame = getframe(1); %Save as a 'frame' whatever is in figure #3
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if i == 1
            imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
        else
            if( mod(i,saveGifImageEvery) == 0 )
                imwrite(imind,cm,filename,'gif','WriteMode','append');
            end
        end
        
    end
end