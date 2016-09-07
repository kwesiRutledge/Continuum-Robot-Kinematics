%Prototyping creating a range of possible task space coordinates for 1
%segment bending in 2D.
% Written By Kwesi Rutledge 4.18.2016

clear all;
close all;
clc;

%% CONSTANTS

% Constants regarding robot design
ri = 0.025;         %Distance between central backbone and each tendon
                    %(when viewing the system's cross section)
li0 = 0.5;          %Length of central backbone

% Constants for creating map
delta_l = 0.1*li0;  %Maximum Deviation in length of a tendon while
                    %doing this mapping.

%% ACTUAL PROGRAM

l_list = [];        %This list will save (in each column) the three tendon
                    %lengths for the associated cartesian point in
                    %reachable_pts.

reachable_pts = []; %This will become a 2 row matrix,
                    %where each column will be the cartesian point
                    %associated with the deviations described in the
                    %l_list.

deviations = [-1:0.05:1]*delta_l;

%Select tendon lengths

for i = 1: length(deviations)
    
    
    if deviations(i) < 0
        %For i's that are negative we want to bend in the negative y
        %direction, which means that tendons 2 and 3 must shorten.
        l = [ li0 li0+deviations(i) li0+deviations(i) ];
        
    else
        if deviations(i) >= 0
            %For i's that are positive we want to bend in the positive y
            %direction, which means that tendons 2 and 3 must shorten.
            l = [ li0-deviations(i) li0 li0 ];
            
        else
            disp('Slipped through conditions. Error!')
        end
    end
    
    %Create HTM
    possible_htm = iw_fwd_kinematics( l , li0 , ri );

    %Extract possible point from the htm.
    reachable_pts(:,i) = possible_htm(2:3,4);
    l_list(:,i) = l';
    
end

%Plot map.

figure;
plot(reachable_pts(1,:),reachable_pts(2,:),'*')
title('Raw output of simple plotting of possible points')
xlabel('Y Axis')
ylabel('Z Axis') 

%Automatically save this as a map that I will use in other programs.

save('1seg_reachable_pts.mat','l_list','reachable_pts')
        

        
    