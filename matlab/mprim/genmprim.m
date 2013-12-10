% /*
%  * Copyright (c) 2008, Maxim Likhachev
%  * All rights reserved.
%  * 
%  * Redistribution and use in source and binary forms, with or without
%  * modification, are permitted provided that the following conditions are met:
%  * 
%  *     * Redistributions of source code must retain the above copyright
%  *       notice, this list of conditions and the following disclaimer.
%  *     * Redistributions in binary form must reproduce the above copyright
%  *       notice, this list of conditions and the following disclaimer in the
%  *       documentation and/or other materials provided with the distribution.
%  *     * Neither the name of the University of Pennsylvania nor the names of its
%  *       contributors may be used to endorse or promote products derived from
%  *       this software without specific prior written permission.
%  * 
%  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
%  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
%  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
%  * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
%  * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
%  * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
%  * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
%  * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
%  * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
%  * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%  * POSSIBILITY OF SUCH DAMAGE.
%  */

function[] = genmprim(outfilename)

%
%generates motion primitives and saves them into file
%
%written by Maxim Likhachev
%---------------------------------------------------
%

%defines

LINESEGMENT_MPRIMS = 1; %set the desired type of motion primitives
UNICYCLE_MPRIMS = 0;



if LINESEGMENT_MPRIMS == 1
    resolution = 0.01;
    numberofangles = 32; %preferably a power of 2, definitely multiple of 8
    numberofprimsperangle = 16;

    %multipliers (multiplier is used as costmult*cost)
    forwardcostmult = 1;
    backwardcostmult = 5;
    forwardandturncostmult = 1;
    sidestepcostmult = 50;
    turninplacecostmult = 50;
    
    %note, what is shown x,y,theta changes (not absolute numbers)
    
    %0 degreees
    basemprimendpts0_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts0_c(1,:) = [1 0 0 forwardcostmult];
    basemprimendpts0_c(2,:) = [4 0 0 forwardcostmult];
    basemprimendpts0_c(3,:) = [8 0 0 forwardcostmult];
    basemprimendpts0_c(4,:) = [6 2 0 sidestepcostmult];
    basemprimendpts0_c(5,:) = [6 -2 0 sidestepcostmult];
    basemprimendpts0_c(6,:) = [2 3 0 sidestepcostmult];
    basemprimendpts0_c(7,:) = [2 -3 0 sidestepcostmult];
    basemprimendpts0_c(8,:) = [-5 0 0 backwardcostmult];
    %1/32 theta change
    basemprimendpts0_c(9,:) = [6 2 1 forwardandturncostmult];
    basemprimendpts0_c(10,:) = [6 -2 -1 forwardandturncostmult];
    %2/32 theta change
    basemprimendpts0_c(11,:) = [4 3 2 forwardandturncostmult];
    basemprimendpts0_c(12,:) = [4 -3 -2 forwardandturncostmult];
    %turn in place
    basemprimendpts0_c(13,:) = [0 0 1 turninplacecostmult];
    basemprimendpts0_c(14,:) = [0 0 -1 turninplacecostmult];
    basemprimendpts0_c(15,:) = [0 0 3 turninplacecostmult];
    basemprimendpts0_c(16,:) = [0 0 -3 turninplacecostmult];    
    
    %45 degrees
    basemprimendpts45_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change 
    basemprimendpts45_c(1,:) = [1 1 0 forwardcostmult];
    basemprimendpts45_c(2,:) = [3 3 0 forwardcostmult];
    basemprimendpts45_c(3,:) = [6 6 0 forwardcostmult];    
    basemprimendpts45_c(4,:) = [2 6 0 sidestepcostmult];  
    basemprimendpts45_c(5,:) = [6 2 0 sidestepcostmult];    
    basemprimendpts45_c(6,:) = [0 4 0 sidestepcostmult];
    basemprimendpts45_c(7,:) = [4 0 0 sidestepcostmult];
    basemprimendpts45_c(8,:) = [-4 -4 0 backwardcostmult];    
    %1/32 theta change
    basemprimendpts45_c(9,:) = [2 6 1 forwardandturncostmult];
    basemprimendpts45_c(10,:) = [6 2 -1 forwardandturncostmult];    
    %2/32 theta change
    basemprimendpts45_c(11,:) = [1 5 2 forwardandturncostmult];
    basemprimendpts45_c(12,:) = [5 1 -2 forwardandturncostmult];    
    %turn in place
    basemprimendpts45_c(13,:) = [0 0 1 turninplacecostmult];
    basemprimendpts45_c(14,:) = [0 0 -1 turninplacecostmult];
    basemprimendpts45_c(15,:) = [0 0 3 turninplacecostmult];
    basemprimendpts45_c(16,:) = [0 0 -3 turninplacecostmult];    
    
    %22.5 degrees
    basemprimendpts22p5_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change     
    basemprimendpts22p5_c(1,:) = [2 1 0 forwardcostmult];
    basemprimendpts22p5_c(2,:) = [4 2 0 forwardcostmult];
    basemprimendpts22p5_c(3,:) = [6 3 0 forwardcostmult];    
    basemprimendpts22p5_c(4,:) = [4 4 0 sidestepcostmult];
    basemprimendpts22p5_c(5,:) = [6 2 0 sidestepcostmult];    
    basemprimendpts22p5_c(6,:) = [0 3 0 sidestepcostmult];
    basemprimendpts22p5_c(7,:) = [4 -1 0 sidestepcostmult];    
    basemprimendpts22p5_c(8,:) = [-4 -2 0 backwardcostmult];    
    %1/32 theta change
    basemprimendpts22p5_c(9,:) = [4 4 1 forwardandturncostmult];
    basemprimendpts22p5_c(10,:) = [6 2 -1 forwardandturncostmult];    
    %2/32 theta change
    basemprimendpts22p5_c(11,:) = [2 4 2 forwardandturncostmult];
    basemprimendpts22p5_c(12,:) = [6 0 -2 forwardandturncostmult];
    %turn in place
    basemprimendpts22p5_c(13,:) = [0 0 1 turninplacecostmult];
    basemprimendpts22p5_c(14,:) = [0 0 -1 turninplacecostmult];
    basemprimendpts22p5_c(15,:) = [0 0 3 turninplacecostmult];
    basemprimendpts22p5_c(16,:) = [0 0 -3 turninplacecostmult];    
    
    %11.25 degrees
    basemprimendpts11p25_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult (multiplier is used as costmult*cost)
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change     
    basemprimendpts11p25_c(1,:) = [3 1 0 forwardcostmult];
    basemprimendpts11p25_c(2,:) = [6 2 0 forwardcostmult];
    basemprimendpts11p25_c(3,:) = [9 3 0 forwardcostmult];
    basemprimendpts11p25_c(4,:) = [4 3 0 sidestepcostmult];
    basemprimendpts11p25_c(5,:) = [6 0 0 sidestepcostmult];
    basemprimendpts11p25_c(6,:) = [1 3 0 sidestepcostmult];
    basemprimendpts11p25_c(7,:) = [3 -2 0 sidestepcostmult];
    basemprimendpts11p25_c(8,:) = [-6 -2 0 backwardcostmult];
    %1/32 theta change
    basemprimendpts11p25_c(9,:) = [4 3 1 forwardandturncostmult];
    basemprimendpts11p25_c(10,:) = [6 0 -1 forwardandturncostmult];
    %2/32 theta change
    basemprimendpts11p25_c(11,:) = [2 4 2 forwardandturncostmult];
    basemprimendpts11p25_c(12,:) = [5 -1 -2 forwardandturncostmult];
    %turn in place
    basemprimendpts11p25_c(13,:) = [0 0 1 turninplacecostmult];
    basemprimendpts11p25_c(14,:) = [0 0 -1 turninplacecostmult];
    basemprimendpts11p25_c(15,:) = [0 0 3 turninplacecostmult];
    basemprimendpts11p25_c(16,:) = [0 0 -3 turninplacecostmult];    
    
    %33.75 degrees
    basemprimendpts33p75_c = zeros(numberofprimsperangle, 4); %x,y,theta,costmult 
    %x aligned with the heading of the robot, angles are positive
    %counterclockwise
    %0 theta change
    basemprimendpts33p75_c(1,:) = [3 2 0 forwardcostmult];
    basemprimendpts33p75_c(2,:) = [6 4 0 forwardcostmult];
    basemprimendpts33p75_c(3,:) = [9 6 0 forwardcostmult];    
    basemprimendpts33p75_c(4,:) = [4 5 0 sidestepcostmult];
    basemprimendpts33p75_c(5,:) = [6 2 0 sidestepcostmult];    
    basemprimendpts33p75_c(6,:) = [0 4 0 sidestepcostmult];    
    basemprimendpts33p75_c(7,:) = [3 -2 0 sidestepcostmult];    
    basemprimendpts33p75_c(8,:) = [-6 -4 0 backwardcostmult];    
    %1/32 theta change
    basemprimendpts33p75_c(9,:) = [4 5 1 forwardandturncostmult];
    basemprimendpts33p75_c(10,:) = [6 2 -1 forwardandturncostmult];
    %2/32 theta change    
    basemprimendpts33p75_c(11,:) = [1 5 2 forwardandturncostmult];
    basemprimendpts33p75_c(12,:) = [3 -2 -2 forwardandturncostmult];
    %turn in place
    basemprimendpts33p75_c(13,:) = [0 0 1 turninplacecostmult];
    basemprimendpts33p75_c(14,:) = [0 0 -1 turninplacecostmult];
    basemprimendpts33p75_c(15,:) = [0 0 3 turninplacecostmult];
    basemprimendpts33p75_c(16,:) = [0 0 -3 turninplacecostmult];        
    
    
elseif UNICYCLE_MPRIMS == 1
    fprintf(1, 'ERROR: unsupported mprims type\n');
    return;
else
    fprintf(1, 'ERROR: undefined mprims type\n');
    return;    
end;
    
    
fout = fopen(outfilename, 'w');


%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
fprintf(fout, 'totalnumberofprimitives: %d\n', numberofprimsperangle*numberofangles);

%iterate over angles
for angleind = 1:numberofangles
    
    figure(1);
    hold off;

    text(0, 0, int2str(angleind));
    
    %iterate over primitives    
    for primind = 1:numberofprimsperangle
        fprintf(fout, 'primID: %d\n', primind-1);
        fprintf(fout, 'startangle_c: %d\n', angleind-1);

        %current angle
        currentangle = (angleind-1)*2*pi/numberofangles;
        currentangle_36000int = round((angleind-1)*36000/numberofangles);
        
        %compute which template to use
        if (rem(currentangle_36000int, 9000) == 0)
            basemprimendpts_c = basemprimendpts0_c(primind,:);    
            angle = currentangle;
        elseif (rem(currentangle_36000int, 4500) == 0)
            basemprimendpts_c = basemprimendpts45_c(primind,:);
            angle = currentangle - 45*pi/180;
        elseif (rem(currentangle_36000int-7875, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts33p75_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts33p75_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts33p75_c(primind, 3); %reverse the angle as well
            angle = currentangle - 78.75*pi/180;
            fprintf(1, '78p75\n');
        elseif (rem(currentangle_36000int-6750, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts22p5_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts22p5_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts22p5_c(primind, 3); %reverse the angle as well
            %fprintf(1, '%d %d %d onto %d %d %d\n', basemprimendpts22p5_c(1), basemprimendpts22p5_c(2), basemprimendpts22p5_c(3), ...
            %    basemprimendpts_c(1), basemprimendpts_c(2), basemprimendpts_c(3));
            angle = currentangle - 67.5*pi/180;
            fprintf(1, '67p5\n');            
        elseif (rem(currentangle_36000int-5625, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            basemprimendpts_c(1) = basemprimendpts11p25_c(primind, 2); %reverse x and y
            basemprimendpts_c(2) = basemprimendpts11p25_c(primind, 1);
            basemprimendpts_c(3) = -basemprimendpts11p25_c(primind, 3); %reverse the angle as well
            angle = currentangle - 56.25*pi/180;
            fprintf(1, '56p25\n');
        elseif (rem(currentangle_36000int-3375, 9000) == 0)
            basemprimendpts_c = basemprimendpts33p75_c(primind,:);
            angle = currentangle - 33.75*pi/180;
            fprintf(1, '33p75\n');
        elseif (rem(currentangle_36000int-2250, 9000) == 0)
            basemprimendpts_c = basemprimendpts22p5_c(primind,:);
            angle = currentangle - 22.5*pi/180;
            fprintf(1, '22p5\n');
        elseif (rem(currentangle_36000int-1125, 9000) == 0)
            basemprimendpts_c = basemprimendpts11p25_c(primind,:);
            angle = currentangle - 11.25*pi/180;
            fprintf(1, '11p25\n');
        else
            fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentangle_36000int);
            return;
        end;
        
        %now figure out what action will be        
        baseendpose_c = basemprimendpts_c(1:3);
        additionalactioncostmult = basemprimendpts_c(4);        
        endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
        endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
        endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
        endpose_c = [endx_c endy_c endtheta_c];
        
        fprintf(1, 'rotation angle=%f\n', angle*180/pi);
        
        if baseendpose_c(2) == 0 & baseendpose_c(3) == 0
            %fprintf(1, 'endpose=%d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        end;
        
        %generate intermediate poses (remember they are w.r.t 0,0 (and not
        %centers of the cells)
        numofsamples = 10;
        intermcells_m = zeros(numofsamples,3);
        if LINESEGMENT_MPRIMS == 1
            startpt = [0 0 currentangle];
            endpt = [endpose_c(1)*resolution endpose_c(2)*resolution ...
                rem(angleind - 1 + baseendpose_c(3), numberofangles)*2*pi/numberofangles];
            intermcells_m = zeros(numofsamples,3);
            for iind = 1:numofsamples
                intermcells_m(iind,:) = [startpt(1) + (endpt(1) - startpt(1))*(iind-1)/(numofsamples-1) ...
                                        startpt(2) + (endpt(2) - startpt(2))*(iind-1)/(numofsamples-1) ...
                                        0];
                rotation_angle = (baseendpose_c(3) ) * (2*pi/numberofangles);
                intermcells_m(iind,3) = rem(startpt(3) + (rotation_angle)*(iind-1)/(numofsamples-1), 2*pi);
            end;
        end;
    
        %write out
        fprintf(fout, 'endpose_c: %d %d %d\n', endpose_c(1), endpose_c(2), endpose_c(3));
        fprintf(fout, 'additionalactioncostmult: %d\n', additionalactioncostmult);
        fprintf(fout, 'intermediateposes: %d\n', size(intermcells_m,1));
        for interind = 1:size(intermcells_m, 1)
            fprintf(fout, '%.4f %.4f %.4f\n', intermcells_m(interind,1), intermcells_m(interind,2), intermcells_m(interind,3));
        end;
        
        plot(intermcells_m(:,1), intermcells_m(:,2));
        text(intermcells_m(numofsamples,1), intermcells_m(numofsamples,2), int2str(endpose_c(3)));
        hold on;
        
    end;
    grid;
    pause;
end;
        
fclose('all');
