function[] = genmprim_unicycle_circular_non_uniform (outfilename)
% Generates circular motion primitives with min turning radius and saves them into a file.
% For each angle from the list_of_angles array generates a set of 4 forward moving motion primitives 
% and 4 symmetric backward moving motion primitives. 
%
% This function takes one optional argument - 'outfilename', which specifies
% the location of the output file where motion primitives data is saved.
% If this argiment is omitted, the file name is hardcoded.
%
% The script builds the following types of motion primitives with unique ids for each
% angle from the list_of_angles array:
%
% if has_turn_in_place_prims == 0
%
% 1  -  short forward
% 2  -  long forward
% 3  -  forward and turn left
% 4  -  forward and turn right
% 5  -  short backward
% 6  -  long backward
% 7  -  backward and turn left
% 8  -  backward and turn right
%
% if has_turn_in_place_prims == 1
%
% 1  -  short forward
% 2  -  long forward
% 3  -  forward and turn left
% 4  -  forward and turn right
% 5 -   turn in place forward
% 6  -  short backward
% 7  -  long backward
% 8  -  backward and turn left
% 9  -  backward and turn right
% 10 -  turn in place backward
%
%
% Configurarion parameters:
%   resolution                      grid resolution
%   errmin                          max acceptable error for the distance between the endpoint of the motion primitive and the grid node.
%   rmin_m                          min acceptable turning radius im meters
%   xmin                            min x coordinate for the endpoint of the circular motion primitive
%   ymin                            min y coordinate for the endpoint of the circular motion primitive
%   xmax                            max x coordinate for the endpoint of the circular motion primitive
%   ymax                            max y coordinate for the endpoint of the circular motion primitive
%   countmax                        max number of different acceptable (by errmin and rmax) primitives of different lengths to calculate before choosing the best among them. 
%
%   alfa                            the angle satisfying the condition: alfa = atan(0.5)= 26.56505118 degrees, 0.46364761 radians
%   numberofangles                  number of angles for which motion primitives are generated
%   number_of_base_prims_per_angle  number of base (forward moving) primitives per angle
%   numberofbaseangles              number of base angles: 0, alfa and 45
%                                   degrees. All other angles are rotations or reflections of these base angles.
%   has_backward_prims              can be 1 or 0, if 0, backward moving primitives are not generated. 
%   list_if_angles                  list of angles for which sets of motion primitives are built   
%   degrees: 0 26.5651 45.00 63.4349  90.00  116.5651  135.00  153.4349  180.00  206.5651  225.00  243.4349  270.00  296.5651  315.00  333.4349  360.00
%   radians: 0 0.4538 0.7854 1.1170 1.5708 2.0246 2.3562 2.6878 3.1416 3.5954 3.9270  4.2586 4.7124 5.1662 5.4978  5.8294 6.2832
%   
%   Cost multipliers (multiplier is used as costmult*cost)
%   forwardcostmult                 cost multiplier for the forward motion
%   backwardcostmult                cost multiplier for the backward motion
%   forwardandturncostmult          cost multiplier for the forward-and-turn motion
%   backwardandturncostmult         cost multiplier for the backward-and-turn motion
%
% This script prints 3 coordinates for each motion primitive: x, y, theta
%%%%%

fprintf(1, 'genmprim_unicycle_circular_non_uniform\n');

% configuration parameters
resolution =  0.1; %0.025; %0.1; 
errmin = 0.05; %0.05;
rmin_m = 3.0; %1.0; 
rmin_c = rmin_m/resolution;   
fprintf(1, 'resolution=%f rmin_m=%f meters rmin_c=%f cells errmin=%f\n', resolution, rmin_m, rmin_c, errmin);

if nargin < 1
    %outputFile = 0;
    basefilename = 'non_uniform_';
    outfilename = generate_file_name(basefilename, resolution, rmin_m, errmin);
    fprintf(1, 'outfilename = %s\n', outfilename);
end

xmin = 0; 
ymin = 0; 
xmax = 40; 
ymax = 40;
countmax = 40; 
interm_spacing_m = resolution/2; % Approximate spacing of intermediate poses in meters
interm_spacing_c = interm_spacing_m/resolution;
interm_spacing_rad = 0.05;       % Approximate spacing of intermediate poses in radians
fprintf(1, 'xmin=%f xmax=%f ymin=%f ymax=%f\n', xmin, xmax, ymin, ymax);
fprintf(1, 'countmax=%d interm_spacing_m=%f meters interm_spacing_rad=%f radians\n', countmax, interm_spacing_m, interm_spacing_rad);

numberofangles = 16; 
%number_of_base_prims_per_angle = 4;
%max_number_of_prims_per_angle = 8;
numberofbaseangles = 3;                                 % 0, alfa and 45 degrees
has_backward_prims = 1;
has_turn_in_place_prims = 1;
number_of_base_prims_per_angle = 4 + has_turn_in_place_prims;
max_number_of_prims_per_angle = 2 * number_of_base_prims_per_angle; %this is only needed for arrays memory allocations
numberofprimsperangle = (1 + has_backward_prims) * number_of_base_prims_per_angle; % This is the total number of primitives per angle

totalnumberofprimitives = numberofprimsperangle*numberofangles;
fprintf(1, 'numberofangles=%d number_of_base_prims_per_angle=%d\n', numberofangles, number_of_base_prims_per_angle);
fprintf(1, 'has_backward_prims=%d\n', has_backward_prims);
fprintf(1, 'has_turn_in_place_prims=%d\n', has_turn_in_place_prims);
fprintf(1, 'numberofprimsperangle=%d totalnumberofprimitives=%d\n', numberofprimsperangle, totalnumberofprimitives);

start_x = 0.0;
start_y = 0.0;

%multipliers (multiplier is used as costmult*cost)
forwardcostmult = 1;
backwardcostmult = 5; 
forwardandturncostmult = 2;
backwardandturncostmult = 3;
turninplacecostmult = 5;

costmult = []
if (has_turn_in_place_prims)
    costmult = [forwardcostmult, forwardcostmult, forwardandturncostmult, forwardandturncostmult, turninplacecostmult, backwardcostmult, backwardcostmult, backwardandturncostmult, backwardandturncostmult, turninplacecostmult];
else
    costmult = [forwardcostmult, forwardcostmult, forwardandturncostmult, forwardandturncostmult, tbackwardcostmult, backwardcostmult, backwardandturncostmult, backwardandturncostmult];
end;    

% list of angles increments in discrete units for the end pose of the primitive relative to the start pose
prim_angle_increments = [0, 0, 1, -1];

alfa = 26.56505118; 
alfa1 = alfa*100000000;
beta = 90 - alfa;
beta1 = beta*100000000;
imax = has_backward_prims + 1;
fprintf(1, 'alfa1=%f beta=%f beta1=%f imax=%d\n', alfa1, beta, beta1, imax);

%list_of_angles (degrees): 0 26.5651 45.00 63.4349  90.00  116.5651  135.00  153.4349  180.00  206.5651  225.00  243.4349  270.00  296.5651  315.00  333.4349  360.00
%list_of_angles (radians): 0 0.4538 0.7854 1.1170 1.5708 2.0246 2.3562 2.6878 3.1416 3.5954 3.9270  4.2586 4.7124 5.1662 5.4978  5.8294 6.2832
fprintf(1, '*******************************\n');
list_of_angles = zeros(1, numberofangles+1);
list_of_angles(1:2:17) = (0:45:360);
list_of_angles([2 6 10 14]) = list_of_angles([1 5 9 13]) + alfa;
list_of_angles([4 8 12 16]) = list_of_angles([5 9 13 17]) - alfa;
list_of_angles
list_of_angles = list_of_angles .* pi/180;
list_of_angles
fprintf(1, '*******************************\n');

fout = fopen(outfilename, 'w');
%write the header
fprintf(fout, 'resolution_m: %f\n', resolution);
fprintf(fout, 'min_turning_radius_m: %f\n', rmin_m);
fprintf(fout, 'numberofangles: %d\n', numberofangles);
for angleind = 1:numberofangles
    fprintf(fout, 'angle:%d %.8f\n', angleind-1, list_of_angles(angleind));
end;
fprintf(fout, 'totalnumberofprimitives: %d\n', totalnumberofprimitives);
     
% Note, what is shown x,y,theta changes (not absolute numbers) x aligned
% with the heading of the robot, angles are positive and counterclockwise
    
% Initialize arrays to store acceptable circular primitives before choosing the best among them. 
% We only need 3 base sets of primitives for angles 0, alfa and 45 degrees.
primdata = zeros(numberofbaseangles, max_number_of_prims_per_angle, countmax, 8); % startx, starty, starttheta, endx, endy, endtheta, tvoverrv, rv
primcount = zeros(numberofbaseangles, max_number_of_prims_per_angle);
basemprimendpts_c = zeros(numberofbaseangles, numberofprimsperangle, 6); % x_c, y_c, theta_c, costmult, tvoverrv, rv
        
% Calculate sets of valid primitives with ids 2,3,4 for base angles 0, alfa and 45 degrees. 
% Primitives with id=1 (short forward) will be calculated by re-scaling primitives with id=2 (long forward).
for angleind=1:3
    for primind=2:4
        if (angleind == 1 && primind == 4)
            gen_base_prims(angleind, primind, xmin, xmax, -ymax, -ymin);
        else
            gen_base_prims(angleind, primind, xmin, xmax, ymin, ymax);
        end;
    end;
end;

fprintf(1, '\n');
fprintf(1, 'find best primitives start:\n');
find_best_prims();

fprintf(1, '\n++++++++++++++++++\n');
fprintf(1, 'start writing primitives to file:\n');
fprintf(1, '\n');

h1f = figure(1);
h1a = axes('parent',h1f);
hold on;

s = 2.0; %0.6;
xxmin = -s;
xxmax = s;
yymin = -s;
yymax = s;
xt = 5*resolution;
yt = 5*resolution;
            
axis equal;
axis([xxmin xxmax yymin yymax]);
%axis([-0.5 0.5 -0.5 0.5]);
set(gca,'XTick', xxmin:xt:xxmax);
set(gca,'YTick',yymin:yt:yymax);
grid on;

primposes = [];
primsamples = [];
primendangle = [];

% Iterate over angles
for angleind = 1:numberofangles 
    fprintf(1, '***************\n');
    fprintf(1, 'angleind=%d\n', angleind-1); 

    currentangle = list_of_angles(angleind);
    currentangle_36000int = round(currentangle*18000000000/pi);
            
    fprintf(1, '--------------------\n');
    fprintf(1, 'currentangle=%f currentangle_36000int=%d\n', currentangle, currentangle_36000int); 

    numofsamples0 = zeros(numberofprimsperangle);
    endPoses_c = zeros(numberofprimsperangle, 3); % x_c, y_c, theta_c
    turningradiuses = zeros(numberofprimsperangle);
    intermposes_m = [];
    intermcells_m_orig = []; %zeros(numofsamples, 3); %x,y,theta
    
    for primind = 1:numberofprimsperangle %number_of_base_prims_per_angle %[1,3,4] 
        
        fprintf(1, '+++++++++++++++++++++++++++\n');
        fprintf(1, 'primind=%d\n', primind-1);
        
        %compute which template to use
        if (rem(currentangle_36000int, 9000000000) == 0)
            mprimendpts_c = basemprimendpts_c(1, primind,:);
            angle = currentangle;
            turning_radius = mprimendpts_c(5);
            rvel = mprimendpts_c(6);
            fprintf(1, '0000  angle=%f\n', angle);
        elseif (rem(currentangle_36000int, 4500000000) == 0)
            mprimendpts_c =  basemprimendpts_c(3, primind,:); 
            angle = currentangle - 45*pi/180;
            turning_radius = mprimendpts_c(5);
            rvel = mprimendpts_c(6);
            fprintf(1, '4500 angle=%f\n', angle);
        elseif (rem(currentangle_36000int-beta1, 9000000000) == 0) %6400
            mprimendpts_c =  basemprimendpts_c(2, primind,:); 
            mprimendpts_c(1) =  basemprimendpts_c(2, primind,2); %reeverse x and y
            mprimendpts_c(2) =  basemprimendpts_c(2, primind,1); 
            mprimendpts_c(3) =  -basemprimendpts_c(2, primind,3); %reverse the angle as well
            angle = currentangle - beta*pi/180;
            turning_radius = -mprimendpts_c(5);
            rvel = -mprimendpts_c(6);
            fprintf(1, '90-alfa angle=%f\n', angle); 
        elseif (rem(currentangle_36000int-alfa1, 9000000000) == 0)
            mprimendpts_c =  basemprimendpts_c(2, primind,:);
            angle = currentangle - alfa*pi/180;
            turning_radius = mprimendpts_c(5);
            rvel = mprimendpts_c(6);
            fprintf(1, 'alfa angle=%f\n', angle);
        else
            fprintf(1, 'ERROR: invalid angular resolution. angle = %d\n', currentangle_36000int);
            return;
        end;
        
        [res, turning_radius_center] = generate_intermediate_poses(angleind, primind, currentangle, angle, rvel, turning_radius, mprimendpts_c);
            
        for ii = 1:numofsamples   
                intermposes_m(primind, ii, :) = [intermcells_m(primind,ii,1)*resolution intermcells_m(primind,ii,2)*resolution intermcells_m(primind,ii,3)];  % x, y, theta
                %intermposes_m_orig(primind, iind, :) = [sgn*intermcells_m_orig(iind,1) sgn*intermcells_m_orig(iind,2) intermcells_m(iind,3)];  % x, y, theta
                primposes(angleind, primind, ii, :) = intermposes_m(primind, ii, :);
        end;
        %pause;
        
    end; %for primind = 1:numberofprimsperangle
    
    %output to the mprim file and plot
    for primind = 1:numberofprimsperangle 
            fprintf(fout, 'primID: %d\n', primind-1);
            fprintf(fout, 'startangle_c: %d\n', angleind-1);
            fprintf(fout, 'endpose_c: %d %d %d\n', endPoses_c(primind,1), endPoses_c(primind,2), endPoses_c(primind,3));
                      
            additionalcostmult = costmult(primind);
            
            fprintf(fout, 'additionalactioncostmult: %d\n', additionalcostmult);
            fprintf(fout, 'turning_radius: %.4f\n', turningradiuses(primind));
            
            numofsamples = numofsamples0(primind);
            
            fprintf(fout, 'intermediateposes: %d\n', numofsamples); 
            for interind = 1:numofsamples 
                fprintf(fout, '%.4f %.4f %.4f\n', intermposes_m(primind,interind,1),  intermposes_m(primind,interind,2),  intermposes_m(primind,interind,3));
            end;
            plot(squeeze(intermposes_m(primind, 1:numofsamples, 1)), squeeze(intermposes_m(primind, 1:numofsamples, 2)), 'r');
       
            dl = 0.0;
            if (primind == 3)
                dl = 0.05;
            elseif (primind == 4) 
                dl = -0.05;
            end;
            slabel = [int2str(angleind), '-', int2str(primind)];
            %text(intermposes_m(primind,numofsamples,1)+dl, intermposes_m(primind,numofsamples,2)+dl, slabel); 
            
    end; %primid
    %pause;
end; %angleid

%%%%%%%%%%%%%%%%%%%%%%%******************************

fprintf(1, '!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n');
fclose('all');

function[] = gen_base_prims(angle_ind, prim_ind, xmin0, xmax0, ymin0, ymax0)
  
    fprintf(1, '\ngen_base_prims: angle_ind=%d prim_ind=%d\n', angle_ind, prim_ind);
    angle_incr = prim_angle_increments(prim_ind);
    start_theta = list_of_angles(angle_ind);
    if (angle_ind == 1 && angle_incr == -1)
        end_theta = -list_of_angles(2); 
    else
        end_theta = list_of_angles(angle_ind + angle_incr); 
    end;

    for i=1:imax
        count = 0; 
        if (i == 1)
            xmn0 = xmin0;
            xmx0 = xmax0;
            ymn0 = ymin0;
            ymx0 = ymax0;
            prm_ind = prim_ind;
        else
            xmn0 = -xmax0;
            xmx0 = -xmin0;
            ymn0 = -ymax0;
            ymx0 = -ymin0;
            prm_ind = prim_ind + number_of_base_prims_per_angle;   %4;
        end;
        basemprimendpts_c(angle_ind, prm_ind, 3) = angle_incr;
      
        fprintf(1, '\nangle_ind=%d prm_ind=%d\n', angle_ind, prm_ind);
        fprintf(1, 'start_theta=%f end_theta=%f basemprimendpts_c(3)=%f angle_incr=%d\n', start_theta, end_theta, basemprimendpts_c(angle_ind,prim_ind,3), angle_incr);
        
        %fprintf(1,'xmn0=%f xmx0=%f\n', xmn0,xmx0);
        %fprintf(1,'ymn0=%f ymx0=%f\n', ymn0,ymx0);
        
        for endx_c = xmn0:xmx0
            for endy_c = ymn0:ymx0
                prim_endx_c = endx_c;
                prim_endy_c = endy_c;

                if (prim_ind == 3 || prim_ind == 4)
                    R = [sin(end_theta) - sin(start_theta); cos(start_theta) - cos(end_theta)];
                    RT = R';
                    B = [prim_endx_c - start_x; prim_endy_c - start_y];
                    tvoverrv = pinv(RT*R)*RT*B;
                    if (abs(tvoverrv) < rmin_c) 
                        continue;
                    end;

                    rv = end_theta - start_theta;
                    tv = rv*tvoverrv;
                    err = abs(R*tvoverrv - B);
                    errxy = norm(err); 

                    if (errxy < errmin)    
                        fprintf(1, 'count=%d endx_c=%d endy_c=%d errxy=%f tvoverrv=%f rv=%f tv=%f prim_endx_c=%f prim_endy_c=%f\n', count, endx_c, endy_c, errxy, tvoverrv, rv, tv, prim_endx_c, prim_endy_c);
                    end
                else
                    yy = prim_endx_c*tan(end_theta);
                    errxy = abs(prim_endy_c - yy);
                    tvoverrv = sqrt(prim_endx_c*prim_endx_c + yy*yy);
                    rv = 0.0;
                    tv = tvoverrv;
    %                 if (errxy < 2*errmin) 
    %                     fprintf(1, 'count=%d endx_c=%d endy_c=%d errxy=%f tvoverrv=%f rv=%f tv=%f prim_endx_c=%f prim_endy_c=%f\n', count, endx_c, endy_c, errxy, tvoverrv, rv, tv, prim_endx_c, prim_endy_c);
    %                 end
                end; 

                if (errxy > errmin) 
                     continue;
                end;

                if (count+1 > countmax) 
                    fprintf(1, 'endy: count = %d, countmax = %d\n', count, countmax);
                    break;
                end;
                count = count + 1;
                
                primdata(angle_ind, prm_ind, count, 1) = start_x;
                primdata(angle_ind, prm_ind, count, 2) = start_y;
                primdata(angle_ind, prm_ind, count, 3) = start_theta;
                primdata(angle_ind, prm_ind, count, 4) = prim_endx_c;
                primdata(angle_ind, prm_ind, count, 5) = prim_endy_c;
                primdata(angle_ind, prm_ind, count, 6) = end_theta;
                primdata(angle_ind, prm_ind, count, 7) = tvoverrv;
                primdata(angle_ind, prm_ind, count, 8) = rv;
                %pause;
            end; %for end_y

            if (count+1 > countmax) 
                fprintf(1, 'endx: count = %d, countmax = %d\n', count, countmax);
                break;
            end;

        end; %for end_x
        primcount(angle_ind, prm_ind) = count;
        fprintf(1, 'count=%f\n', primcount(angle_ind, prm_ind));
        
    end %for i=1:imax
end

function[] = find_best_prims()
    % First find the motion primitive for angle_ind = 1 and prim_ind = 4
    % with minimal turning radius among selected set of primitives that
    % satisfy the given min turning radius and epsilon conditions.
    
    [rmin0, countmin] = min(abs(primdata(1, 4, 1:primcount(1, 4), 7)));
    fprintf(1, 'countmin=%f rmin0=%f\n', countmin, rmin0);
   
    %Iterate over all saved motion primitives for all angles and for each
    %angle find a set of primitives with turning radius closest to the min radius found above. 
    
    rrm = rmin0;
    %rrm = 30.0;
    %rad = [];
    
    for angle_id=1:numberofbaseangles
    
        fprintf(1, '*************  angle_id=%f  *********\n', angle_id);
    
        i1 = number_of_base_prims_per_angle + 3;  %7;
        i2 = number_of_base_prims_per_angle + 4;  %8;
        
        for prim_ind = [3,4,i1,i2] %4:-1:3
            if (has_backward_prims == 0 && prim_ind > number_of_base_prims_per_angle)  %>4
                continue;
            end
                        
            fprintf(1, '-------------  prim_ind=%f  -----------\n', prim_ind);  
                        
%            [dmin, countmin] = min(abs(abs(primdata(angle_id, prim_ind, 1:primcount(angle_id, prim_ind), 7)) - rmin0));
%              if (angle_id == 1)
%                  rrm = rmin0;
%              elseif (prim_ind == 3)
%                  rrm = rad(angle_id, 4);
%              else
%                  rrm = rad(angle_id-1, 3);
%              end;
             fprintf(1, 'rrm=%f\n', rrm);
             
             fprintf(1, 'primcount(angle_id, prim_ind)=%d\n', primcount(angle_id, prim_ind));
            
            [dmin, countmin] = min(abs(abs(primdata(angle_id, prim_ind, 1:primcount(angle_id, prim_ind), 7)) - rrm));
            fprintf(1, 'countmin=%f dmin=%f\n', countmin, dmin);
        
            tvoverrv = primdata(angle_id, prim_ind, countmin, 7);
            %rad(angle_id, prim_ind) = abs(tvoverrv);
            
            rv = primdata(angle_id, prim_ind, countmin, 6) - primdata(angle_id, prim_ind, countmin, 3);
            tv = rv*tvoverrv;
            fprintf(1, 'tvoverrv=%f rv=%f tv=%f\n', tvoverrv, rv, tv);

            basemprimendpts_c(angle_id, prim_ind, 1) = primdata(angle_id, prim_ind, countmin, 4);
            basemprimendpts_c(angle_id, prim_ind, 2) = primdata(angle_id, prim_ind, countmin, 5);
            basemprimendpts_c(angle_id, prim_ind, 5) = primdata(angle_id, prim_ind, countmin, 7);
            basemprimendpts_c(angle_id, prim_ind, 6) = primdata(angle_id, prim_ind, countmin, 8);
            
            fprintf(1, 'basemprimendpts_c=%f %f %f %f %f %f\n', basemprimendpts_c(angle_id, prim_ind, :)); %x, y, angle_incr, costmult,tvoverrv, rv
        end;
           
        for i = 1:imax
            %prim_ind = 2; %long forward
            prim_ind = 2 + (i-1)*number_of_base_prims_per_angle;  %4;
            fprintf(1, '-------------  prim_ind=%f  -----------\n', prim_ind); 
            i1 = prim_ind+1;
            i2 = prim_ind+2;
            
            x = (basemprimendpts_c(angle_id, i1, 1) + basemprimendpts_c(angle_id, i2, 1))/2;
            y = (basemprimendpts_c(angle_id, i1, 2) + basemprimendpts_c(angle_id, i2, 2))/2;
            fprintf(1, 'x=%f y=%f primcount=%d\n', x, y, primcount(angle_id, prim_ind));
        
            [dmin, countmin] = min(sqrt((primdata(angle_id, prim_ind, 1:primcount(angle_id, prim_ind), 4)-x).^2 + (primdata(angle_id, prim_ind, 1:primcount(angle_id, prim_ind), 5)-y).^2));
        
            fprintf(1, 'angle_id=%d prim_ind=%d countmin=%d dmin=%f\n', angle_id, prim_ind, countmin, dmin);
            basemprimendpts_c(angle_id, prim_ind, 1) = primdata(angle_id, prim_ind, countmin, 4);
            basemprimendpts_c(angle_id, prim_ind, 2) = primdata(angle_id, prim_ind, countmin, 5);
            fprintf(1, 'basemprimendpts_c=%f %f %f %f %f %f\n', basemprimendpts_c(angle_id, prim_ind, :));
        
            %prim_ind = 1; %short forward
            i1 = prim_ind;
            prim_ind = prim_ind - 1;
            fprintf(1, '-------------  prim_ind=%f  -----------\n', prim_ind); 
            x = basemprimendpts_c(angle_id, i1, 1);
            y = basemprimendpts_c(angle_id, i1, 2);
            d = nonzero_min(abs(x), abs(y));
            basemprimendpts_c(angle_id, prim_ind, 1) = x/d;
            basemprimendpts_c(angle_id, prim_ind, 2) = y/d; 
            fprintf(1, 'basemprimendpts_c=%f %f %f %f %f %f\n', basemprimendpts_c(angle_id, prim_ind, :));
            
            %prim_ind = 5; %short forward
            if (has_turn_in_place_prims == 1)
                prim_ind = 5 + (i-1)*number_of_base_prims_per_angle;
                fprintf(1, '-------------  prim_ind=%f  -----------\n', prim_ind); 
                basemprimendpts_c(angle_id, prim_ind, 1) = 0.0;
                basemprimendpts_c(angle_id, prim_ind, 2) = 0.0; 
                if (prim_ind == 5)
                    i1 = prim_ind-2;
                else
                    i1 = prim_ind-1;
                end;
                basemprimendpts_c(angle_id, prim_ind, 3) = basemprimendpts_c(angle_id, i1, 3); 
                basemprimendpts_c(angle_id, prim_ind, 5) = 0.0;
                basemprimendpts_c(angle_id, prim_ind, 6) = basemprimendpts_c(angle_id, i1, 6); 
                fprintf(1, 'basemprimendpts_c=%f %f %f %f %f %f\n', basemprimendpts_c(angle_id, prim_ind, :));
            end
        end;
    end;
end

%generate intermediate poses (remember they are w.r.t 0,0 (and not centers of the cells)
function[res, turning_radius_center] = generate_intermediate_poses(angleind, primind, currentangle, angle, rvel, turning_radius, mprimendpts_c)
      
    baseendpose_c = mprimendpts_c(1:3);
    endtheta_c = rem(angleind - 1 + baseendpose_c(3), numberofangles);
   
    fprintf(1,'\n');
    fprintf(1, 'generate_intermediate_poses start\n');
    fprintf(1, 'baseendpose_c: %f %f %f endtheta_c=%d\n', baseendpose_c, endtheta_c); 
    startpt = [0 0 currentangle];
        
    endx_c = round(baseendpose_c(1)*cos(angle) - baseendpose_c(2)*sin(angle));        
    endy_c = round(baseendpose_c(1)*sin(angle) + baseendpose_c(2)*cos(angle));
    if (angleind == 1 && endtheta_c == -1) 
        end_angle = -list_of_angles(2);
    else
        end_angle = list_of_angles(endtheta_c + 1);
    end;
    endpt = [endx_c endy_c end_angle];
        
    dist_m = sqrt(sum((endpt(1:2) - startpt(1:2)).^2));
    dist_rad = abs(currentangle - end_angle);
    fprintf(1, 'currentangle=%f end_angle=%f\n', currentangle, end_angle);

    if (endx_c == 0 && endy_c == 0)
        numofsamples = abs(round(dist_rad/interm_spacing_rad));
    else
        numofsamples = floor(dist_m/interm_spacing_c) + 1;
    end;
    %numofsamples = 1 + max(round(dist_m/interm_spacing_m), abs(round(dist_rad/interm_spacing_rad)));
    fprintf(1, 'endx_c=%f endy_c=%f endtheta_c=%d turning_radius=%f rvel=%f transformation angle=%f end_angle=%f numofsamples=%d dist_m=%f dist_rad=%f\n', endx_c, endy_c, endtheta_c, turning_radius, rvel, angle*180/pi, end_angle, numofsamples, dist_m, dist_rad);
    numofsamples0(primind) = numofsamples;
                  
    if (baseendpose_c(3) == 0) %move forward/backward   
        for iind = 1:numofsamples
            dt = (iind-1)/(numofsamples-1);
            intermcells_m(primind, iind,:) = [startpt(1) + (endpt(1) - startpt(1))*dt ...
                                    startpt(2) + (endpt(2) - startpt(2))*dt ...
                                    rem(startpt(3) + (endpt(3) - startpt(3))*dt, 2*pi)];
        end;   
        res = correct_poses();
        turning_radius_center = 0.0;
    elseif (endx_c == 0 && endy_c == 0) %turn in place
         for iind = 1:numofsamples
            dt = (iind-1)/(numofsamples-1);
            dtheta = rvel*dt + startpt(3);
            intermcells_m(primind, iind,:) = [0.0 0.0 dtheta];
         end;
         res = correct_poses();
         turning_radius_center = 0.0;
    else
        for iind = 1:numofsamples  
            dt = (iind-1)/(numofsamples-1);
      
            dtheta = rvel*dt + startpt(3);              
            x = startpt(1) + turning_radius*(sin(dtheta) - sin(startpt(3)));
            y = startpt(2) + turning_radius*(cos(startpt(3)) - cos(dtheta));
            intermcells_m(primind, iind,:)  = [x y dtheta];
%             fprintf(1, 'iind=%d dt=%f x=%f y=%f dtheta=%f %f %f\n', iind, dt, intermcells_m(iind,1), intermcells_m(iind,2), dtheta, x, y);
        end;     
        res = correct_poses();
        [turning_radius_center, arc_len_m, turn_angle] = average_turning_radius();
    end; 
    %pause;
      
    k = primind;
    endPoses_c(k, :) =  [round(endx_c) round(endy_c) round(endtheta_c)]; 
    turningradiuses(k) = turning_radius*resolution;
    turningradiuses_center(k) = turning_radius_center;

    primsamples(angleind, k) = numofsamples;
    primendangle(angleind, k) = endtheta_c;
    %pause;
    
    function[turn_rad_aver, arc_len_m, turn_angle] = average_turning_radius()
        
        turn_angle = 0.0;
        turn_rad_aver = 0.0;
        arc_len_m = 0.0;
        for ind = 1:numofsamples 
            if (ind > 1)
                dx = intermcells_m(primind, ind,1) - intermcells_m(primind, ind-1,1);
                dy = intermcells_m(primind, ind,2) - intermcells_m(primind, ind-1,2);
                ds = sqrt(dx*dx + dy*dy);
                dth = (intermcells_m(primind, ind,3) - intermcells_m(primind, ind-1,3));
                arc_len_m = arc_len_m + ds;
                turn_angle = turn_angle + dth;
                turn_rad = ds/dth;
                turn_rad_aver = turn_rad_aver + abs(turn_rad);
                %fprintf(1, 'ind=%d ds=%f dth=%f turn_rad=%f\n', ind, ds, dth, turn_rad);
            end;
        end
        turn_rad_aver = turn_rad_aver/(numofsamples - 1);
        turn_angle = abs(turn_angle);
        fprintf('arc_len_m = %f turn_angle = %f turn_rad_aver = %f\n', arc_len_m, turn_angle, turn_rad_aver);
    end
        
    %correct intermediate poses using interpolation to place the end point of the motion primitive exactly to the grid node.
    function[status] = correct_poses()
        
        status = true;
        %error = [endpt(1) - intermcells_m(primind, numofsamples,1) endpt(2) - intermcells_m(primind, numofsamples,2) endpt(3) - intermcells_m(primind, numofsamples,3)];
        errx = endpt(1) - intermcells_m(primind, numofsamples,1);
        erry = endpt(2) - intermcells_m(primind, numofsamples,2);
        errth = endpt(3) - intermcells_m(primind, numofsamples,3);
        
        fprintf(1, '********************\n');
        fprintf(1, 'correction:\n');
        fprintf(1,'numofsamples = %d\n', numofsamples);
        fprintf(1, 'startpt(1)=%f startpt(2)=%f startpt(3)=%f\n', startpt(1), startpt(2), startpt(3));
        fprintf(1, 'endpt(1)=%f endpt(2)=%f endpt(3)=%f\n', endpt(1), endpt(2), endpt(3));
        fprintf(1, 'mprimendpts_c(1)=%f mprimendpts_c(2)=%f mprimendpts_c(3)=%f\n', mprimendpts_c(1), mprimendpts_c(2), mprimendpts_c(3));
        
        fprintf(1, 'error: dx %f dy %f dtheta %f\n', errx, erry, errth);
        
        R0 = [sin(end_angle) - sin(startpt(3)); cos(startpt(3)) - cos(end_angle)];
        RT0 = R0';
        B0 = [endpt(1) - start_x; endpt(2) - start_y];
        tr = abs(pinv(RT0*R0)*RT0*B0);
        fprintf(1, 'tr = %f\n', tr);
    
        if (tr > 0 && tr < rmin_m) 
            fprintf(1, 'WARNING: After interpolation turning radius is less than the specified min!\n');
            status = false;
        end;
                
        interpfactor = 0:1/(numofsamples-1):1;
        
        %save original motion for display purposes only
        for i=1:numofsamples
            intermcells_m_orig(primind, i,1) = intermcells_m(primind, i,1); 
            intermcells_m_orig(primind, i,2) = intermcells_m(primind, i,2); 
            intermcells_m_orig(primind, i,3) = intermcells_m(primind, i,3); 
        end;
                
        %correct intermideate poses using interpolation
        for i=1:numofsamples
            intermcells_m(primind, i,1) = intermcells_m(primind, i,1) + errx*interpfactor(i);
            intermcells_m(primind, i,2) = intermcells_m(primind, i,2) + erry*interpfactor(i);
        end;
    end
end

function d = nonzero_min(x,y)
    if (x == 0)
        d = y;
    elseif (y == 0)
        d = x;
    else
        d = min(x, y);
    end;
end

function[x1,y1,theta1] = rotate(x,y,theta,theta0)
	x1 = x * cos(theta0) - y * sin(theta0);
	y1 = x * sin(theta0) + y * cos(theta0);
    
    r = [cos(theta/2), 0.0, 0.0, sin(theta/2)];
	q = [cos(theta0/2), 0.0, 0.0, sin(theta0/2)];
	
	f(1) = (r(1) * q(1)) - (r(2) * q(2)) - (r(3) * q(3)) - (r(4) * q(4));
	f(2) = (r(1) * q(2)) + (r(2) * q(1)) - (r(3) * q(4)) + (r(4) * q(3));
	f(3) = (r(1) * q(3)) + (r(2) * q(4)) + (r(3) * q(1)) - (r(4) * q(2));
	f(4) = (r(1) * q(4)) - (r(2) * q(3)) + (r(3) * q(2)) + (r(4) * q(1));
    theta1 = 2*atan2(f(4), f(1));
end

function[x1,y1,theta1] = rotate_clockwise(x,y,theta,theta0)
	   
    r = [cos(theta/2), 0.0, 0.0, sin(theta/2)];
	q = [cos(theta0/2), 0.0, 0.0, sin(theta0/2)];
    q(4) = -q(4);
    theta00 = 2*atan2(q(4), q(1));
    
    x1 = x * cos(theta00) - y * sin(theta00);
	y1 = x * sin(theta00) + y * cos(theta00);
    	
	f(1) = (r(1) * q(1)) - (r(2) * q(2)) - (r(3) * q(3)) - (r(4) * q(4));
	f(2) = (r(1) * q(2)) + (r(2) * q(1)) - (r(3) * q(4)) + (r(4) * q(3));
	f(3) = (r(1) * q(3)) + (r(2) * q(4)) + (r(3) * q(1)) - (r(4) * q(2));
	f(4) = (r(1) * q(4)) - (r(2) * q(3)) + (r(3) * q(2)) + (r(4) * q(1));
    theta1 = 2*atan2(f(4), f(1));
end

function[x1,y1,theta1] = add_origin_and_turn(x,y,theta, x0,y0,theta0)
	[x1,y1,theta1] = rotate(x,y,theta,theta0);
	x1 = x1 + x0;
	y1 = y1 + y0;
end

function[x1,y1,theta1] = remove_origin_and_turn(x,y,theta, x0,y0,theta0)
   	x2 = x - x0;
	y2 = y - y0;
    [x1,y1,theta1] = rotate_clockwise(x2,y2,theta,theta0);
end

function[x2,y2,theta2] = transform(x0, y0, theta0, start_theta)
    [x1,y1,theta1] = add_origin_and_turn(1.0,0.0,0.0, x0,y0,theta0);
    x2 = x1 - cos(start_theta);
    y2 = y1 - sin(start_theta);
    theta2 = theta1;
end

function[outfilename] = generate_file_name(base_name, res, rmin_m, errmin)
    res_s = num2str(res, 2);
    res_s = strrep(res_s, '.', '');
    rmin_s = num2str(rmin_m, 2);
    rmin_s = strrep(rmin_s, '.', '');
    err_s = num2str(errmin, 2);
    err_s = strrep(err_s, '.', '');
    outfilename1 = strcat(base_name, 'res');
    outfilename2 = strcat(outfilename1, res_s);
    outfilename1 = strcat(outfilename2, '_rad');
    outfilename2 = strcat(outfilename1, rmin_s);
    outfilename1 = strcat(outfilename2, '_err');
    outfilename2 = strcat(outfilename1, err_s);
    outfilename = strcat(outfilename2, '.mprim');
end

end