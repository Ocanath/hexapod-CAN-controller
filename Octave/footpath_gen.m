%returns the vector V in R^2 (3-vector with z=0) which
% is parameterized through the periodic function defined here
function v = footpath_gen(time, h, w, period)
    TWO_PI = 2*pi;
    t = mod(time*TWO_PI/period,TWO_PI)/(TWO_PI); %create normalized time
    
    %todo: change timestamps p1 and p2 (percentages) to normalize speed for each region
    parabola_length = footpath_integral_closedform(w,h,1)-footpath_integral_closedform(w,h,-1);  %parabola length.
    perc_step = w/(w+parabola_length);   %percent of total path length occupied by the step portion of the motion
    
    p1 = perc_step/2;
    p2 = 1-p1;
%     p1 = 1/3;
%     p2 = 2/3;
    
    if(t >= 0 && t < p1)
        t = (t - 0)/(p1 - 0);
        v = [
            t*(w/2)
            0
            0
        ];
    elseif(t >= p1 && t < p2)
        t = 2*(t-p1)/(p2-p1) - 1; %for this, t must go from -1 to 1
        v = [
            -t*w/2
            (-t.^2+1)*h
            0
        ];
    elseif(t >= p2 && t < 1)
        t = (t - p2)/(1-p2);
        v = [
            t*(w/2)-(w/2)
            0
            0
        ];
    else
        v = [0,0,0];    %should never happen
    end
end