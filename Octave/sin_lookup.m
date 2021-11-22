function out = sin_lookup(theta, result_radix)
    assert(isa(theta,'int32'),'Error! input must be type int32');
    
    % defines. 
    HALF_PI_12B                 =   int32(pi/2*2^12);  %int32(1.57079632679*4096);
    THREE_BY_TWO_PI_12B         =   int32(3*pi/2*2^12);
    PI_12B                      =   int32(pi*2^12);    %int32(3.14159265359*4096);
    TWO_PI_12B                  =   int32(2*pi*2^12);   %int32(6.28318530718*4096);
     
    theta = mod(theta+PI_12B,TWO_PI_12B)-PI_12B;
    
%     theta_scan = 1:1:double(HALF_PI_12B)+1;
%     lookup = int32(2^30*sin( double(theta_scan)/4096 ));    % logistically easier to generate the lookup table every time.
    % 30 is max possible scale for 32bit signed lookup table with base 2 radix conversion
    
    
    theta_scan = double(0+1:1:HALF_PI_12B+1);
    lookup_quadrant1 = (2^30*sin( double(theta_scan)/4096 ));    % this yields max error in quadrants 2 and 3, 0 err in quadrants 1 and 4
  
    theta_scan = double(-PI_12B+1  :1:    -HALF_PI_12B+1);
    lookup_quadrant3 = (2^30* sin(theta_scan/4096));    %when used, this yields the same max error as lookup quad 1, except in quadrants 4 and 1 vs 2 and 3
    
    lookup = int32( (lookup_quadrant1-lookup_quadrant3) /2);
%     lookup = int32(-lookup_quadrant3);
%     lookup = int32(lookup_quadrant1);

    
    % Begin
    is_neg = 0;
    if(theta > HALF_PI_12B && theta <= PI_12B)	% if positive and in quadrant II, put in quadrant I (same)
		theta = (PI_12B - theta);   % this has weird error dynamics, i don't know why
        
    elseif (theta < -HALF_PI_12B && theta >= -PI_12B ) % if negative and in quadrant III,
		is_neg = 1;
        theta = PI_12B+theta;

    elseif (theta < 0 && theta >= -HALF_PI_12B) %necessary addition for 4th order asymmetry
        is_neg = 1;
        theta = -theta; %this and the quadrant I case are 0 error
    end

    if(theta <= 0)  %%
        out = 0;
        return;
    end

    if(is_neg)
        out = -bitshift(lookup(theta-1   +1),-(30-result_radix));  %NOTE!!! for C-indexed (at 0) lookup tables, it is lookup(theta-1)
    else
        out =  bitshift(lookup(theta-1   +1),-(30-result_radix));
    end
    
end