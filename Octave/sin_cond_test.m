function theta = sin_cond_test(theta)
    assert(isa(theta,'int32'),'Error! input must be type int32');
    
    % defines. 
    HALF_PI_12B                 =   int32(pi/2*2^12);  %int32(1.57079632679*4096);
    THREE_BY_TWO_PI_12B         =   int32(3*pi/2*2^12);
    PI_12B                      =   int32(pi*2^12);    %int32(3.14159265359*4096);
    TWO_PI_12B                  =   int32(2*pi*2^12);   %int32(6.28318530718*4096);
    
    ONE_BY_PI_15B               =   int32( (1/pi) * 2^15 );
    
    % Begin
    is_neg = 0;
    if(theta > HALF_PI_12B && theta <= PI_12B)	% if positive and in quadrant II, put in quadrant I (same)
		theta = PI_12B - theta;
    elseif (theta >= PI_12B && theta < THREE_BY_TWO_PI_12B)
        is_neg = 1;
        theta = theta - PI_12B;
    elseif (theta > THREE_BY_TWO_PI_12B && theta < TWO_PI_12B)
        theta = theta - TWO_PI_12B;
    elseif (theta < -HALF_PI_12B && theta >= -PI_12B ) % if negative and in quadrant III,
		is_neg = 1;
		theta = theta+PI_12B;
    elseif (theta < 0 && theta >= -HALF_PI_12B) %necessary addition for 4th order asymmetry
        is_neg = 1;
        theta = -theta;
    end
end