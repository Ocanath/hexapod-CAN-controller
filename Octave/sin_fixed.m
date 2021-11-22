% INPUT: angle, fixed point. Must be constrained from -4096*pi to 4096*pi
% externally, or else the internal constraint must be applied.
%
% OUTPUT: sin theta. Range is -4096 to 4096
%
function y = sin_fixed(theta)
    assert(isa(theta,'int32'),'Error! input must be type int32');
    
    % defines. 
    HALF_PI                 =   int32(6434);  %int32(1.57079632679*4096);
    THREE_BY_TWO_PI         =   int32(19302);
    PI                      =   int32(12868);    %int32(3.14159265359*4096);
    TWO_PI                  =   int32(25736);   %int32(6.28318530718*4096);
    
%     theta = mod(theta+PI,TWO_PI)-PI;
    
    % Begin
    is_neg = 0;
    if(theta > HALF_PI && theta <= PI)	% if positive and in quadrant II, put in quadrant I (same)
		theta = PI - theta;
    elseif (theta >= PI && theta < THREE_BY_TWO_PI)
        is_neg = 1;
        theta = theta - PI;
    elseif (theta > THREE_BY_TWO_PI && theta < TWO_PI)
        theta = theta - TWO_PI;
    elseif (theta < -HALF_PI && theta >= -PI ) % if negative and in quadrant III,
		is_neg = 1;
		theta = theta+PI;
    elseif (theta < 0 && theta >= -HALF_PI) %necessary addition for 4th order asymmetry
        is_neg = 1;
        theta = -theta;
    end
    
    %5 fixed point multiply
% 	theta_2 = bitshift(theta*theta,-12);
% 	theta_3 = bitshift(theta_2*theta,-12);
% 	theta_5 = bitshift(theta_3*theta_2,-12);
% 	res = bitshift(theta,12)-theta_3*ONE_BY_THREE_FACTORIAL + theta_5 * ONE_BY_FIVE_FACTORIAL;
% 	res = bitshift(res,-12);

    %7 fixed point multiply
    c = int32([117,-835,86,4077,1]);
    theta2 = bitshift(theta*theta,-12);
    theta3 = bitshift(theta2*theta,-12);
    theta4 = bitshift(theta3*theta,-12);
    res = c(1)*theta4 + c(2)*theta3 + c(3)*theta2 + c(4)*theta + bitshift(c(5),12);
    res = bitshift(res,-12);
    
    %trade 2 multiplies and an if statement for order of magnitude reduction in error...
    
    if(is_neg == 1)
		y = -res;
	else
		y = res;
    end    
end