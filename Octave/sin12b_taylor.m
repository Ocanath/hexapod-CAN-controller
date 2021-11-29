function y = sin12b_taylor(theta)
    assert(isa(theta,'int32'),'Error! input must be type int32');
    
    % defines. 
    HALF_PI_12B                 =   int32(pi/2*2^12);  %int32(1.57079632679*4096);
    THREE_BY_TWO_PI_12B         =   int32(3*pi/2*2^12);
    PI_12B                      =   int32(pi*2^12);    %int32(3.14159265359*4096);
    TWO_PI_12B                  =   int32(2*pi*2^12);   %int32(6.28318530718*4096);
    
    ONE_BY_PI_15B               =   int32( (1/pi) * 2^15 );
    
    theta = mod(theta+PI_12B, TWO_PI_12B) - PI_12B;
    
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
    end    
    % This stage is where we compute n terms of the taylor series.
    %
    % This is meant to be more accurate while still operating in 32 bit
    % space. 
    %
    % The idea behind this section:
    %   1. We normalize our -4096*pi to 4096*pi to -1 to 1, 15bit. (15 is
    %   so we can do 2^15*2^15 without overflowing a 32 bit)
    %   
    %   2. We compute order constrained from 0-1. The normalization step
    %   guarantees that this will shrink to 0 the higher terms we go,
    %   instead of growing exponentially
    
    %   3. We pos-multiply our terms by pi^term_order, which we
    %   pre-compute. 
    
    %   Note: an input angle of 4096*pi will mean the resulting computes
    %   are all 1. similarly -4096*pi will alternate 1 and -1. 
    %   We will *have* to leverage this fact, and the fact that any value
    %   less decays to zero after some number of terms,
    %   because multiplying pi^n by 2^15 overflows 32bit at order 9
    %      
%     PI_N2 = int32(pi^2*4096);

    C_N3 = int32( (pi)^3/factorial(3)*4096);
    C_N5 = int32( (pi)^5/factorial(5)*4096);
    C_N7 = int32( (pi)^7/factorial(7)*4096);
    C_N9 = int32( (pi)^9/factorial(9)*4096);
    C_N11 = int32( (pi)^11/factorial(11)*4096);
    C_N13 = int32( (pi)^13/factorial(13)*4096);
    

    theta_15b_norm = bitshift(theta*ONE_BY_PI_15B, -12);
    theta2 = bitshift(theta_15b_norm*theta_15b_norm, -15);
    
    theta3 = bitshift(theta2*theta_15b_norm,-15);
    theta5 = bitshift(theta2*theta3,-15);
    theta7 = bitshift(theta2*theta5,-15);
    theta9 = bitshift(theta2*theta7,-15);
    theta11 = bitshift(theta2*theta9,-15);  %max value 16. ususally 0
    theta13 = bitshift(theta2*theta11,-15); %max value is 4. usually 0
    
    %this is getting ridiculous. Should do a lookup table. 
    res = theta*8 - bitshift(theta3*C_N3,-12);
    res = res + bitshift(theta5*C_N5,-12);
    res = res - bitshift(theta7*C_N7,-12);
    res = res + bitshift(theta9*C_N9,-12);
    res = res - bitshift(theta11*C_N11,-12);
    res = res + bitshift(theta13*C_N13,-12);
    
%     res = bitshift(res,-3); %to convert from 15bit to 12bit

    if(is_neg == 1)
		y = -res;
	else
		y = res;
    end    
end
