function y = sin_Nb64_taylor(theta, nout)
    assert(isa(theta,'int32'),'Error! input must be type int32');
    
    % defines. 
    HALF_PI_12B                 =   int32(pi/2*2^12);  %int32(1.57079632679*4096);
    THREE_BY_TWO_PI_12B         =   int32(3*pi/2*2^12);
    PI_12B                      =   int32(pi*2^12);    %int32(3.14159265359*4096);
    TWO_PI_12B                  =   int32(2*pi*2^12);   %int32(6.28318530718*4096);
        
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

    ncoef = 31;
    C_N3 = int64( (pi)^3/factorial(3)*2^ncoef);
    C_N5 = int64( (pi)^5/factorial(5)*2^ncoef);
    C_N7 = int64( (pi)^7/factorial(7)*2^ncoef);
    C_N9 = int64( (pi)^9/factorial(9)*2^ncoef);
    C_N11 = int64( (pi)^11/factorial(11)*2^ncoef);
%     C_N13 = int64( (pi)^13/factorial(13)*2^ncoef);
%     C_N15 = int64( (pi)^15/factorial(15)*2^ncoef); 
%     C_N17 = int64( (pi)^17/factorial(17)*2^ncoef); 
%     C_N19 = int64( (pi)^17/factorial(19)*2^ncoef); % next term is 1, decays to 0 after.
    ONE_BY_PI_31B               =   int64( (1/pi) * 2^31 );
    
    theta_31b_norm = bitshift(int64(theta)*ONE_BY_PI_31B, -12);    %remove the radix12 present in the input. result radix15
    theta2 = bitshift(theta_31b_norm*theta_31b_norm, -31);  %radix31 result
    
    theta3 = bitshift(theta2*theta_31b_norm,-31);
    theta5 = bitshift(theta2*theta3,-31);
    theta7 = bitshift(theta2*theta5,-31);
    theta9 = bitshift(theta2*theta7,-31);
    theta11 = bitshift(theta2*theta9,-31);  %max value 16. ususally 0
%     theta13 = bitshift(theta2*theta11,-31); %max value is 4. usually 0
%     theta15 = bitshift(theta2*theta13,-31); %max value is 4. must be 0 like feraking alwasy
%     theta17 = bitshift(theta2*theta15,-31); %max value is 4. must be 0 like feraking alwasy
%     theta19 = bitshift(theta2*theta17,-31); %max value is 4. nearly always 0

    lshift = nout-(31+ncoef);
    %this is getting ridiculous. Should do a lookup table. 
    res = int64(theta)*2^(nout-12) - bitshift(theta3*C_N3, lshift);    %theta is radix12, we need to convert to radix_nout
    res = res + bitshift(theta5*C_N5, lshift);
    res = res - bitshift(theta7*C_N7, lshift);
    res = res + bitshift(theta9*C_N9, lshift);
    res = res - bitshift(theta11*C_N11, lshift);
%     res = res + bitshift(theta13*C_N13, lshift);
%     res = res - bitshift(theta15*C_N15, lshift);
%     res = res + bitshift(theta17*C_N17, lshift);
%     res = res - bitshift(theta19*C_N19, lshift);
    
%     res = int32(res);   %restrict 30 bit

    if(is_neg == 1)
		y = -res;
	else
		y = res;
    end    
    
    %for kinematics, on the dh_hex, there is no tangible benefit to any
    %order beyond 11
end
