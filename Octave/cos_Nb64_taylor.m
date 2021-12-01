function y = cos_Nb64_taylor(theta, nout)
    HALF_PI                 =   int32(6434);  %int32(1.57079632679*4096);
    y = sin_Nb64_taylor(theta+HALF_PI, nout);
end