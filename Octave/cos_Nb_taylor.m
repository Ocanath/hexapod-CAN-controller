function y = cos_Nb_taylor(theta, nout)
    HALF_PI                 =   int32(6434);  %int32(1.57079632679*4096);
    y = sin_Nb_taylor(theta+HALF_PI, nout);
end