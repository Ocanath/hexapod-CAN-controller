function x = cos_lookup(theta,result_radix)
    HALF_PI                 =   int32(6434);  %int32(1.57079632679*4096);
    x = sin_lookup(theta+HALF_PI,result_radix);
end