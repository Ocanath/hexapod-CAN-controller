function x = cos_fixed(theta)
    HALF_PI                 =   int32(6434);  %int32(1.57079632679*4096);
    x = sin_fixed(theta+HALF_PI);
end