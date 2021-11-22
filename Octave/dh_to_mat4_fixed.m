function m = dh_to_mat4_fixed(d,a,alpha)
    assert(isa(d,'int32')&&isa(a,'int32')&&isa(alpha,'int32'),'Error: input types must be int32');
    ONE_12B = int32(1*2^12);
    m = [
            ONE_12B 0 0 a;
            0 cos_fixed(alpha) -sin_fixed(alpha) 0;
            0 sin_fixed(alpha) -sin_fixed(alpha) d;
            0 0 0 ONE_12B;
        ];
end