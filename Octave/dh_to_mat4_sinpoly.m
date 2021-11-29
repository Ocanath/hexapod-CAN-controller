function links = dh_to_mat4_sinpoly(d,a,alpha)
    assert(isa(d,'int32')&&isa(a,'int32')&&isa(alpha,'int32'),'Error: input types must be int32');

    nd = size(d,2);
    na = size(a,2);
    nal = size(alpha,2);
    assert(nd == na & na == nal, 'Error! size mismatch for input arrays');
    n = nd;
    
    ONE_12B = int32(1*2^12);
    links = cell(1,n);
    for i = 1:n
        m = [
            ONE_12B 0 0 a(i);
            0 cos_fixed(alpha(i)) -sin_fixed(alpha(i)) 0;
            0 sin_fixed(alpha(i)) cos_fixed(alpha(i)) d(i);
            0 0 0 ONE_12B;
        ];
        links{i} = int32(m);
    end     
    
end