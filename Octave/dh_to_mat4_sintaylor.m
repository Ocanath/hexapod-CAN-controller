function links = dh_to_mat4_sintaylor(d,a,alpha, sin_output_order)
    assert(isa(d,'int32')&&isa(a,'int32')&&isa(alpha,'int32'),'Error: input types must be int32');

    nd = size(d,2);
    na = size(a,2);
    nal = size(alpha,2);
    assert(nd == na & na == nal, 'Error! size mismatch for input arrays');
    n = nd;

    ONE_FIXED = int32(1*2^sin_output_order);
    links = cell(1,n);
    for i = 1:n
        m = [
            ONE_FIXED 0 0 a(i);
            0 int32(cos_Nb64_taylor(alpha(i), sin_output_order)) int32(-sin_Nb64_taylor(alpha(i),sin_output_order)) 0;
            0 int32(sin_Nb64_taylor(alpha(i),sin_output_order)) int32(cos_Nb64_taylor(alpha(i),sin_output_order)) d(i);
            0 0 0 ONE_FIXED;
        ];
        links{i} = int32(m);
    end     
    
end