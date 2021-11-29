function links = dh_to_mat4(d,a,alpha)
    nd = size(d,2);
    na = size(a,2);
    nal = size(alpha,2);
    assert(nd == na & na == nal, 'Error! size mismatch for input arrays');
    n = nd;
    
    links = cell(1,n);
    for i = 1:n
        m = [   %can easily interchange this with a urdf - style rpy+translation
                1 0 0 a(i);
                0 cos(alpha(i)) -sin(alpha(i)) 0;
                0 sin(alpha(i)) cos(alpha(i)) d(i);
                0 0 0 1;
            ];
        links{i} = m;
    end     
end