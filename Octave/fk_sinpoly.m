function m = fk_sinpoly(hb_0, q, links)
    assert(isa(hb_0,'int32')&isa(q,'int32'),'Err: inputs must all be int32');
    assert(size(q,2)==size(links,2), 'Error: num dofs must equal num links');
    n = size(q,2);
    m = hb_0;
    for i = 1:n
%       m = m*Hz_sinpoly(q(i))*links{i};  
        tmp = htmatrix_mult_fixed(Hz_sinpoly(q(i)),links{i}, 12);
        m = htmatrix_mult_fixed(m,tmp, 12);
    end
end