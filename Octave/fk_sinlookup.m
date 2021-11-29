function m = fk_sinlookup(hb_0, q, links, sin_order)
    assert(isa(hb_0,'int32')&isa(q,'int32'),'Err: inputs must all be int32');
    assert(size(q,2)==size(links,2), 'Error: num dofs must equal num links');
    n = size(q,2);
    m = hb_0;
    for i = 1:n
%       m = m*Hz_sinpoly(q(i))*links{i};  
        tmp = htmatrix_mult_fixed(Hz_sinlookup(q(i),sin_order),links{i}, sin_order);
        m = htmatrix_mult_fixed(m,tmp, sin_order);
    end
end