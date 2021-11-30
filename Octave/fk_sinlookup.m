function m = fk_sinlookup(hb_0, q, links, sin_order)
    assert(isa(hb_0,'int32')&isa(q,'int32'),'Err: inputs must all be int32');
    assert(size(q,2)==size(links,2), 'Error: num dofs must equal num links');
    n = size(q,2);
    m = hb_0;
    for i = 1:n
        tmp = ht_mult_64b(Hz_sinlookup(q(i),sin_order),links{i}, sin_order);
        m = ht_mult_64b(m,tmp, sin_order);
%         m = zeros(4,4);
    end
end