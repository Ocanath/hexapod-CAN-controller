function m = fk_single(hb_0, q, links)
    assert(size(q,2)==size(links,2), 'Error: num dofs must equal num links');
    n = size(q,2);
    m = single(hb_0);
    for i = 1:n
%       m = m*single(Hz(single(q(i))))*single(links{i});  
        q12 = int32(q(i)*4096);
        rZ_single = single(Hz_sinlookup(q12,30))./single(2^30);
        m = m*rZ_single*single(links{i});
    end
end