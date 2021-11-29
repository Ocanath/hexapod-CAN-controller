function m = fk(hb_0, q, links)
    assert(size(q,2)==size(links,2), 'Error: num dofs must equal num links');
    n = size(q,2);
    m = hb_0;
    for i = 1:n
      m = m*Hz(q(i))*links{i};  
    end
end