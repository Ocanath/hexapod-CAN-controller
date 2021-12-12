function m_list = fk(hb_0, q, links)
    assert(size(q,2)==size(links,2), 'Error: num dofs must equal num links');
    n = size(q,2);
    m = hb_0;
    m_list = cell(1,n);
    for i = 1:n
      m = m*Hz(q(i))*links{i};  
      m_list{i} = m;
    end
end