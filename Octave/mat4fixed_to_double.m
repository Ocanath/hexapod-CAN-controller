function m = mat4fixed_to_double(in, translation_order_n)
    assert(isa(in,'int32'),'Error: input type must be int32');
    assert(min(size(zeros(4,4)) == [4, 4]),'Error: input must be homogeneous transofmration matrix');
    assert(in(4,1) == 0 & in(4,2) == 0 & in(4,3) == 0 & in(4,4) == 4096, 'Error: input must be homogeneous transformation matrix');

    in = double(in);    % cast as a double!
    m = zeros(4,4);
    m(1:3,1:3) = in(1:3,1:3)/4096;
    m(1:3,4) = in(1:3,4)/(2^translation_order_n);
    m(4,4) = in(4,4)/4096;
end