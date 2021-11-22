function ret = htmatrix_mult(m1, m2)
    assert(isa(m1,'int32')&&isa(m2,'int32'),"Error: inputs must be 32 bit");
    
    n = 12; % binary decimal place of sin and cos
    
    ONE_N = int32(2^n);
    ret = [ 
            ONE_N 0 0 0; 
            0 ONE_N 0 0; 
            0 0 ONE_N 0; 
            0 0 0 ONE_N;
    ];
    
    for r = 0+1:3
        for c = 0+1:3
            tmp = int32(0);
            for i = 0+1:4
                tmp = tmp + m1(r,i)*m2(i,c);
            end
            ret(r,c) = bitshift(tmp,-n);
        end
    end
    
    for r = 0+1:4
        tmp = int32(0);
        for i = 0+1:4
            tmp = tmp + m1(r,i)*m2(i,3+1);
        end
        ret(r,3+1) = bitshift(tmp,-n);
    end
    
end
