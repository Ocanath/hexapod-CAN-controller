% variation on htmatmult which uses int64 buffer calculation to prevent overload
% on the translation parts of the htmat.
function ret = ht_mult_64b(m1, m2, n)
    assert(isa(m1,'int32')&&isa(m2,'int32'),"Error: inputs must be 32 bit");
    
%     n = 12; % binary decimal place of sin and cos
    
    ONE_N = int32(2^n);
    ret = [ 
            ONE_N 0 0 0; 
            0 ONE_N 0 0; 
            0 0 ONE_N 0; 
            0 0 0 ONE_N;
    ];
    
    %HEY
    %so the row and column vectors of all possible rotation matrices
    % are all unit vectors. 
    % that means the bound of the sum tmp is sqrt(3)*ONE_N, meaning
    % tmp will overflow N by 1 bit
    % so, we're gonna move radix of the sum with 1 bitshift per loop
    % iteration
    % then move the radix back to n all at once at the end
    if(n >= 30)
        for r = 0+1:3
            for c = 0+1:3
                tmp = int64(0);
                for i = 0+1:4   
                    m64 = int64(m1(r,i))*int64(m2(i,c));
                    tmp = tmp + bitshift(m64,-1);
                end
                ret(r,c) = int32(bitshift(tmp,-(n-1)));
            end
        end

        for r = 0+1:4
            tmp = int64(0);
            for i = 0+1:4
                m64 = int64(m1(r,i))*int64(m2(i,3+1));
                tmp = tmp + bitshift(m64,-1);
            end
            ret(r,3+1) = int32(bitshift(tmp,-(n-1)));
        end
    else
        for r = 0+1:3
            for c = 0+1:3
                tmp = int64(0);
                for i = 0+1:4   
                    tmp = tmp + int64(m1(r,i))*int64(m2(i,c));
                end
                ret(r,c) = int32(bitshift(tmp,-n));
            end
        end

        for r = 0+1:4
            tmp = int64(0);
            for i = 0+1:4
                tmp = tmp + int64(m1(r,i))*int64(m2(i,3+1));
            end
            ret(r,3+1) = int32(bitshift(tmp,-n));
        end
    end
end
