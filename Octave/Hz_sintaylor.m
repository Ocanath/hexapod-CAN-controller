function m = Hz_sintaylor(angle, n)
    assert(isa(angle,'int32'), 'Error: input type must be int32');
    ONE_FIXED = int32(1*2^n);
    m = [
            cos_Nb64_taylor(angle,n) -sin_Nb64_taylor(angle,n) 0 0 ; 
            sin_Nb64_taylor(angle,n) cos_Nb64_taylor(angle,n) 0 0 ;
            0 0 ONE_FIXED 0 ;
             0 0 0 ONE_FIXED; 
        ];
end