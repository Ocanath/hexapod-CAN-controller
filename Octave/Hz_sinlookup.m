function m = Hz_sinlookup(angle, n)
    assert(isa(angle,'int32'), 'Error: input type must be int32');
    ONE_FIXED = int32(1*2^n);
    m = [
            cos_lookup(angle,n) -sin_lookup(angle,n) 0 0 ; 
            sin_lookup(angle,n) cos_lookup(angle,n) 0 0 ;
            0 0 ONE_FIXED 0 ;
             0 0 0 ONE_FIXED; 
        ];
end