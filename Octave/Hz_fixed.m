function m = Hz_fixed(angle)
    assert(isa(angle,'int32'), 'Error: input type must be int32');
    ONE_12B = int32(1*4096);
    m = [
            cos_fixed(angle) -sin_fixed(angle) 0 0 ; 
            sin_fixed(angle) cos_fixed(angle) 0 0 ;
            0 0 ONE_12B 0 ;
             0 0 0 ONE_12B; 
        ];
end