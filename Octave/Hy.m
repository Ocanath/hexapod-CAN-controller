function m = Hy(angle)
    m = [
            cos(angle) 0 sin(angle) 0; 
            0 1 0 0;
            -sin(angle) 0 cos(angle) 0;
            0 0 0 1;
        ];
end