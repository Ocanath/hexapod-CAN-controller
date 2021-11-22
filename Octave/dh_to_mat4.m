function m = dh_to_mat4(d,a,alpha)
    m = [
            1 0 0 a;
            0 cos(alpha) -sin(alpha) 0;
            0 sin(alpha) -sin(alpha) d;
            0 0 0 1;
        ];
end