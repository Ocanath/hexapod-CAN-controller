function m = rpy_to_mat4(rpy, xyz)
    
    m = Hz(rpy(3)) * Hy(rpy(2)) * Hx(rpy(1));

end