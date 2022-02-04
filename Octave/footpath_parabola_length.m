function len = footpath_parabola_length(w,h)
    len = footpath_integral_closedform(w,h,1)-footpath_integral_closedform(w,h,-1);
end