function ret = footpath_integral_closedform(w, h, t)
    ww = w.^2;
    hh = h.^2;
    tt = t.^2;
    
    a = sqrt(16*hh*tt + ww);
    b = 4*h*t;
    
    tmp1 = a*b + ww*log(h*(a+b));
    ret = tmp1/(h*16);  %final step
end