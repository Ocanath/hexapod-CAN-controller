%% pathlength
w =15;
h= 5;

%compute xy path
t = -1:.001:1;
x = -(w/2)*t;
y = h*(-t.^2+1);

%compute path length by taking 
pathlength = 0;
vprev = [x(1) y(1)];
for i = 2:length(t)
    v = [x(i) y(i)];
    vd = (v-vprev);
    vprev = v;
    
    mag = sqrt(dot(vd,vd));
    pathlength = pathlength + mag;    %we can sum the magnitudes directly without factors of dt
    %because the integral and derivative are both wrt. dt, and are inverse
    %operations
end


%get closed form monstrosity produced by wolfram alpha
pl2 = footpath_integral_closedform(w,h,1)-footpath_integral_closedform(w,h,-1);

plot(x,y);
axis equal

% disp(pathlength);
% disp(pl2);
disp(pathlength-pl2)
% clear x y w h t
