function plot2points(p1,p2)
    n = length(p1);
    X = [p1(1),p2(1)];
    Y = [p1(2),p2(2)];
    if n == 2
        plot(X,Y);
    elseif n == 3
        Z = [p1(3),p2(3)];
        plot3(X,Y,Z);
    end
end