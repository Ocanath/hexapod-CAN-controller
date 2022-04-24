%%
w = 4;
h = 2;

t = -1:0.001:1;
x = -t*(w/2);
y = h*(-t.^2+1);

figure(1)
clf
hold on
plot(x,y);

x1 = -sin(t*pi/2)*(w/2);
x2 = -sin(t).*sin(t+pi/2)*2.199*(w/2);

plot(x2,y);
hold off
axis equal

% figure(2)
% plot(t,y);

figure(3)
clf
hold on
plot(t,x);
plot(t,x1);
plot(t,x2);
hold off
axis equal

