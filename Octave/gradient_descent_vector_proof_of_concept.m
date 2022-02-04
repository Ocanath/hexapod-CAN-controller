%%

theta = 35*pi/180;
vq = [cos(theta),sin(theta),0];
z = [0 0 1];

tau = pi/6+.2;
tauref = [cos(theta+tau),sin(theta+tau),0];

tangent = cross(z,vq);
vq_step = vq+tangent*tau;
vq_new = vq_step/sqrt(dot(vq_step,vq_step));

figure(1)
clf
hold on
t = -pi:.001:pi;
x = cos(t);
y = sin(t);
plot(x,y);
plot_2points([0,0],vq(1:2));
plot_2points(vq(1:2),vq_step(1:2));
plot_2points([0,0],vq_new(1:2));
plot_2points([0,0],tauref(1:2));
hold off
xlim([-1.5,1.5])
ylim([-1.5,1.5])

legend('','vq','vq step','vq_new','sums');
axis equal

clear all