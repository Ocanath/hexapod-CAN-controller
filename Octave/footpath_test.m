%% test footpath gen
h = 3;
w = 2;

t = 0:.001:3.0;
v = zeros(3,length(t));
for i = 1:length(t)
    v(:,i) = footpath_gen(t(i),h,w,3);
end

v4 = zeros(4,length(t));
v4(1:3,:) = v;
v4(4,:) = 1;

startpos_foot = [-1 0 -1]';
vf = Hx(pi/2)*v4;
vf = Hz(-pi/2)*vf;
vf = vf(1:3,:) + startpos_foot;%translate



figure(1)
clf
hold on
plot3(t/max(t),0*t,0*t);
plot3(0*t,t/max(t),0*t);
plot3(0*t,0*t,t/max(t));
plot3(vf(1,:),vf(2,:),vf(3,:));
hold off
% ylim([-0.1-h,h+.1]);
% xlim([-w/2-.1,w/2+.1]);
legend('x','y','z','FOOT');
axis equal
%view it
az = -42.2277;
el = 23.1631;
view(az,el);