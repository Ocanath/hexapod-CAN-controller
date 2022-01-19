%% test footpath gen
h = 3;
w = 2;

t = 0:.001:3.0;
v = zeros(3,length(t));
for i = 1:length(t)
    v(:,i) = footpath_gen(t(i),h,w,3);
end
figure(1)
clf
hold on
plot(v(1,:),v(2,:))
hold off
ylim([-0.1,h+.1]);
xlim([-w/2-.1,w/2+.1]);