

%% Eval Accuracy of different methods
%setup float
q = [pi/4, pi/4, pi/4];
hb_0 = [ 
    -1 0 0 109.7858; 
    0 1 0 0; 
    0 0 -1 0;
    0 0 0 1
    ];
d = [65.66,29.00,21.5];
a = [-53.2, -100.46602344, -198.31677025];
alpha = [pi/2,pi,0];
%setup fixed
q_12b = int32(q*4096);
translation_order_n = 7;
hb_0_12b = int32(4096*hb_0);
hb_0_12b(1:3,4) = int32(hb_0(1:3,4)*2^translation_order_n);
d_fixed = int32(d*2^translation_order_n);
a_fixed = int32(a*2^translation_order_n);
alpha12 = int32(alpha*4096);


% FK true
links = dh_to_mat4(d,a,alpha);
hb_2_TRUE = fk(hb_0,q,links);
o3_b_true = hb_2_TRUE(1:3,4)

% FK sinpoly
links_sinpoly = dh_to_mat4_sinpoly(d_fixed,a_fixed,alpha12);
hb_2_sinpoly = fk_sinpoly(hb_0_12b,q_12b,links_sinpoly);
o3_b_sinpoly = double(hb_2_sinpoly(1:3,4))/2^translation_order_n

% FK lookup
sin_order = 15;
links_lkp = dh_to_mat4_sinlookup(d_fixed,a_fixed,alpha12,sin_order);
hb_0_nB = hb_0_12b;
hb_0_nB(1:3,1:3) = hb_0_12b(1:3,1:3)*2^(sin_order-12);
hb_2_lkp = fk_sinlookup(hb_0_nB,q_12b,links_lkp,sin_order);
o3_b_lkp = double(hb_2_lkp(1:3,4))/2^translation_order_n

%% Various sin methods comparison!

HALF_PI_12B = int32(4096*pi);
theta = -HALF_PI_12B:10:HALF_PI_12B;

sth_ctl = sin(double(theta)/4096);

sth_lookup = zeros(1, length(theta));
for i = 1:length(theta)
    sth_lookup(i) = sin_lookup(theta(i),30);
end
sth_lookup = double(sth_lookup)/2^30;

sth_taylor = zeros(1, length(theta));
for i = 1:length(theta)
    sth_taylor(i) = sin12b_taylor(theta(i));
end
sth_taylor = double(sth_taylor)/2^15;

sth_poly = zeros(1, length(theta));
for i = 1:length(theta)
    sth_poly(i) = sin_fixed(theta(i));
end
sth_poly = double(sth_poly)/2^12;

figure(2)
plot(sth_taylor)
figure(3)
plot(sth_poly)

figure(1)
clf
hold on
plot(sth_ctl-sth_lookup);
plot(sth_ctl-sth_taylor);
plot(sth_ctl - sth_poly);
hold off

