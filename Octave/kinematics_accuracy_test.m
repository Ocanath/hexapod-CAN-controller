%% Eval Accuracy of different methods

%Notes.
% translation order 7 and sin order 12 will 
% allow about 4meter sphere outside the robot's base frame for 32 bits.

% if 64 bit buffers are used, 12 and 29 can be used for 4km radius outside the
% robots base frame. The cost is 
% many 64 bit multiplies and shifts.

translation_order_n = 16;   % the radix which is used to convert mm to fixed point
sin_order = 21;             % radix used to represent sin-cos(rad12)

%setup float
q = [pi/4, pi/4, pi/4];
q = floor(q*4096)/4096;
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
hb_0_12b = int32(4096*hb_0);
hb_0_12b(1:3,4) = int32(hb_0(1:3,4)*2^translation_order_n);

d_fixed = int32(d*2^translation_order_n);
a_fixed = int32(a*2^translation_order_n);
alpha12 = int32(alpha*4096);

% Calculate Links
links = dh_to_mat4(d,a,alpha);

lowerorder = (translation_order_n - 7);
d7 = bitshift(d_fixed,-lowerorder);
a7 = bitshift(a_fixed,-lowerorder);
links_sinpoly = dh_to_mat4_sinpoly(d7,a7,alpha12);
hb_0_12brot_7btrans = hb_0_12b;
hb_0_12brot_7btrans(1:3,4) = bitshift(hb_0_12brot_7btrans(1:3,4),-lowerorder);

links_lkp = dh_to_mat4_sinlookup(d_fixed,a_fixed,alpha12,sin_order);
hb_0_nB = hb_0_12b;
hb_0_nB(1:3,1:3) = hb_0_12b(1:3,1:3)*2^(sin_order-12);

links_tay = dh_to_mat4_sintaylor(d_fixed,a_fixed,alpha12,sin_order);
% end calculate links

oerr = [];
efpos_true = [];
efpos_lkp = [];
efpos_lkp_fixed = [];
efpos_poly = [];
efpos_tay = [];
efpos_single = [];
qlist = [];
r = 10;
div = r;
listelem = 1;
for i = -0:0
    for j = -r:r
        for k = -r:r

            q_12b = int32(sweepq(listelem,:));
            listelem = listelem + 1;
            qlist = [qlist,q_12b'];
            
            q = double(q_12b)/4096;
%             q = [i*pi/div,j*pi/div,k*pi/div];
% %             q = [j*pi/r,0,k*pi/r];
% %             q = [j*pi/r,k*pi/r,0];
%             q = floor(q*4096)/4096;
%             q_12b = int32(q*4096);


            
            % FK true
            hb_2_TRUE = fk(hb_0,q,links);
            o3_b_true = hb_2_TRUE{3}(1:3,4);
            efpos_true = [efpos_true,o3_b_true];
            
            hb_2_single = fk_single(hb_0,q,links);
            efpos_single = [efpos_single,hb_2_single(1:3,4)];
            
%             % FK sinpoly
            hb_2_sinpoly = fk_sinpoly(hb_0_12brot_7btrans,q_12b,links_sinpoly);
            o3_b_sinpoly = double(hb_2_sinpoly(1:3,4))/2^7; %sinpoly has fixed translation order 7
            efpos_poly = [efpos_poly,o3_b_sinpoly];

            % FK lookup
            hb_2_lkp = fk_sinlookup(hb_0_nB,q_12b,links_lkp,sin_order);
            o3_b_lkp_fixed = hb_2_lkp(1:3,4);
            efpos_lkp_fixed = [efpos_lkp_fixed, o3_b_lkp_fixed];
            o3_b_lkp = double(o3_b_lkp_fixed)/2^translation_order_n;
            efpos_lkp = [efpos_lkp,o3_b_lkp];
            
            hb_2_tay = fk_taylor(hb_0_nB,q_12b, links_tay, sin_order);
            o3_b_tay = double(hb_2_tay(1:3,4))/2^translation_order_n;
            efpos_tay = [efpos_tay,o3_b_tay];

            errlkp = sqrt( sum (abs(o3_b_true - o3_b_lkp)).^2 );
            oerr = [oerr,errlkp];
        end
    end
end




figure(1)
plot(oerr)

figure(2)
clf 
hold on
plot3(efpos_true(1,:),efpos_true(2,:), efpos_true(3,:));
plot3(efpos_lkp(1,:),efpos_lkp(2,:), efpos_lkp(3,:));
plot3(efpos_poly(1,:),efpos_poly(2,:), efpos_poly(3,:));
plot3(efpos_tay(1,:),efpos_tay(2,:), efpos_tay(3,:));
plot3(efpos_single(1,:),efpos_single(2,:), efpos_single(3,:));
plot3(sweepfixed(:,1)/2^16,sweepfixed(:,2)/2^16,sweepfixed(:,3)/2^16);
hold off
axis vis3d
view([-41.3720,22.2014]);



figure(3)
clf
hold on

err_lkp = (efpos_true - efpos_lkp);
plot3(err_lkp(1,:),err_lkp(2,:), err_lkp(3,:));

err_sinpoly = (efpos_true - efpos_poly);
err_sinpoly = err_sinpoly / 20;
plot3(err_sinpoly(1,:),err_sinpoly(2,:), err_sinpoly(3,:));

err_sintay = (efpos_true - efpos_tay);
plot3(err_sintay(1,:),err_sintay(2,:), err_sintay(3,:));


errcpp = (floor(efpos_lkp*2^16) - sweepfixed')/2^16;
plot3(errcpp(1,:),errcpp(2,:), errcpp(3,:));


err_single = (efpos_true - efpos_single);
plot3(err_single(1,:),err_single(2,:), err_single(3,:));

hold off
maxmean = [max(oerr), mean(oerr)]
axis vis3d
view([8.5178,9.2187]);
efpos_lkp_fixed = efpos_lkp_fixed';

%%
for i = 1:length(sweepq)
    q_12b = int32(sweepq(i,:));
    
    hb_2_lkp = fk_sinlookup(hb_0_nB,q_12b,links_lkp,sin_order);
    o3_b_lkp_fixed = hb_2_lkp(1:3,4);
    
    o3_cpp = sweepfixed(i,:);
    
%     disp(double(o3_b_lkp_fixed));
%     disp(o3_cpp');
%     disp(int32(o3_cpp' ) - o3_b_lkp_fixed);
    error = int32(o3_cpp' ) - o3_b_lkp_fixed;
    
    if(max(error) > 0)
        disp(q_12b)
    end
end
%%
q = floor(4096*[pi/4,-pi/6,pi/4])/4096;
% q = [0,0,0];
hb_2_TRUE = fk(hb_0,q,links);
o3_b_true = hb_2_TRUE{1}(1:3,4)
disp(hb_2_TRUE{3})

% h0_1 = Hz(q(1))*links{1}
% h1_2 = Hz(q(2))*links{2}
% h2_3 = Hz(q(3))*links{3}
% 
% hb_1 = hb_0*h0_1
% hb_2 = hb_1*h1_2
% hb_3 = hb_2*h2_3


%%
varr = [];
sumarr = [];
for i = -10:10
    for j = -10:10
        for k = -10:10
            v = [i j k];
            norm = sqrt(sum(v.^2));
            v_norm = v./norm;
            varr = [varr,v_norm'];
            sumarr = [sumarr,sum(v_norm)];
        end
    end
end
figure(1)
plot3(varr(1,:), varr(2,:), varr(3,:));
figure(2)
plot(sumarr);

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
