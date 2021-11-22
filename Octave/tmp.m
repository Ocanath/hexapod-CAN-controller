%%


syms cth sth r00 r01 r02 r03 r10 r11 r12 r13 r20 r21 r22 r23



Rz = [ cth -sth 0 0; sth cth 0 0; 0 0 1 0; 0 0 0 1; ]
Rgeneric = [r00 r01 r02 r03; r10 r11 r12 r13; r20 r21 r22 r23 ; 0 0 0 1;]

res = Rz*Rgeneric

for r = 1:4
    for c = 1:4
       fprintf("him1_i->m[%d][%d] = %s;\n", r-1 ,c-1, res(r,c));
    end
end


%%

syms d a alpha
%%
n = 8;

v = 198.31677025;
v6b = int32(floor(v*2.^n));

vm = bitshift(int32(4096)*v6b, -n);


res = double(vm)/4096;
v - res


%% 
m1 = Hz(1.243);
m2 = Hy(-2.34);
m1(1:3,4) = [10.0 , 15.0 , -55.0];
m2(1:3,4) = [11,-140, 7];

translation_order_n = 7;

% Htest = htmatrix_mult(m1,m2);
m1_fixed = int32(4096*m1);
m1_fixed(1:3,4) = bitshift(m1_fixed(1:3,4), -(12-translation_order_n) ); %enforce 2^6 order translation distance. roughly bottom 2 decimals
m2_fixed = int32(4096*m2);
m2_fixed(1:3,4) = bitshift(m2_fixed(1:3,4), -(12-translation_order_n) ); %enforce 2^6 order translation distance. roughly bottom 2 decimals

Htest_fixedpoint = htmatrix_mult(m1_fixed,m2_fixed)
Htest = double(Htest_fixedpoint);
Htest(1:3,1:3) = Htest(1:3,1:3)/4096;
Htest(1:3,4) = Htest(1:3,4)/2^translation_order_n;
Htest(4,4) = Htest(4,4)/4096;


fprintf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
Htest
Hctl = m1*m2



% Htest-Hctl

%%
theta = [pi/4,-pi/4,pi/4];
dh_table = [
	65.66 	-53.2			pi/2;
    29.00	-100.46602344		pi;
    21.50,	-198.31677025,		0;
];
n = 7;
hlink_1 = dh_to_mat4_fixed(int32(dh1(1)*2^n),int32(dh1(2)*2^n),int32(dh1(3)*2^12));
h0_1_fx = htmatrix_mult(Hz_fixed(int32(4096*theta(1))),hlink_1);

h0_1 = mat4fixed_to_double(h0_1_fx,n)
% h0_1(1:3,4) = h0_1(1:3,4)*10

%% number of possible orders for a 12bit angle:
TWO_BY_PI_15B = int32((1/pi)*2^15);

theta = int32(pi*4096) -1;

theta_15b_norm = bitshift(TWO_BY_PI_15B*theta,-12);
theta_n = int32(2^15);  % i.e. 1
n = 0;
for n = 1:9
    theta_n = bitshift(theta_15b_norm*theta_n,-15);
%     n = n + 1;
end

