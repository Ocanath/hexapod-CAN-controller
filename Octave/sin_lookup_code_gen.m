
%%
HALF_PI_12B = int32(4096*pi/2);


% theta = 0:1:double(HALF_PI_12B);
% lookup = int32(2^30*sin( double(theta)/4096 ));
theta_scan = 1:1:double(HALF_PI_12B)+1;
lookup = int32(2^30*sin( double(theta_scan)/4096 ));    % logistically easier to generate the lookup table every time.

fileID = fopen('sin_lookup.c','w');
fprintf(fileID,"int32_t lookup_sin_30bit[%d] = {\n",length(lookup));
for i = 1:length(lookup)
    fprintf(fileID,"    %d,\n",lookup(i));
end
fprintf(fileID,"};\n");
fclose(fileID);

%%
PI_12B = int32(4096*pi);
theta = -PI_12B:1:PI_12B;
theta_conditioned = zeros(1,length(theta));
for i = 1:length(theta)
    theta_conditioned(i) = sin_cond_test(int32(theta(i)));
end
figure(1)
clf
hold on
plot(theta);
plot(theta_conditioned)
hold off


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

err = sth_ctl-sth_lookup;
figure(1)
clf
hold on
plot(theta,err);
% plot(theta_ctl);
% plot(theta_lookup);
% plot(theta_taylor);
hold off


e3 = max(err)

%%
theta_1 = 0:.001:pi/2;
theta_2 = pi/2:.001:pi;
theta_3 = -pi:.001:-pi/2;
theta_4 = -pi/2:.001:0;
figure(2)
clf

subplot(2,2,1)
plot(theta_1/pi,sin(theta_1));

subplot(2,2,2)
hold on
plot(theta_2/pi, sin(theta_2));
plot((pi-theta_1)/pi, sin(pi-theta_1));
hold off

subplot(2,2,3)
hold on
plot(theta_3/pi,sin(theta_3));
plot( (theta_1+pi)/pi,-sin(theta_1+pi) );
hold off

subplot(2,2,4)
hold on
plot(theta_4/pi, sin(theta_4));
plot(-theta_1/pi, sin(-theta_1));
hold off



%%
HALF_PI_12B = int32(pi/2*4096);
PI_12B= int32(4096*pi);


theta_scan = 1:1:double(HALF_PI_12B)+1;
lookup_quadrant1 = int32(2^30*sin( double(theta_scan)/4096 ));    % logistically easier to generate the lookup table every time.

theta_scan = (-PI_12B-2  :1:    -HALF_PI_12B-2);
lookup_quadrant3 = int32(2^30* sin (double(PI_12B+theta_scan)/4096));
% lookup = (lookup_quadrant1 + lookup_quadrant3)/2;
lookups = [lookup_quadrant1',lookup_quadrant3'];

figure(3)
clf
hold on
plot(lookup_quadrant1)
plot(lookup_quadrant3)
hold off

theta_float = -pi+.1;
theta = int32(4096*theta_float);

PI_12B = int32(pi*4096);

% lkt = sin_lookup(theta,30);   
lkt_reconstr = -int32(sin(double(PI_12B+theta)/4096)*2^30);
shouldbe = int32(sin(double(theta)/4096)*2^30);



