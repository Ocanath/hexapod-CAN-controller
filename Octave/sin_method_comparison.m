%% 

theta = -pi:.001:pi;
theta = round(theta*4096)/4096;
theta12 = int32(theta*4096);


s_lk = int32(zeros(1,length(theta12)));
s_tay64 = int32(zeros(1,length(theta12)));
s_tay32 = int32(zeros(1,length(theta12)));
for i = 1:length(theta12)
    s_lk(i) = sin_lookup(theta12(i),30);
    s_tay64(i) = sin_Nb64_taylor(theta12(i),30);
    s_tay32(i) = sin_Nb_taylor(theta12(i),30);
end

s_ctl = sin(theta);
s_lk = double(s_lk)/2^30;
s_tay64 = double(s_tay64)/2^30;
s_tay32 = double(s_tay32)/2^30;

figure(2)
clf
hold on
plot(s_ctl);
plot(s_lk);
plot(s_tay64);
plot(s_tay32);
hold off

err_tay64 = s_ctl - s_tay64;
err_lk = s_ctl - s_lk;
err_tay32 = s_ctl - s_tay32;

figure(1)
clf
hold on
plot(err_lk)
plot(err_tay64)
plot(err_tay32)
hold off

% max(abs(err_lk-err_tay))
maxlk_tay = [max(abs(err_lk)) max(abs(err_tay64))]
