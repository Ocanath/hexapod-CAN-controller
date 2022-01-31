%% Load inputs and outputs to compare against (csv)
n = 10;
% q_csv = [linspace(-pi/4,pi/4,n)' linspace(-pi/4,pi/4,n)' linspace(-pi/4,pi/4,n)'];
q_csv = [0 0 0];
%% Compute 'true' working double precision fk on the same input set
float_input_q = q_csv;



%setup output buffer
efpos_true = zeros(size(q_csv, 1));

%Setup float dh-hex params
q = [10*pi/180 10*pi/180 10*pi/180];
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
% Calculate Links
links = dh_to_mat4(d,a,alpha);

% FK true
hb_2_TRUE = fk(hb_0,q,links);
o3_b_true = hb_2_TRUE{3}(1:3,4)

%% Compute error and display it
