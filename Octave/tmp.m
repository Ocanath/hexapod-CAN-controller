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


