%%
HALF_PI_12B = int32(4096*pi/2);

% GENERATE THE LOOKUP TABLE
theta_scan = double(0+1:1:HALF_PI_12B+1);
lookup_quadrant1 = (2^30*sin( double(theta_scan)/4096 ));    % this yields max error in quadrants 2 and 3, 0 err in quadrants 1 and 4
%     theta_scan = double(-PI_12B+1  :1:    -HALF_PI_12B+1);
%     lookup_quadrant3 = (2^30* sin(theta_scan/4096));    %when used, this yields the same max error as lookup quad 1, except in quadrants 4 and 1 vs 2 and 3
%     lookup = int32( (lookup_quadrant1-lookup_quadrant3) /2);
%     lookup = int32(-lookup_quadrant3);
lookup = int32(lookup_quadrant1);   

fileID = fopen('sin_lookup.c','w');
fprintf(fileID,"int32_t lookup_sin_30bit[%d] = {\n",length(lookup));
for i = 1:length(lookup)
    fprintf(fileID,"    %d,\n",lookup(i));
end
fprintf(fileID,"};\n");
fclose(fileID);

