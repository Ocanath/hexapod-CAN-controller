function print_c_code_linkframe_list(links)
    for i = 1:size(links,2)
        m = links{i};
        fprintf("static const mat4_32b_t h32_link%d = {\n",i);
        fprintf("    {\n");
        for r = 1:3
            fprintf("        {%d, %d, %d, %d},\n",m(r,1), m(r,2), m(r,3), m(r,4));
        end
        r = 4;
        fprintf("        {%d, %d, %d, %d}\n",m(r,1), m(r,2), m(r,3), m(r,4));
        
        fprintf("    }\n");
        fprintf("};\n");
    end
end