function print_c_code_linkframe_list(links)
    num_links = size(links,2);
    fprintf("static const mat4_32b_t links_def[%d] = {\n",num_links);
    for i = 1:num_links
        m = links{i};

        fprintf("    {\n");
        fprintf("        {    //link %d\n",i);
        
        for r = 1:3
            fprintf("            {%d, %d, %d, %d},\n",m(r,1), m(r,2), m(r,3), m(r,4));
        end
        r = 4;
        fprintf("            {%d, %d, %d, %d}\n",m(r,1), m(r,2), m(r,3), m(r,4));
        
        fprintf("        }\n");
        if(i < num_links)
            fprintf("    },\n");
        else
            fprintf("    }\n");
        end
        
    end
    fprintf("};\n");
end