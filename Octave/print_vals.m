function [saved_data, time] = print_vals(num_lines, type, baud)

slist = serialportlist;
try
    s = serialport(slist(1), baud);
catch
    disp("No serial port found");
end

f = figure(1);
H = uicontrol();
H.Position = [0,0,0,0];
pause(.01);
tic;
while(ishandle(H))
    
    data = read(s,num_lines,type);
    
    if(isempty(data) ~= 1 && size(data,2) == num_lines)
        set(gcf,'Renderer','OpenGL');

        for i = 1:num_lines
            v = double(data(i));
            if(type == "single" || type == "double")
                fprintf("%f, ",v);
            else
                fprintf("%d, ",v);
            end
        end
        fprintf("\n");
    end
end

clear s;

end
