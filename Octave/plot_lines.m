function [saved_data, time] = plot_lines(num_lines, type, baud, buf_width, limit_mode, ylowerlim, yupperlim)

slist = serialportlist;
try
    s = serialport(slist(1), baud);
    fprintf("found serial port: %s",slist(1));
catch
    disp("No serial port found");
end

f = figure(1);
H = uicontrol();
H.Position = [0,0,0,0];
pause(.01);

colorlist=[
    [0, 0.4470, 0.7410]
	[0.8500, 0.3250, 0.0980]
    [0.9290, 0.6940, 0.1250]
    [0.4940, 0.1840, 0.5560]
    [0.4660, 0.6740, 0.1880]
    [0.3010, 0.7450, 0.9330]
    [0.6350, 0.0780, 0.1840]
    [0, 0, 1]
    [0, 0.5, 0]
    [1, 0, 0]
    [0, 0.75, 0.75]
    [0.75, 0, 0.75]
    [0.75, 0.75, 0]
    [0.25, 0.25, 0.25]
];
for i = 1:num_lines
    if(i <= length(colorlist))
        anim_line(i) = animatedline('Color', colorlist(i,:), 'LineWidth', 1.0);
    elseif(i > length(colorlist))
       anim_line(i) = animatedline;
    end
end

%OPTIONAL: shows legend on plots
legend

grid on;
tic;

linebuf = zeros(num_lines,buf_width);
timebuf = zeros(1,buf_width);

ylim([ylowerlim,yupperlim]);
xlim([0,buf_width]);
tic;

saved_data = 0;
time = 0;

xidx = 0;
while(ishandle(H))
    
    try
        data = read(s, num_lines, type);
        
        %for aenc
%         data8 = read(s,5,'uint8');
%         data = typecast(uint8(data8(1:4)),'single');
    catch
        disp("read timeout or error");
        pause(0.33);
    end
    
    if(isempty(data) ~= 1 && size(data,2) == num_lines)
        set(gcf,'Renderer','OpenGL');

        xidx = xidx + 1;
        for i = 1:num_lines
            v = double(data(i));
            if(ishandle(anim_line(i)))
                addpoints(anim_line(i), xidx, v);
                linebuf(i,xidx) = v;
                if(limit_mode == "auto")
                    if(v > yupperlim)
                        yupperlim = v;
                        ylim([ylowerlim,yupperlim]);
                    elseif(v < ylowerlim)
                        ylowerlim = v;
                        ylim([ylowerlim,yupperlim]);
                    end
                end
                timebuf(xidx) = toc;
            end
        end

        drawnow limitrate nocallbacks
        
        if(xidx >= buf_width)
            for i = 1:num_lines
                clearpoints(anim_line(i));
            end
            xidx = 0;
            saved_data = linebuf;
            time = timebuf;
            linebuf = zeros(num_lines,buf_width);

%             linebuf(1:end-1) = linebuf(2:end);
%             addpoints(anim_line(i),1:xidx,linebuf(1:xidx));
%             xidx = buf_width - 1;
        end

        

%         timebuf(2:end) = timebuf(1:end-1);
%         timebuf(1) = toc;
    end
    
end
if(saved_data == 0)
    saved_data = linebuf;
    time = timebuf;
end

clear s;

end
