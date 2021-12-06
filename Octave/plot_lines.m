function [saved_data, time] = plot_lines(num_lines, type, baud, buf_width, limit_mode, ylowerlim, yupperlim)

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
for i = 1:num_lines
    anim_line(i) = animatedline;
end

anim_line(1) = animatedline('Color',[.5 0 0] );
if(num_lines >= 3)
    anim_line(2) = animatedline('Color',[0 .5 0]);
    anim_line(3) = animatedline('Color', [0,0,.5] );
end

grid on;
tic;

linebuf = zeros(num_lines,buf_width);
timebuf = zeros(1,buf_width);

ylim([ylowerlim,yupperlim]);
xlim([0,buf_width]);
tic;

xidx = 0;
while(ishandle(H))
    
    data = read(s,num_lines,type);
    
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

clear s;

end
