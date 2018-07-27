function [p_color] = ColorMap(p_field)
% set color map for p filed.

nel = length(p_field);
p_color = zeros(nel,3);
if (max(p_field) > -min(p_field))
    for ie = 1:nel
        if (p_field(ie) >= 0)
            if (p_field(ie) > max(p_field)/2)
                p_color(ie,:) = [0.0, (1-p_field(ie)/max(p_field))*2, 1.0];
            else
                p_color(ie,:) = [0.0, 1.0, p_field(ie)/max(p_field)/2];
            end
        else
            if (-p_field(ie) > max(p_field)/2)
                p_color(ie,:) = [1.0, (1+p_field(ie)/max(p_field))*2, 0.0];
            else
                p_color(ie,:) = [-p_field(ie)/max(p_field)/2, 1.0, 0.0];
            end
        end
    end
else
    for ie = 1:nel
        if (p_field(ie) >= 0)
            if (p_field(ie) > -min(p_field)/2)
                p_color(ie,:) = [0.0, (1+p_field(ie)/min(p_field))*2, 1.0];
            else
                p_color(ie,:) = [0.0, 1.0, -p_field(ie)/min(p_field)/2];
            end
        else
            if (-p_field(ie) > -min(p_field)/2)
                p_color(ie,:) = [1.0, (1-p_field(ie)/min(p_field))*2, 0.0];
            else
                p_color(ie,:) = [p_field(ie)/min(p_field)/2, 1.0, 0.0];
            end
        end
    end
end

end