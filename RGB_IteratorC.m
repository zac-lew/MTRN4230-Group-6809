% RGB at Conveyor
function [color_rgb_hi,color_rgb_low] = RGB_IteratorC(counter)
    if (counter == 1)
        % Threshold for RGB - RED
        color_rgb_low = [96,0,0];
        color_rgb_hi = [255,70,70];
    end
    if (counter == 2)
        % Threshold for RGB - GREEN
        color_rgb_low = [0,0,0];
        color_rgb_hi = [45,255,131];
    end
    if (counter == 3)
        % Threshold for RGB - BLUE
        color_rgb_low = [0,0,150];
        color_rgb_hi = [130,171,255];        
    end
    if (counter == 4)
        % Threshold for RGB - YELLOW
        color_rgb_low = [0,147,0];
        color_rgb_hi = [255,255,86];
    end
end