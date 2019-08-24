% HSV at Robot Cell
function [color_rgb_hi,color_rgb_low] = HSV_Iterator(counter)
    if (counter == 1)
        % Threshold for RBG - RED
        color_rgb_low = [93,0,0];
        color_rgb_hi = [155,74,100];
    end
    if (counter == 2)
        % Threshold for RBG - GREEN
        color_rgb_low = [0,83,0];
        color_rgb_hi = [100,140,115];
    end
    if (counter == 3)
        % Threshold for RBG - BLUE
        color_rgb_low = [0,40,100];
        color_rgb_hi = [90,95,255];        
    end
    if (counter == 4)
        % Threshold for RBG - YELLOW
        color_rgb_low = [105,160,12];
        color_rgb_hi = [255,255, 150];
    end
end