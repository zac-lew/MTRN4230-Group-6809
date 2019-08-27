% HSV at Robot Cell
function [color_rgb_hi,color_rgb_low] = HSV_Iterator(counter)
    
    switch(counter)        
        case(1)
            % Threshold for RBG - RED
            color_rgb_low = [160,0,0];
            color_rgb_hi = [230,95,255];
        case(2)
            % Threshold for RBG - GREEN
            color_rgb_low = [60,86,65];
            color_rgb_hi = [80,125,125];
        case(3)
            % Threshold for RBG - BLUE
            color_rgb_low = [0,0,120];
            color_rgb_hi = [180,130,225];        
        case(4)
            % Threshold for RBG - YELLOW
            color_rgb_low = [30,145,0];
            color_rgb_hi = [160,255,140];
        otherwise
            color_rgb_hi = [0,0,0];
            color_rgb_low = [0,0,0];  
    end
end

