% HSV at Robot Cell
function [color_rgb_hi,color_rgb_low] = HSV_Iterator(counter)
    
    switch(counter)        
        case(1)
            % Threshold for RBG - RED
            color_rgb_low = [93,0,0];
            color_rgb_hi = [155,74,100];
        case(2)
            % Threshold for RBG - GREEN
            color_rgb_low = [0,83,0];
            color_rgb_hi = [100,140,115];
        case(3)
            % Threshold for RBG - BLUE
            color_rgb_low = [0,40,100];
            color_rgb_hi = [90,95,255];        
        case(4)
            % Threshold for RBG - YELLOW
            color_rgb_low = [105,160,12];
            color_rgb_hi = [255,255, 150];
        otherwise
            color_rgb_hi = [0,0,0];
            color_rgb_low = [0,0,0];  
    end
end