% RGB at Conveyor
function [color_rgb_hi,color_rgb_low] = RGB_IteratorC(counter)
    
    if (~isscalar(counter))
        color_rgb_hi = [0,0,0];
        color_rgb_low = [0,0,0];  
        return
    end
    counter = uint8(fix(counter));
    switch(counter)
        case(1)
             % Threshold for RGB - RED
            color_rgb_low = [96,0,0];
            color_rgb_hi = [255,70,70];
        case(2)
            % Threshold for RGB - GREEN
            color_rgb_low = [0,0,0];
            color_rgb_hi = [45,255,130];
        case(3)
            % Threshold for RGB - BLUE
            color_rgb_low = [0,0,150];
            color_rgb_hi = [130,180,255];        
        case(4)
            % Threshold for RGB - YELLOW
            color_rgb_low = [0,145,0];
            color_rgb_hi = [255,255,90];
        otherwise              
            color_rgb_hi = [0,0,0];
            color_rgb_low = [0,0,0];  
    end       
    
end