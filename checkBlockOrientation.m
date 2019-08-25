function block_angle = checkBlockOrientation(block_image,location)
    
    block_image = rgb2gray(block_image);
    
    if (location == 1) % using robot cell camera
        threshold = 0.8;
    else
        threshold = 0.25;
    end
    
    block_image = imbinarize(block_image,threshold);         

%     figure
%     imshow(block_image);
%     hold on;
    
    % blob detection. detect single square. Refine blob detection
    blobs = iblobs(block_image);
    
    if (location == 1)
        blobs = iblobs(block_image,'boundary','area',[550,2800],'class',0,'aspect', [0.80,1]);    
    else
        blobs = iblobs(block_image,'boundary','area',[600,1500],'class',1,'aspect', [0.4,0.95]);
    end
    
    %blobs(1).plot_box('g','LineWidth',2)    
    edges = blobs(1).edge; % assuming only one blob has been detected - needs fixing by tightening the blob detection criteria so this is true
    [leftmost_x_val, leftmost_pt_ind] = min(edges(1,:));
    [highest_sq_y_val, highest_pt_ind] = min(edges(2,:));
    del_y = edges(2, leftmost_pt_ind) - edges(2, highest_pt_ind);
    del_x = edges(1, highest_pt_ind) - edges(1, leftmost_pt_ind);
    if (del_y == 0)
        block_angle = 0;
    else
        block_angle = atand(del_y/del_x);
    end   

end
