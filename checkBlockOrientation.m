function block_angle = checkBlockOrientation(block_image)

    % find angle - Ben's code
    %     grey = im2double(rgb2gray(block_image));
    %     th = otsu(grey);
    %     grey_th = grey >= th;
    %     assumptions for blob detection:
    %     - area is between 1000-1500 pixels
    %     - the blob is black (class 0; white is class 1)
    %     blob detection. detect single square. Refine blob detection
    %     blobs = iblobs(grey_th, 'boundary', 'area', [500, 1500], 'class', 0, 'aspect', [0.8,1]);
    %     edges = blobs(1).edge; % assuming only one blob has been detected - needs fixing by tightening the blob detection criteria so this is true
    %     [leftmost_x_val, leftmost_pt_ind] = min(edges(1,:));
    %     [highest_sq_y_val, highest_pt_ind] = min(edges(2,:));
    %     del_y = edges(2, leftmost_pt_ind) - edges(2, highest_pt_ind);
    %     del_x = edges(1, highest_pt_ind) - edges(1, leftmost_pt_ind);
    %     block_angle = atand(del_y/del_x);
    
end