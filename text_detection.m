function text_detection()
    close all; 
    basename = 'cake-design-photos/'; 
    files = dir(strcat(basename, 'table*'));
    
    for f = files'
       img = iread(strcat(basename, f.name));
       run_text_detection(img, f.name);
       pause;
       close all;
    end
        
end

function run_text_detection(img, filename)
    %img = iread('w8-photos/table (31).jpg');
    grey = im2double(rgb2gray(img));
    grey = im2double(grey); % normalises greyscale to [0,1] range
    grid_th = getGridRoi(grey);
    blobs = getBlobs(grid_th);
    
    % plot the blobs
    figure; imshow(grid_th);
    title(sprintf('%s, num blobs = %d', filename, length(blobs)));
    blobs.plot_box;    
end

function grid_th = getGridRoi(grey)
    % grid_roi_rect = [470,1112;346,703]; % found with iroi
    grid_roi_rect = [418,1182;285,789]; % covers all grid squares, including side ones
    th = otsu(grey);
    grey_th = (grey >= th); 
    grid_th = iroi(grey_th, grid_roi_rect);
end

function blobs = getBlobs(grid_th)
    blobs = iblobs(grid_th, 'area', [520, 3450], 'boundary', ...
        'class', 0, 'aspect', [0.08,1]);
    blobs(blobs.a < 40) = []; 
    blobs(blobs.perimeter < 160) = [];
    blobs(blobs.perimeter > 650) = [];
    
    % bold letters: 
    % area: [1075, 3417] (other blobs can be included in this range => don't use to differentiate)
    % aspect: [0.182, 0.990]
    % major axis height: [40,]
    % circularity: [0.1213, 0.6573]
    % perimeter: [184, 628]
    
    % thin letters:
    % area: [530, 1845]
    % aspect: [0.087, 0.950]
    % major axis height: [41,]
    % circularity: [0.061, 0.374]
    % perimeter: [172, 634]
    
    % use class of 0 to only detect black blobs. i.e. prevents blobs 
    % within shapes (e.g. o, b) being detected. 
    
    % note: letters will be *at least* 50mm high. So do not rely on upper
    % area bound, or a specfic major axis height (though could have lower
    % bound). 
end