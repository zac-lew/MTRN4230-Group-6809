function text_detection()
    close all; 
    basename = '../cake-design-photos/'; 
    files = dir(strcat(basename, 'table*'));
    
    for f = files'
       img = iread(strcat(basename, f.name));
       run_text_detection(img, f.name);
       pause;
       close all;
    end
        
end

function run_text_detection(img, filename)
    % img = iread('../w8-photos/table (5).jpg');
    grey = im2double(rgb2gray(img));
    grey = im2double(grey); % normalises greyscale to [0,1] range
    grid_th = getGridRoi(grey);
    blobs = getBlobs(grid_th);
    triple_pts = getTriplePoints(grey, grid_th, blobs);
    
    % plot the blobs
    %figure; imshow(grid_th);
    %title(sprintf('%s, num blobs = %d', filename, length(blobs)));
    %blobs.plot_box;    
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

function triple_points = getTriplePoints(im, grid_th, blobs)
    n = size(blobs, 2);
    figure; idisp(grid_th);
    hold on;
    
    for i = 1:n
        % use a larger bbox to ensure the blob isn't touching the edges
        roi = blobs(i).bbox + [-1 1; -1 1];
        blob_im = iroi(grid_th, roi);
        skeleton = ithin(1 - blob_im); % use inverse image
        triple_points_log = itriplepoint(skeleton);
        end_pts_log = iendpoint(skeleton);
        
        %[tp_y, tp_x] = find(triple_points_log); 
        
        %triple_points = [tp_x'; tp_y'];
        % convert to picture coord frame, allowing for larger bbox
        %triple_points = triple_points + [blobs(i).umin; blobs(i).vmin] ...
        %    + [-1; -1];
        
        tp_offset = [blobs(i).umin; blobs(i).vmin] + [-2; -2];
        triple_points = convertLogToPoints(triple_points_log, tp_offset);
        loc_tps = convertLogToPoints(triple_points_log, zeros(2,1));
        skel_points = convertLogToPoints(skeleton, tp_offset);
        end_pts = convertLogToPoints(end_pts_log, tp_offset);
        plot_point(skel_points, 'r.');
        plot_point(triple_points, 'w+');
        plot_point(end_pts, 'go');
    end
end

function pts = convertLogToPoints(logical, offset) 
    [y, x] = find(logical);
    pts = [x'; y'];
    pts = pts + offset;
end