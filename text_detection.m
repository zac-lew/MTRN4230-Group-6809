function [paths, stroke_im, n_letters, letter_thickness] = text_detection(img)
    debug = 0;
    
    addpath('export-fig\'); % need export-fig to export blob stroke plot
    addpath('douglas-peucker\'); % for line simplification
    grey = im2double(rgb2gray(img));
    [grid_th, grid_offset] = getGridRoi(grey);

    blobs = getBlobs(grid_th);
    [thin_blob_ind, thick_blob_ind] = findThinThickBlobs(blobs, grid_th);
    %letter_thickness = findLetterThickness(blobs, grid_th);

    %triple_pts = getTriplePoints(grey, grid_th, blobs);

    n_letters = size(blobs,2);
    
    load('irb120.mat');
    paths = cell(1, n_letters); 
    letter_thickness = cell(1, n_letters);
    stroke_im = cell(1, n_letters); 

    if debug == 1
        % plot all strokes on the grid image
        grid_fh = figure; idisp(grid_th); hold on;
    else
        % we don't want to plot the strokes
        grid_fh = -1;
    end
    
    letter_fh = figure; 

    for i = 1:n_letters
        [strokes_grid_frame, stroke_im{i}] = calculateStrokes(grid_th, blobs(i), grid_fh, letter_fh);
        paths{i} = strokesToRobFrame(strokes_grid_frame, grid_offset);
        letter_thickness{i} = getLetterThickness(thin_blob_ind, ...
            size(paths{i}, 2), i);
        %plotStrokesWithRobot(strokes_rob_frame, irb120);
    end
    
    close(letter_fh);
    
    % plot the blobs
    %figure; imshow(grid_th);
    %title(sprintf('num blobs = %d', lengt(blobs)));
    %blobs(thin_blob_ind).plot_box; 
    %blob_img = 1;
end

function letter_thickness = getLetterThickness(thin_blob_ind, n_strokes, letter_ind)
   letter_thickness = cell(1, n_strokes);
   
   for i = 1:n_strokes
       letter_thickness{i} = thin_blob_ind(letter_ind) * 1;
   end
end

function [thin_blob_ind, thick_blob_ind] = findThinThickBlobs(blobs, grid_th)
    thick_grid_th = erodeThinLetters(grid_th);
    thick_blobs = getBlobs(thick_grid_th);   
    
    n = size(blobs, 2);
    n_th = size(thick_blobs, 2);
    sig_dist = 3; % pixels
    
    if isempty(thick_blobs)
       thick_blob_ind = logical(zeros(1, n)); 
       thin_blob_ind = logical(ones(1, n));
       return;
    end    
    
    thick_blob_ind = zeros(1, n);

    for i = 1:n
        for j = 1:n_th
            dist = distToPt(blobs(i).p, thick_blobs(j).p);
            
            if dist < sig_dist
                thick_blob_ind(i) = 1;
            end
        end
    end
    
    thick_blob_ind = logical(thick_blob_ind);
    thin_blob_ind = logical(1 - thick_blob_ind);
end

function plotStrokesWithRobot(strokes_rob_frame, irb120)
    n = size(strokes_rob_frame, 2);
    qz = [0 0 0 0 0 0];    
    
    for i = 1:n
        fh2 = figure;
        n_pts = size(strokes_rob_frame{i},1);
        plot3(strokes_rob_frame{i}(:,1), strokes_rob_frame{i}(:,2), 0.147*ones(n_pts,1), 'k', 'LineWidth', 1);
        irb120.plot(qz);
        pause;
        close(fh2);
    end
end

function [grid_th, grid_offset] = getGridRoi(grey)
    % grid_roi_rect = [470,1112;346,703]; % found with iroi
    grid_roi_rect = [418,1182;285,789]; % covers all grid squares, including side ones
    %grid_roi_rect = [542,1055;278,791]; % covers only middle grid squares
    th = otsu(grey);
    grey_th = (grey >= th); 
    grid_th = iroi(grey_th, grid_roi_rect);
    grid_offset = [grid_roi_rect(1,1); grid_roi_rect(2,1)];
end

function im = erodeThinLetters(grid_th) 
    im = idilate(grid_th, ones(5,5));
end

function blobs = getBlobs(grid_th)
    blobs = iblobs(grid_th, 'area', [520, 3450], 'boundary', ...
        'class', 0, 'aspect', [0.08,1]);
    
    if isempty(blobs)
        return
    end
    
    if ~isempty(find(blobs.a < 38, 1))
        blobs(blobs.a < 38) = []; 
    end
    
    if ~isempty(find(blobs.perimeter < 160, 1))
        blobs(blobs.perimeter < 160) = [];
    end
    
    if ~isempty(find(blobs.perimeter > 650, 1))
        blobs(blobs.perimeter > 650) = [];
    end    
    
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
        %loc_tps = convertLogToPoints(triple_points_log, zeros(2,1));
        skel_points = convertLogToPoints(skeleton, tp_offset);
        end_pts = convertLogToPoints(end_pts_log, tp_offset);
        plot_point(skel_points, 'r.');
        plot_point(triple_points, 'w+');
        plot_point(end_pts, 'go');
    end
end

% Calculates the strokes of the letter. A stroke is the part of the letter
% the end-effector can complete in one motion without going back over
% already-placed ink. Each letter is made up of one or more strokes. 
function [strokes_grid_frame, stroke_im] = calculateStrokes(grid_th, blob, grid_fh, letter_fh)
    %close all;
    end_pt_th = 15;
    max_strokes = 20;
    max_strk_size = 150;
    strk_pt_freq = 2; % save every n pts for stroke
    
    % use a larger bbox to ensure the blob isn't touching the edges
    roi = blob.bbox + [-1 1; -1 1];
    blob_im = iroi(grid_th, roi);
    skeleton = ithin(1 - blob_im); % use inverse image
    triple_points = itriplepoint(skeleton);
    end_points = iendpoint(skeleton);
    
    % offset = [blob.umin; blob.vmin]; % + [-2; -2]; 
    offset = [0; 0]; % offset is zero in the skeleton frame of ref. 
    unexplored_tps = convertLogToPoints(triple_points, offset);
    %loc_tps = convertLogToPoints(triple_points_log, zeros(2,1));
    
    %skel_points = convertLogToPoints(skeleton, tp_offset);
    %end_pts = convertLogToPoints(end_pts_log, tp_offset);

    n_unexplored = size(unexplored_tps, 2);
    cur_pt = unexplored_tps(:,1); % todo: optimise tp choice
    prev_pt = cur_pt;
    %figure; idisp(blob_im);
    visited_tps = zeros(size(triple_points));
    %figure; idisp(skeleton);
    strokes = cell(1, max_strokes);
    strk = zeros(3, max_strk_size);
    seg = zeros(3, max_strk_size);
    strokes_ind = 1;
    strk_ind = 1;
    seg_ind = 1;
    i = 0;
    
    while n_unexplored > 0
        next_pt = getNextPt(cur_pt, prev_pt, skeleton);%, unexplored_tps, n_unexplored);
        prev_pt = cur_pt;        
        %plot_point(cur_pt, 'w.');
        %idisp(skeleton);
        
        % only remove non-tps from skeleton, allowing us to revisit
        if ~triple_points(cur_pt(2), cur_pt(1))
            skeleton(cur_pt(2), cur_pt(1)) = 0; % we've visited this pt
        end
        
        % save points to current segment
        if mod(i, strk_pt_freq) == 0 % || triple_points(cur_pt(2), cur_pt(1))
           seg(:, seg_ind) = [cur_pt; 0]; % todo remove 0: replace with line width flag
           seg_ind = seg_ind + 1;
        end        
        
        if triple_points(cur_pt(2), cur_pt(1))
            visited_tps(cur_pt(2), cur_pt(1)) = 1;            
            
            if regionIsExplored(cur_pt, skeleton) ...
                    || willBeDeserted(cur_pt, skeleton, visited_tps)
                % remove tp from search
                [v, ind] = ismember(cur_pt, unexplored_tps);
                
                if ind(2) > 0 
                    unexplored_tps(:, ceil(ind(2)/2)) = [];
                    
                    % decrement n_unexplored tps
                    n_unexplored = n_unexplored - 1;
                end                    
                    
                skeleton(cur_pt(2), cur_pt(1)) = 0;
                %triple_points(cur_pt(2), cur_pt(1)) = 0;
                
                if n_unexplored > 0
                    next_pt = unexplored_tps(:, n_unexplored);
                end
                %prev_pt = next_pt; % to prevent problems in getNextStep
                
                % finish stroke
                [seg_ind, strk_ind, strk, seg] = flushSeg(seg_ind, strk_ind, seg, strk);
                [strk_ind, strokes_ind, strokes, strk] = flushStroke(strokes_ind, strk_ind, strk, strokes);
            else
                [seg_ind, strk_ind, strk, seg] = flushSeg(seg_ind, strk_ind, seg, strk);
                last_tp = cur_pt; 
            end
        elseif end_points(cur_pt(2), cur_pt(1)) ...
                || regionIsExplored(cur_pt, skeleton)
            if distToPt(cur_pt, last_tp) > end_pt_th 
               % keep this segment and finish the stroke 
               [seg_ind, strk_ind, strk] = flushSeg(seg_ind, strk_ind, seg, strk);
               [strk_ind, strokes_ind, strokes, strk] = flushStroke(strokes_ind, strk_ind, strk, strokes);
               next_pt = unexplored_tps(:, n_unexplored);
            else 
               % discard this segment, and move back to the last tp
               seg_ind = 1;
               next_pt = last_tp;
            end
        end
        
        cur_pt = next_pt;
        i = i + 1;
    end
    
    % plot the path
    
    tp_offset = [blob.umin; blob.vmin] + [-2; -2];
    strokes = deleteUnusedStrokes(strokes, strokes_ind, max_strokes);
    strokes_grid_frame = changeStrokesFrame(strokes, strokes_ind, tp_offset);
    
    col = {'w-', 'r-', 'g-', 'y-', 'c-', 'm-', 'b-'};
    %col = {'wo', 'ro', 'go', 'yo', 'co', 'mo', 'bo'};
    col3 = {'w+', 'r+', 'g+', 'y+', 'c+', 'm+', 'b+'};
    
    figure(letter_fh); imshow(blob_im); hold on;
    
    for i = 1:strokes_ind-1
        %figure(1); hold on;
        %plot_point(strokes_grid_frame{i}(1:2,:), col2{i});
        
        if grid_fh ~= -1
            figure(grid_fh);
            plot(strokes_grid_frame{i}(1,:), strokes_grid_frame{i}(2,:),col{i});%, 'LineWidth', 1.5);
            plot_point(strokes_grid_frame{i}(1:2,1), col3{i});%, 'LineWidth', 1.5);
            plot_point(strokes_grid_frame{i}(1:2,end), col3{i+1});%, 'LineWidth', 1.5);
        end
        
        figure(letter_fh);
        plot(strokes{i}(1,:), strokes{i}(2,:),col{i});
        plot_point(strokes{i}(1:2,1), col3{i});
        plot_point(strokes{i}(1:2,end), col3{i+1});
    end
    
    stroke_im = export_fig(letter_fh, '-q0');
    figure(letter_fh); hold off;
end

function strokes = deleteUnusedStrokes(strokes, strokes_ind, max_strokes)
    for i = max_strokes:-1:strokes_ind
       strokes(i) = []; 
    end
end

function rob_strokes = strokesToRobFrame(strokes, offset)
    n = size(strokes, 2);
    rob_strokes = cell(1, n);
    
    load('cameraParams.mat');
    load('rot_matrix.mat');
    load('trans_mat.mat');    

    for i = 1:n
       strokes{i}(1:2,:) = strokes{i}(1:2,:) + offset;
       rob_strokes{i} = pointsToWorld(cameraParams, R, t, strokes{i}(1:2,:)') / 1000;
    end
    %strokes_cam = changeStrokesFrame(strokes_grid_frame, n, grid_offset);
end

function strokes = changeStrokesFrame(strokes, strokes_ind, offset)
    for i = 1:strokes_ind - 1
       strokes{i}(1:2,:) = strokes{i}(1:2,:) + offset;
    end
end

function [seg_ind, strk_ind, strk, seg] = flushSeg(seg_ind, strk_ind, seg, strk)
    seg_offset = seg_ind - 1;
    strk(:, strk_ind : strk_ind + seg_offset) = seg(:, 1:seg_ind);
    strk_ind = strk_ind + seg_offset;
    seg_ind = 1;
    %seg = zeros(size(seg));
end

function [strk_ind, strokes_ind, strokes, strk] = flushStroke(strokes_ind, strk_ind, strk, strokes)
    if strk_ind < 5
        strk_ind = 1;
        return;
    end
    
    simplify = 1;
    
    if simplify
        % simplify the stroke with the Douglas Peucker algorithm

        simple_strk = DouglasPeucker(strk(1:2, 1:strk_ind - 1), 1);
        strokes{strokes_ind} = simple_strk';
    else
        strokes{strokes_ind} = strk(:, 1:strk_ind - 1); 
    end
        
    strk_ind = 1;
    strokes_ind = strokes_ind + 1;
    %strk = zeros(size(strk));
end

function bool = willBeDeserted(cur_pt, skeleton, visited_tps)
   region = skeleton(cur_pt(2) - 1:cur_pt(2) + 1, ...
       cur_pt(1) - 1:cur_pt(1) + 1); 
   visited_tp_region = visited_tps(cur_pt(2) - 1:cur_pt(2) + 1, ...
       cur_pt(1) - 1:cur_pt(1) + 1); 
   region(visited_tp_region == 1) = 0; % can't visit the seen tps
   region(2,2) = 0; % set current pt to zero
   
   if sum(region(:)) == 0
       bool = 1;
   else 
       bool = 0;
   end
end

function dist = distToPt(a, b)
   dist = sqrt( (a(1) - b(1))^2 + (a(2) - b(2))^2 );
end

function bool = regionIsExplored(cur_pt, skeleton)
   region = skeleton(cur_pt(2) - 1:cur_pt(2) + 1, ...
       cur_pt(1) - 1:cur_pt(1) + 1);    
   num_unexplored_paths = sum(region(:));
   
   if num_unexplored_paths > 0 
       bool = 0;
   else
       bool = 1;
   end
end

function pt = getNextPt(cur_pt, prev_pt, skeleton)
   %ccw_mask = [9 10 3; 8 0 4; 7 6 5]; 
   ccw_mask = [6 10 3; 9 0 7; 5 8 4];
   region = skeleton(cur_pt(2) - 1:cur_pt(2) + 1, ...
       cur_pt(1) - 1:cur_pt(1) + 1);
   
   % remove prev_pt from the region as we can't retrace our steps
   del = prev_pt - cur_pt; 
   if abs(del(1)) <= 1 && abs(del(2)) <=1
       region_mid = [2; 2];
       region(del(2) + region_mid(2), del(1) + region_mid(1)) = 0;
   end
   
   % also remove all points in the direction from which we came, when
   % we came from either N, E, S or W. 
   if abs(del(1)) <= 1 && abs(del(1)) > 0 && abs(del(2)) == 0
       region(:, del(1) + region_mid(1)) = 0;
   elseif abs(del(2)) <= 1 && abs(del(2)) > 0 && abs(del(1)) == 0
       region(del(2) + region_mid(2), :) = 0;
   end
   
   masked_region = region .* ccw_mask; 
   [max_val, max_ind] = max(masked_region(:));
   
   % find the coordinates of this new-pt in the skeleton frame, using 
   % the returned masked value. 
   pt = cur_pt;
   
   x_change_mask = [-1 0 1; -1 0 1; -1 0 1];
   y_change_mask = [-1 -1 -1; 0 0 0; 1 1 1];
   
   chosen_pt = zeros(3,3);
   chosen_pt(max_ind) = 1;
   
   x_change = x_change_mask .* chosen_pt;
   y_change = y_change_mask .* chosen_pt;
   
   pt(1) = pt(1) + x_change(max_ind);
   pt(2) = pt(2) + y_change(max_ind);
end

function pts = convertLogToPoints(logical, offset) 
    [y, x] = find(logical);
    pts = [x'; y'];
    pts = pts + offset;
end
