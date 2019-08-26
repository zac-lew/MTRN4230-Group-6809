function [shape_color,missingBlockMatch] = analyseCustomerImage(customerImage,ML_threshold,min_block_size)
    global detector_updated_FINAL;
    global camParam_Table R_Table t_Table;

    rectROI = [560.51,290.51,477.98,485.98];
    ROI_image = imcrop(customerImage,rectROI); 

    figure
    imshow(ROI_image)
    hold on

    % Detect Quirkle Blocks using ML detector
    [bboxes,scores,labels] = detect(detector_updated_FINAL,ROI_image,'Threshold',...
        ML_threshold,'NumStrongestRegions',15);

    % Remove double detections from ML
    sorted_detect = sort(scores,'descend');

    for j = 1 : size(bboxes,1) 
        for check_ctr = 1 : size(bboxes,1)  
            D = sqrt((bboxes(j,1) - bboxes(check_ctr,1))^2 +...
                (bboxes(j,2) - bboxes(check_ctr,2))^2);
            if (D < 15 && D > 0)
                if scores(j) > scores(check_ctr)
                    bboxes(check_ctr,:) = 0;                
                else
                    bboxes(j,:) = 0;                
                end            
            end    
        end
    end  

    bboxes = bboxes(any(bboxes,2),:);

    % Annotate BB around detected shapes
    for j = 1 : size(bboxes,1)

        if (scores(j) < ML_threshold)
            continue;
        end

        if (bboxes(j,1) ~= 0)
            rectangle('Position',[bboxes(j,1),bboxes(j,2),bboxes(j,3),bboxes(j,4)],'EdgeColor'...
                  ,'r','LineWidth',2); 
            ML_1 = sprintf('%0.3f',scores(j));
            ML_2 = sprintf('%s',labels(j));
            text(bboxes(j,1)-10,bboxes(j,2)-25,{[ML_1];[ML_2]},'FontSize',10,'Color','r','FontWeight','bold')
        end

    end

    % 3. HSV Color Filtering + Localisation

    num_blocks = find(bboxes(:,1) ~= 0);
    num_blocks = size(num_blocks,1);

    % array to store which color (R,G,B,Y)
    color_array = zeros(1,4); 
    color_hsv_low = zeros(1,3);
    color_hsv_hi = zeros(1,3);

    hsv_path = ROI_image; %using RBG now
    curr_filter_on = 1;
    max_hsv = 4;
    filter_counter = num_blocks;
    block_counter = 0;

    % "struct" to store x,y,z and hsv filter
    cv_block_struct = zeros(4,num_blocks);
    block_struct_row = 1;

        % Making HSV filtering dynamic and automatically iterate through all
        % 4 HSV filter ranges

    for h = curr_filter_on:max_hsv % encoding RGBY as 1234

        [color_hsv_hi,color_hsv_low] = HSV_Iterator(h);

        % Create mask to find pixels with desired HSV ranges (binary mask) -
        % Current iterated HSV filter    
        mask_desired = (hsv_path(:,:,1) >= color_hsv_low(1)) & (hsv_path(:,:,1) <= color_hsv_hi(1)) & ...
                (hsv_path(:,:,2) >= color_hsv_low(2) ) & (hsv_path(:,:,2) <= color_hsv_hi(2)) & ...
                (hsv_path(:,:,3) >= color_hsv_low(3) ) & (hsv_path(:,:,3) <= color_hsv_hi(3));

        SE = strel('disk',2);
        mask_desired = imdilate(mask_desired,SE);

        stats = regionprops(mask_desired,'basic');
        centroids = cat(1,stats.Centroid);

        areas = cat(1,stats.Area); %(suitable area > 150)
        [sort_area_m,sorted_area_row] = sort(areas,'descend');
        % Checking max number of items WITH a particular HSV filter
        for p = 1:filter_counter

          [color_row,~] = size(sort_area_m); %number of centroids per color
          % Error handling if no suitably sized object with color is present

            if (p > color_row)
                continue;
            elseif (color_row < num_blocks && sort_area_m(p,1) < min_block_size)
                continue;
            end                

            if (sort_area_m(p,1) >= min_block_size)
                hold on
                % Only plot a + if there is a suitable sized binary area           
                plot(centroids(sorted_area_row(p),1),centroids(sorted_area_row(p),2),'g+','LineWidth',2)
                cv_block_struct(1,block_struct_row) = centroids(sorted_area_row(p),1);
                cv_block_struct(2,block_struct_row) = centroids(sorted_area_row(p),2);
                cv_block_struct(3,block_struct_row) = 147.00;
                cv_block_struct(4,block_struct_row) = h; % which HSV filter was used
                block_counter = block_counter + 1;
                block_struct_row = block_struct_row + 1;
            else
                % not even one suitable block was found in particular color
                continue;
            end
        end 
    end

    % -----------ERROR CHECKING-----------
    % Cleaning up double color localistaion errors AND outside ML results
    for j = 1 : size(cv_block_struct,2) 
        for check_ctr = 1 : size(cv_block_struct,2)  
            D = sqrt((cv_block_struct(1,j) - cv_block_struct(1,check_ctr))^2 +...
                (cv_block_struct(2,j) - cv_block_struct(2,check_ctr))^2);
            if (D < 10 && D > 0)
               cv_block_struct(:,check_ctr) = 0;
            end    
        end
    end
    
    image_place_data = cv_block_struct; % Image coordinate frame
    colShapCheck = false;
    checkOnce = false;
    tempCheck = false;
    colIncorrect = 0;
    for k = 1 : size(image_place_data,2)
        for centroidCheck = 1 : size(bboxes,1)
            colShapCheck = isInROI(bboxes(centroidCheck,:),image_place_data(1,k),...
                image_place_data(2,k));
            if (colShapCheck == true)
                tempCheck = true;
            end
            if (tempCheck == false && centroidCheck == size(bboxes,1))
                colIncorrect = k;            
            end
        end
        tempCheck = false;
    end
    
    if (colIncorrect ~= 0)
        % Removing any missing centroids before color_shape match
        %image_place_data(:,colIncorrect) = 0;
        %image_place_data( :, ~any(image_place_data,1) ) = [];  %columns
    end
    
    pick_counter = 1;
    j = 1;
    missingCentroid = false;
    shape_color = zeros(2,size(bboxes,1));
    image_place_data = fix(image_place_data);
    missingBlockMatch = find(all(bboxes == 0,2));
    missingNumber = size(missingBlockMatch,1);

    while (pick_counter <= size(bboxes,1) && missingCentroid == false)
        % each bounding box vector from FRCNN in turn
        % Skip over double detection
        if (bboxes(pick_counter,1) == 0)
            pick_counter = pick_counter + 1;
        end
        ROI = bboxes(pick_counter,:); 
        while (j <= size(bboxes,1))
            % check if any of the color/centroids are in this current ROI 

            tf = isInROI(ROI,image_place_data(1,j),image_place_data(2,j));

            if (tf == true)            
                % Store which shape
                shape_color(1,pick_counter) = labels(pick_counter);
                % Store which color (matching)
                shape_color(2,pick_counter) = image_place_data(4,j);
                shape_color(3:4,pick_counter) = image_place_data(1:2,j);
                pick_counter = pick_counter + 1; % fill in shape_color array
                % after each successful match
                % Break out of loop
                plot(image_place_data(1,j),image_place_data(2,j),'bo','LineWidth',1.5);
                break;
            end

            % Error handle in case of missing centroid
            if (j == size(bboxes,1))
                % Can't find centroid

                if (pick_counter < size(bboxes,1))
                    pick_counter = pick_counter + 1;
                    break;
                else
                    missingCentroid = true; % raise flag
                    break;
                end                         
            end

            % if tf is false (the current centroid is not in current ROI
            j = j+1; % increment to next centroid location from CV
        end

        j = 1; % reset counter for the next centroid check
    end

    for r = 1 : size(shape_color,2)
        fprintf('%s %s\n',whatShape(shape_color(1,r)),whatColor(shape_color(2,r)));
    end

    % ---------------------Orientation of Blocks----------------------
    % Image processing to determine orientation of blocks
    % Call function for each detected block in turn

    tempROI_image = ROI_image;
    for k = 1: size(shape_color,2)

        hold on

        % avoid calculating angle if missing color/shape match up
        if (shape_color(1,k) == 0 || shape_color(2,k) == 0)
            continue;
        end
        block_dim = 52;
        % Make temp comparison image (for each block)    
        angle_roi = [shape_color(3,k)-block_dim/2,shape_color(4,k)-block_dim/2,block_dim,block_dim];
        aligned_block = imcrop(tempROI_image,angle_roi); % CustomerImage remains as RGB for color detection

        % Call function to detect orientation
        % !!!!!!!!!!!!!!!!!!!! TEMPORARILY COMMENTED OUT WHILE NOT WORKING
        %block_angle = checkBlockOrientation(aligned_block,1);   
        block_angle = 45;
        text(shape_color(3,k)-50,shape_color(4,k),num2str(block_angle),'FontSize',10,'Color','b','FontWeight','bold')
        shape_color(5,k) = round(block_angle);
        tempROI_image = ROI_image;
    end

    hold off

    % Convert image points to world coordinates (PLACE)
    for bCount = 1:num_blocks
        shape_color(3:4,bCount) = pointsToWorld(camParam_Table,...
            R_Table,t_Table,...
            [559.51+shape_color(3,bCount),290.51+shape_color(4,bCount)]);
    end
    
    missingBlockMatch = find(shape_color(1,:) == 0);
    missingBlockMatch = size(missingBlockMatch,2);
end
