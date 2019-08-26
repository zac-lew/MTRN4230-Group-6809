function [correctColor,tempX,tempY] = isShapeRightColor(shape_color,tempJ,cBboxes,cImage,tempID,min_conveyor)
    
    correctColor = false;    
    tempX = 0;
    tempY = 0;
    csv_encoding = shape_color(2,tempJ);
    [color_rgb_hi,color_rgb_low] = RGB_IteratorC(csv_encoding);               

    if (color_rgb_hi == [0,0,0])
        correctColor = false;
        return;
    end
    
    mask_desiredC = (cImage(:,:,1) >= color_rgb_low(1)) & (cImage(:,:,1) <= color_rgb_hi(1)) & ...
            (cImage(:,:,2) >= color_rgb_low(2) ) & (cImage(:,:,2) <= color_rgb_hi(2)) & ...
            (cImage(:,:,3) >= color_rgb_low(3) ) & (cImage(:,:,3) <= color_rgb_hi(3));

    statsC = regionprops(mask_desiredC,'basic');
    Ccentroids = cat(1,statsC.Centroid);
    Careas = cat(1,statsC.Area); %(suitable area > 150)
    [sorted_area_C,sorted_area_rowC] = sort(Careas,'descend'); 
    checkMatch = false;
    missingBlockMatch = find(shape_color(1,:) == 0);
    missingBlockMatch = size(missingBlockMatch,2);
    
    if (size(sorted_area_rowC,1) > 0)
        for ctr = 1 : size(shape_color,2) - missingBlockMatch
            checkMatch = isInROI(cBboxes(tempID,:),Ccentroids(sorted_area_rowC(ctr),1),...
                Ccentroids(sorted_area_rowC(ctr),2));
            if (checkMatch == true && sorted_area_C(ctr) > min_conveyor)
                %which color was in BBox of correct shape
                tempX = Ccentroids(sorted_area_rowC(ctr),1);
                tempY = Ccentroids(sorted_area_rowC(ctr),2);
                disp('Correct Color AND Correct Shape!');
                plot(tempX,tempY,'g*','LineWidth',2);
                correctColor = true;
                break;
            end
        end
    end
end
