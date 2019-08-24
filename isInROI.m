function checkMatch = isInROI(ROI,x,y)
    checkMatch = false;
    %ROI is (x,y,x_length,y_length)
    if ((x > ROI(1) && x < ROI(1) + ROI(3)) && (y > ROI(2) && y < ROI(2) + ROI(4)))
        checkMatch = true;
    end

end