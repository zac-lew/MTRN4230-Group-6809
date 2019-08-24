% Check if any label from ML detector on live frame
% matches the designated customer shape/color     
function [shapeFound,shapeID] = shapeCheck(curr_Labels,curr_Shape)
%encoded shape ~ 1-6
    
    shapeFound = false;
    shapeID = 0;
    
    %eg: args are cLabels,shape_color(1,j)
    for k = 1 : size(curr_Labels,1)
        if (curr_Labels(k) == curr_Shape)
            shapeFound = true;
            shapeID = k;
        end
    end

end