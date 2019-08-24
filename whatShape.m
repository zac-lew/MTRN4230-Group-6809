% Encoding for color and shape
function shapeName = whatShape(match_shape)

    if (match_shape == 1)
        shapeName = 'Circle';
    elseif (match_shape == 2)
        shapeName = 'Clover';
    elseif (match_shape == 3)
        shapeName = 'CrissCross';
    elseif (match_shape == 4)
        shapeName = 'Diamond';
    elseif (match_shape == 5)
        shapeName = 'Square';
    elseif (match_shape == 6)
        shapeName = 'Starburst';
    else
        shapeName = '-';
    end
    
end