% Move Conveyor Forwards
function moveConveyor(direction,enable)
    
    global socket_1;    

    if (enable)
        if (direction) %direction = true (forward towards robot)
            %fwrite command for direction
            fwrite(socket_1,'CFW');
            fwrite(socket_1,'CON');
            pause(0.50);
            fwrite(socket_1,'COF');
        else %direction = false (backward away from robot)
            %fwrite command for direction
            fwrite(socket_1,'CBK');
            fwrite(socket_1,'CON');
            pause(0.50);
            fwrite(socket_1,'COF');
        end
    end
end

