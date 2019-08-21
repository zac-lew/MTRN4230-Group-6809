clear; close all;

robot_IP_address = '127.0.0.1'; % Simulation ip address

robot_port = 1025;

socket = tcpip(robot_IP_address, robot_port);
set(socket, 'ReadAsyncMode', 'continuous');

if(~isequal(get(socket, 'Status'), 'open'))
    try
        fopen(socket);
        disp('Connected');
        app.TextArea.Value = 'Robot Connected SUCCESS';  
        app.ConnectionStatusLamp.Color = 'g';
    catch
        fprintf('Could not open TCP connection to %s on port %d\n',robot_IP_address, robot_port);
        app.TextArea.Value = 'Robot Connection FAILED';  
        app.ConnectionStatusLamp.Color = 'r';
    end
end

% arrays = [{'[-1,1,1,0,45]'};{'[-2,2,2,0,45]'};{'[-3,3,3,0,45]'}];
data = load('sample_x_y.mat');
initial = [length(data),1];

while (isequal(get(socket, 'Status'), 'open'))
    n = 1;
    i = 0;
    fwrite(initial);
    str = fgetl(socket);
    fprintf(char(str));
    while n < length(data.data)
        coord=[data.data(n) data.data(n+length(data.data))];
        fwrite(socket, num2str(coord)); % 1x5 array for pick and place shape blocks
        str = fgetl(socket);
        fprintf(char(str));
        while (~strcmp(str,'DONE'))
            i = i+1;
            disp(i);
            str = fgetl(socket);
            fprintf(char(str));
            fprintf("\n");
        end
        n=n+1;
        pause (2);
    end
end
pause(2);

% Close the socket.
fclose(socket);