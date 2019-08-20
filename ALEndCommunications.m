clear; close all;

robot_IP_address = '127.0.0.1'; % Simulation ip address

robot_port = 1025;

socket = tcpip(robot_IP_address, robot_port);
set(socket, 'ReadAsyncMode', 'continuous');

if(~isequal(get(socket, 'Status'), 'open'))
    try
        fopen(socket);
        disp('Connected');
        %app.TextArea.Value = 'Robot Connected SUCCESS';  
        %app.ConnectionStatusLamp.Color = 'g';
    catch
        fprintf('Could not open TCP connection to %s on port %d\n',robot_IP_address, robot_port);
        %app.TextArea.Value = 'Robot Connection FAILED';  
        %app.ConnectionStatusLamp.Color = 'r';
    end
end

str = ""; 
while (isequal(get(socket, 'Status'), 'open')) 
    initial = "[" + "-10000" + "," + "-10000" + "]"; 
    fwrite(socket,initial);
    while (~strcmp(str,"Thank_You:_Closing_Connection"))
        disp('Waiting for Completion');
        str = fgetl(socket);
        disp(str);
    end
end

% Close the socket.
fclose(socket);










