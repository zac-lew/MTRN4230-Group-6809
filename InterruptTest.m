clear; close all;

robot_IP_address = '127.0.0.1'; % Simulation ip address
% robot_IP_address = '192.168.125.1'; % Simulation ip address

robot_port1 = 1026;
robot_port2 = 1025;

socket = tcpip(robot_IP_address, robot_port1);
set(socket, 'ReadAsyncMode', 'continuous');
set(socket, 'BytesAvailableFcn', 'dispcallback');

socket2 = tcpip(robot_IP_address, robot_port2);
set(socket, 'ReadAsyncMode', 'continuous');

if(~isequal(get(socket, 'Status'), 'open'))
    try
        fopen(socket);
        disp('Connected');
        %app.TextArea.Value = 'Robot Connected SUCCESS';  
        %app.ConnectionStatusLamp.Color = 'g';
    catch
        fprintf('Could not open TCP connection to %s on port %d\n',robot_IP_address, robot_port1);
        %app.TextArea.Value = 'Robot Connection FAILED';  
        %app.ConnectionStatusLamp.Color = 'r';
    end
end

if(~isequal(get(socket2, 'Status'), 'open'))
    try
        fopen(socket2);
        disp('Connected');
        %app.TextArea.Value = 'Robot Connected SUCCESS';  
        %app.ConnectionStatusLamp.Color = 'g';
    catch
        fprintf('Could not open TCP connection to %s on port %d\n',robot_IP_address, robot_port2);
        %app.TextArea.Value = 'Robot Connection FAILED';  
        %app.ConnectionStatusLamp.Color = 'r';
    end
end

while true
    str = fgetl(socket);    
end


pause(10000);

while (isequal(get(socket, 'Status'), 'open')) 
    str = fgetl(socket);
    disp(str);
end

% data = load('sample_x_y.mat');
% str = ""; 
% i = 0;
% while (isequal(get(socket, 'Status'), 'open')) 
%     disp('Data Sending');
%     initial = "[" + 60 + "," + 1 + "]"; 
%     fwrite(socket,initial);
%     % str = fgetl(socket);
%     % get the all clear for the first set of data
%     while (~strcmp(str,'DONE'))
%         disp('Waiting for DONE');
%         str = fgetl(socket);
%         disp(str);
%     end
%     str = ""; 
%     n = 1;
%     %  send the whole array 
%     while n <= length(data.data)
%         send_str = "[" + num2str(data.data(n,1)*1000 ) + "," + num2str(data.data(n,2)*1000) + "]"; 
%         disp(send_str)
%         fwrite(socket, send_str); 
%         % str = fgetl(socket);
%         % get the all clear for array data 
%         while (~strcmp(str,"DONE"))
%             disp('Waiting for DONE');
%             str = fgetl(socket);
%             disp(str);
%         end
%         str = "";
%         n=n+1;
%         i = i + 1;
%         disp(i);
%     end
%     
%     break; 
% end
% pause(2);

% Close the socket.
fclose(socket);