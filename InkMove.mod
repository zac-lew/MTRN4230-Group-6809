MODULE InkMove
    !PERS string In_str := "[1,1][0.3508,-0.17997]";
    !PERS string Out_str := "DONE";
    ! Code for translating a set of trajectories into robot motion
    
    !----------------------------
    ! networking variables 
    ! The socket connected to the client.
    VAR socketdev client_socket;
    ! The host and port that we will be listening for a connection on.
    PERS string host := "127.0.0.1";
    CONST num port := 1025;
    !----------------------------
    
    
    CONST num zTab := 147;
    
    ! VAR num aDrawPt1{59,5}:= [[350.798,-179.966,349.028,-180.007,0],[349.028,-180.007,346.150,-180.073,0],[346.150,-180.073,343.240,-180.139,0],[343.240,-180.139,340.982,-180.191,0],[340.982,-180.191,338.945,-180.238,0],[338.945,-180.238,336.908,-180.285,0],[336.908,-180.285,334.871,-180.332,0],[334.871,-180.332,332.834,-180.378,0],[332.834,-180.378,330.798,-180.425,0],[330.798,-180.425,328.761,-180.472,0],[328.761,-180.472,326.724,-180.519,0],[326.724,-180.519,324.687,-180.565,0],[324.687,-180.565,322.650,-180.612,0],[322.650,-180.612,320.613,-180.659,0],[320.613,-180.659,318.613,-180.695,0],[318.613,-180.695,316.818,-180.673,0],[316.818,-180.673,315.391,-180.545,0],[315.391,-180.545,314.448,-180.278,0],[314.448,-180.278,314.058,-179.854,0],[314.058,-179.854,314.244,-179.264,0],[314.244,-179.264,314.985,-178.515,0],[314.985,-178.515,316.209,-177.628,0],[316.209,-177.628,317.800,-176.636,0],[317.800,-176.636,319.597,-175.585,0],[319.597,-175.585,321.431,-174.523,0],[321.431,-174.523,323.264,-173.462,0],[323.264,-173.462,325.097,-172.400,0],[325.097,-172.400,326.931,-171.339,0],[326.931,-171.339,328.764,-170.277,0],[328.764,-170.277,330.597,-169.216,0],[330.597,-169.216,332.430,-168.155,0],[332.430,-168.155,334.263,-167.093,0],[334.263,-167.093,336.096,-166.032,0],[336.096,-166.032,337.929,-164.971,0],[337.929,-164.971,339.726,-163.920,0],[339.726,-163.920,341.319,-162.926,0],[341.319,-162.926,342.547,-162.034,0],[342.547,-162.034,343.296,-161.277,0],[343.296,-161.277,343.497,-160.673,0],[343.497,-160.673,343.125,-160.229,0],[343.125,-160.229,342.205,-159.939,0],[342.205,-159.939,340.806,-159.784,0],[340.806,-159.784,339.042,-159.730,0],[339.042,-159.730,337.075,-159.734,0],[337.075,-159.734,335.071,-159.748,0],[335.071,-159.748,333.068,-159.761,0],[333.068,-159.761,331.064,-159.775,0],[331.064,-159.775,329.061,-159.789,0],[329.061,-159.789,327.057,-159.803,0],[327.057,-159.803,325.054,-159.816,0],[325.054,-159.816,323.050,-159.830,0],[323.050,-159.830,321.046,-159.844,0],[321.046,-159.844,319.043,-159.858,0],[319.043,-159.858,317.039,-159.871,0],[317.039,-159.871,314.818,-159.887,0],[314.818,-159.887,311.955,-159.906,0],[311.955,-159.906,309.124,-159.926,0],[309.124,-159.926,307.383,-159.938,0],[307.383,-159.938,307.021,-159.940,0]];
    ! VAR num aDrawPt{59,2}:=[[350.798,-179.966],[349.028,-180.007],[346.150,-180.073],[343.240,-180.139],[340.982,-180.191],[338.945,-180.238],[336.908,-180.285],[334.871,-180.332],[332.834,-180.378],[330.798,-180.425],[328.761,-180.472],[326.724,-180.519],[324.687,-180.565],[322.650,-180.612],[320.613,-180.659],[318.613,-180.695],[316.818,-180.673],[315.391,-180.545],[314.448,-180.278],[314.058,-179.854],[314.244,-179.264],[314.985,-178.515],[316.209,-177.628],[317.800,-176.636],[319.597,-175.585],[321.431,-174.523],[323.264,-173.462],[325.097,-172.400],[326.931,-171.339],[328.764,-170.277],[330.597,-169.216],[332.430,-168.155],[334.263,-167.093],[336.096,-166.032],[337.929,-164.971],[339.726,-163.920],[341.319,-162.926],[342.547,-162.034],[343.296,-161.277],[343.497,-160.673],[343.125,-160.229],[342.205,-159.939],[340.806,-159.784],[339.042,-159.730],[337.075,-159.734],[335.071,-159.748],[333.068,-159.761],[331.064,-159.775],[329.061,-159.789],[327.057,-159.803],[325.054,-159.816],[323.050,-159.830],[321.046,-159.844],[319.043,-159.858],[317.039,-159.871],[314.818,-159.887],[311.955,-159.906],[309.124,-159.926],[307.383,-159.938]];
    
    CONST robtarget pZeros := [[0,0,0],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    
    VAR num startFlag:= 1;      ! to break out of the data recieving and drawing loop
    VAR num thickThin := 0;     ! is line thick of thin
    ! VAR num finalDone := 0;  ! is there new data to draw
    VAR num size := 0;          ! This is the size of the parsed trajectory before lifting
    VAR num DrawPt{500,2};      ! This is all continuous trajectory points 
    VAR num count := 0;
    VAR string input;           ! read in a string
    VAR num comma1;             ! to index between two commas
    VAR num comma2;
    
    VAR robtarget Pos1 := pZeros;           ! movement positions 
    VAR num Table_upOff := zTab + 50;       ! table offset 
    
    VAR string received_str := "0";
    VAR string In_str := "0";
    VAR string Out_str := "0";
    
    VAR num gotData{2};
    VAR bool ok;
    
    PROC Main()
        TPWrite "-------------- Start";  
        !----------------------------
        IF RobOS() THEN
            host := "192.168.125.1";
        ELSE
            host := "127.0.0.1";
        ENDIF
        !ListenForAndAcceptConnection;
        !----------------------------
        MoveToCalibPos;

        ! drawing loop
        WHILE startFlag = 1 DO
            ListenForAndAcceptConnection;
            ! Data recieving
            ! - first number of x y points 
            ! - second is thin or thick boolean
            ! - all other values are X, Y pos 
            !------------
            ! Get Setup trajectory data 
            ! Get data
            ! - about number of points 
            ! - thin or thick
            !------------
            ! Receive a string from the client.
            !SocketReceive client_socket \Str:=received_str;
            !received_str := "DONE";
            ! Send the string back to the client, adding a line feed character.
            !SocketSend client_socket \Str:=(received_str + "\0A");
            !------------
            ! Receive a string from the client.
            SocketReceive client_socket \Str:=In_str;
            !------------
            ! Get data from networking
            ! IF startFlag = 1 THEN    
            IF In_str <> "0" THEN 
                !------------
                input := In_str; 
                In_str := "0";                 ! say the string has been recieved 
                Out_str := "DONE";
                ! Send the string back to the client, adding a line feed character.
                SocketSend client_socket \Str:=(Out_str + "\0A");
                !------------
                
                ! test data 
                ! input := "[60,1]"; 
                
                ok := StrToVal(input, gotData);
                ! - first value is number of points 
                size := gotData{1}; 
                ! - second value is thick or thin 
                thickThin := gotData{2}; 
                ! if size = -10000: end the program 
                IF size = -10000 THEN
                    startFlag := 0;     ! break out and exit 
                    SocketSend client_socket \Str:=("Thank_You:_Closing_Connection" + "\0A");
                ENDIF
                
                ! networking - ask for next lot of data 
                !============================
                ! Get trajectory data 
                ! populate an array DrawPt: 2 at a time 
                count := 1;
                WHILE count <= size DO
                    ! Get data from networking
                    !------------
                    ! Receive a string from the client.
                    SocketReceive client_socket \Str:=In_str;
                    !------------
                    ! IF startFlag = 1 THEN  
                    IF In_str <> "0" THEN     
                        !------------
                        input := In_str; 
                        In_str := "0";                 ! say the string has been recieved 
                        Out_str := "DONE";
                        ! Send the string back to the client, adding a line feed character.
                        SocketSend client_socket \Str:=(Out_str + "\0A");
                        !------------

                        ! test data
                        ! input := "[350,-179]"; 
                        
                        ! - feed values into array 
                        ok := StrToVal(input, gotData);
                        ! - first value is number of points 
                        DrawPt{count,1} := gotData{1}; 
                        ! - second value is thick or thin 
                        DrawPt{count,2} := gotData{2}; 
                        count := count + 1;
                        
                    ENDIF
                ENDWHILE
                !--------
                IF size > 0 THEN
                    ! move to the starting position 
                    Pos1.trans := [DrawPt{1,1},DrawPt{1,2},Table_upOff];
                    ! Move just above the object
                    MoveJ Offs(Pos1,0,0,0), v200, fine, tSCup;
                    
                    ! Begin Ink tracing 
                    PrintLetter thickThin; 
                    
                    ! networking - say printing is completed
                ENDIF
            ENDIF
            CloseConnection;
        ENDWHILE
        
        !============================
        ! ALL Done move back to home
        MoveToCalibPos;
        
        TPWrite "-------------- Done";
    ENDPROC

    PROC PrintLetter(num thick_thin)
        VAR robtarget Pos1 := pZeros; 
        VAR num Table_upOff := zTab + 50;
        
        ! networking send -> ink is printing

    
        ! if 1 = thick line 
        IF thick_thin = 0 THEN 
            FOR i FROM 1 TO size DO
                Pos1.trans := [DrawPt{i,1},DrawPt{i,2}, Table_upOff];
                MoveL Pos1, v50, fine, tSCup;
            ENDFOR
            
        ELSEIF thick_thin = 1 THEN
           FOR i FROM 1 TO size DO
                Pos1.trans := [DrawPt{i,1},DrawPt{i,2}, Table_upOff];
                MoveL Pos1, v20, fine, tSCup;
            ENDFOR
        ENDIF
        
        ! networking send -> ink is done printing
        
        
        ! A point just above the target 
        MoveJ Offs(Pos1,0,0,50), v200, fine, tSCup;
        ! MoveToCalibPos;
    ENDPROC
    
    
    
    !----------------------------
     PROC ListenForAndAcceptConnection()
        ! Create the socket to listen for a connection on.
        VAR socketdev welcome_socket;
        SocketCreate welcome_socket;
        ! Bind the socket to the host and port.
        SocketBind welcome_socket, host, port;
        ! Listen on the welcome socket.
        SocketListen welcome_socket;
        ! Accept a connection on the host and port.
        SocketAccept welcome_socket, client_socket \Time:=WAIT_MAX;
        ! Close the welcome socket, as it is no longer needed.
        SocketClose welcome_socket;
    ENDPROC
    !----------------------------
    
    
    
    !----------------------------
    ! Close the connection to the client.
    PROC CloseConnection()
        SocketClose client_socket;
    ENDPROC
    !----------------------------
    
ENDMODULE