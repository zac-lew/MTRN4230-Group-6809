MODULE MattServer   
    VAR string In_str;
    VAR string Out_str;
    VAR string received_str;
    
    ! The socket connected to the client.
    VAR socketdev client_socket;
    ! The host and port that we will be listening for a connection on.
    PERS string host := "127.0.0.1";
    CONST num port := 1025;
    
    PERS num inkData{2};
    PERS num inkSize;
    VAR num inkThick := 0;
    PERS num DrawPt{500,2};
    VAR num Zeros500{500,2};
    VAR num arr5{5};
    VAR num count := 1;
    VAR bool check := FALSE;
    
    PROC Main ()
        TPWrite ">>>>>>S>>>>>>";
        In_str := "0";
        Out_str := "0";
        
        IF RobOS() THEN
            host := "192.168.125.1";
        ELSE
            host := "127.0.0.1";
        ENDIF
        MainServer;
        TPWrite ">>>>>>F>>>>>>";
    ENDPROC

    PROC MainServer()
        MoveToHomePos;
        ListenForAndAcceptConnection;
        WHILE TRUE DO
            WaitForMessage;
        ENDWHILE
        CloseConnection;
    ENDPROC

    PROC WaitForMessage()
		TPWrite ">>>>>>L>>>>>>  " + "In_str: " + In_str + "   Out_str: " + Out_str; 
	
		IF Out_str <> "0" THEN
			SendOutStr;
		ENDIF
		
		IF In_str = "0" THEN
			GetInStr;

			TEST In_str
			CASE "PNP": ! Pick and Place
				SendACK;
				TPWrite "Read PNP";
                ! Making sure not red badly
                check := FALSE;
                WHILE check = FALSE DO
                    In_Str := "0";
                    GetInStr;
                    TPWrite "In_str: " + In_str;
                    ok := StrToVal(In_str,arr5);
                    IF arr5{1}<>0 AND arr5{2}<>0 AND arr5{3}<>0 AND arr5{4}<>0 AND arr5{5}<>0 THEN
                        TPWrite "Good: " + In_str + ", " + ValToStr(arr5{1}) + " " + ValToStr(arr5{2}) + " " + ValToStr(arr5{3}) + " " + ValToStr(arr5{4}) + " " + ValToStr(arr5{5});
                        check := TRUE;
                    ELSE
                        TPWrite "Bad: " + In_str + ", "  + ValToStr(arr5{1}) + " " + ValToStr(arr5{2}) + " " + ValToStr(arr5{3}) + " " + ValToStr(arr5{4}) + " " + ValToStr(arr5{5});
                    ENDIF
                ENDWHILE
                SendACK;
				MattMain;
				
			CASE "INK": ! Ink print
				SendACK;
				TPWrite "Read INK";
				DrawPt := Zeros500;
				inkData := [0,0];
				inkSize := 0;
				count := 1;
				
				In_str := "0";
				GetInStr;
				ok := StrToVal(In_str, inkData);
				inkSize := inkData{1};
				inkThick := inkData{2};
				
				WHILE inkSize <= 0 DO ! Should be skipped if obtained correctly
					FlushSocket;
					In_str := "0";
					GetInStr;
					ok := StrToVal(In_str, inkData);
					inkSize := inkData{1};
					inkThick := inkData{2};
				ENDWHILE
				SendACK;
				
				TPWrite "Size: " + ValToStr(inkSize) + " " + "thickThin: " + ValToStr(inkThick);
				
				! Loading stroke data 
				WHILE count <= inkSize DO
					GetInStrACK;
					IF In_str <> "0" THEN     
						ok := StrToVal(In_str, inkData);    ! - feed values into array 
						!TPWrite "cnt: " + ValToStr(count) + ", Ink1: " + ValToStr(inkData{1}) + ", Ink2: " + ValToStr(inkData{2});
						DrawPt{count,1} := inkData{1};      ! - first value is number of points 
						DrawPt{count,2} := inkData{2};      ! - second value is thick or thin 
						count := count + 1;
						In_str := "0";
					ENDIF
				ENDWHILE
				
				! Begin Ink tracing 
				IF inkSize > 0 THEN                        
					PrintLetter inkThick;
					TPWrite "Finished INK";
					Out_Str := "DONE";
					SendOutStr;
				ENDIF
				   
			
			CASE "CON": ! Conveyor On
				SendACK;
				TPWrite "Read CON";
				In_Str := "0";
				TurnConOn;
				Out_Str := "DONE";
			
			CASE "COF": ! Conveyor Off
				SendACK;
				TPWrite "Read COF";
				In_Str := "0";
				TurnConOff;
				Out_Str := "DONE";
				
			CASE "CFW": ! Conveyor Forward
				SendACK;
				TPWrite "Read CFW";
				In_Str := "0";
				ConForward;
				Out_Str := "DONE";
			
			CASE "CBK": ! Conveyor Backward
				SendACK;
				TPWrite "Read CBK";
				In_Str := "0";
				ConBackward;
				Out_Str := "DONE";
                
            CASE "HOM": ! Move to home position
				SendACK;
				TPWrite "Read HOM";
				In_Str := "0";
				MoveToHomePos;
				Out_Str := "DONE";

			DEFAULT:
				! FlushSocket;
				In_Str := "0";
			ENDTEST
		ENDIF       
			
		ERROR
        IF ERRNO = ERR_SOCK_CLOSED THEN
            SocketClose client_socket;
            ListenForAndAcceptConnection;
            RETRY;
        ELSEIF ERRNO = ERR_SOCK_TIMEOUT THEN
            SocketClose client_socket;
            ListenForAndAcceptConnection;
            RETRY;
        ENDIF
    ENDPROC
    
    PROC GetInStrACK()
        GetInStr;
        SendACK;
    ENDPROC
    
    PROC FlushSocket()
        SocketReceive client_socket \Str:=received_str;
        TPWrite "Flush: " + received_str;
        received_str := "0";
        In_str := "0";
    ENDPROC
    
    PROC GetInStr()
        ! Receive a string from the client.
        SocketReceive client_socket \Str:=received_str;
        In_str := received_str;
        TPWrite "Got new string: " + In_str;
    ENDPROC
    
    PROC SendACK()
        SocketSend client_socket \Str:=("ACK" + "\0A");  
    ENDPROC
    
    
    PROC SendOutStr()
        TPWrite "Sending string: " + Out_str;
        ! Send the string back to the client, adding a line feed character.
        SocketSend client_socket \Str:=(Out_str + "\0A");
        Out_str := "0";
    ENDPROC
    
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
    
    ! Close the connection to the client.
    PROC CloseConnection()
        SocketClose client_socket;
    ENDPROC
    

ENDMODULE