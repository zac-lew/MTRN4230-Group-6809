MODULE KenjiCOM
    VAR string In_str;
    VAR string Out_str;
    VAR socketdev client_socket;
    VAR string received_str;
    PERS string hostCOM;
    CONST num port := 1026;
             
    PROC Main()
        
        IF RobOS() THEN
            hostCOM := "192.168.125.1";
            TPWrite "using 192.";
        ELSE
            hostCOM := "127.0.0.1";
            TPWrite "using 127.";
        ENDIF
        
        ListenForAndAcceptConnection;
        
        InteruptMain;
        
        WHILE TRUE DO 
            WaitTime 1;
        ENDWHILE
        
        CloseConnection;
        
    ENDPROC
ENDMODULE

