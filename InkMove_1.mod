MODULE InkMove
    ! Code for translating a set of trajectories into robot motion
    
    CONST num zTab := 147;
    
    VAR num aDrawPt1{59,5}:= [[350.798,-179.966,349.028,-180.007,0],[349.028,-180.007,346.150,-180.073,0],[346.150,-180.073,343.240,-180.139,0],[343.240,-180.139,340.982,-180.191,0],[340.982,-180.191,338.945,-180.238,0],[338.945,-180.238,336.908,-180.285,0],[336.908,-180.285,334.871,-180.332,0],[334.871,-180.332,332.834,-180.378,0],[332.834,-180.378,330.798,-180.425,0],[330.798,-180.425,328.761,-180.472,0],[328.761,-180.472,326.724,-180.519,0],[326.724,-180.519,324.687,-180.565,0],[324.687,-180.565,322.650,-180.612,0],[322.650,-180.612,320.613,-180.659,0],[320.613,-180.659,318.613,-180.695,0],[318.613,-180.695,316.818,-180.673,0],[316.818,-180.673,315.391,-180.545,0],[315.391,-180.545,314.448,-180.278,0],[314.448,-180.278,314.058,-179.854,0],[314.058,-179.854,314.244,-179.264,0],[314.244,-179.264,314.985,-178.515,0],[314.985,-178.515,316.209,-177.628,0],[316.209,-177.628,317.800,-176.636,0],[317.800,-176.636,319.597,-175.585,0],[319.597,-175.585,321.431,-174.523,0],[321.431,-174.523,323.264,-173.462,0],[323.264,-173.462,325.097,-172.400,0],[325.097,-172.400,326.931,-171.339,0],[326.931,-171.339,328.764,-170.277,0],[328.764,-170.277,330.597,-169.216,0],[330.597,-169.216,332.430,-168.155,0],[332.430,-168.155,334.263,-167.093,0],[334.263,-167.093,336.096,-166.032,0],[336.096,-166.032,337.929,-164.971,0],[337.929,-164.971,339.726,-163.920,0],[339.726,-163.920,341.319,-162.926,0],[341.319,-162.926,342.547,-162.034,0],[342.547,-162.034,343.296,-161.277,0],[343.296,-161.277,343.497,-160.673,0],[343.497,-160.673,343.125,-160.229,0],[343.125,-160.229,342.205,-159.939,0],[342.205,-159.939,340.806,-159.784,0],[340.806,-159.784,339.042,-159.730,0],[339.042,-159.730,337.075,-159.734,0],[337.075,-159.734,335.071,-159.748,0],[335.071,-159.748,333.068,-159.761,0],[333.068,-159.761,331.064,-159.775,0],[331.064,-159.775,329.061,-159.789,0],[329.061,-159.789,327.057,-159.803,0],[327.057,-159.803,325.054,-159.816,0],[325.054,-159.816,323.050,-159.830,0],[323.050,-159.830,321.046,-159.844,0],[321.046,-159.844,319.043,-159.858,0],[319.043,-159.858,317.039,-159.871,0],[317.039,-159.871,314.818,-159.887,0],[314.818,-159.887,311.955,-159.906,0],[311.955,-159.906,309.124,-159.926,0],[309.124,-159.926,307.383,-159.938,0],[307.383,-159.938,307.021,-159.940,0]];
    VAR num aDrawPt{59,2}:=[[350.798,-179.966],[349.028,-180.007],[346.150,-180.073],[343.240,-180.139],[340.982,-180.191],[338.945,-180.238],[336.908,-180.285],[334.871,-180.332],[332.834,-180.378],[330.798,-180.425],[328.761,-180.472],[326.724,-180.519],[324.687,-180.565],[322.650,-180.612],[320.613,-180.659],[318.613,-180.695],[316.818,-180.673],[315.391,-180.545],[314.448,-180.278],[314.058,-179.854],[314.244,-179.264],[314.985,-178.515],[316.209,-177.628],[317.800,-176.636],[319.597,-175.585],[321.431,-174.523],[323.264,-173.462],[325.097,-172.400],[326.931,-171.339],[328.764,-170.277],[330.597,-169.216],[332.430,-168.155],[334.263,-167.093],[336.096,-166.032],[337.929,-164.971],[339.726,-163.920],[341.319,-162.926],[342.547,-162.034],[343.296,-161.277],[343.497,-160.673],[343.125,-160.229],[342.205,-159.939],[340.806,-159.784],[339.042,-159.730],[337.075,-159.734],[335.071,-159.748],[333.068,-159.761],[331.064,-159.775],[329.061,-159.789],[327.057,-159.803],[325.054,-159.816],[323.050,-159.830],[321.046,-159.844],[319.043,-159.858],[317.039,-159.871],[314.818,-159.887],[311.955,-159.906],[309.124,-159.926],[307.383,-159.938]];
    
    CONST robtarget pZeros := [[0,0,0],[0,0,-1,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    
    VAR num startFlag:= 0;
    
    VAR num dataRecvFlag := 0;  ! is there new data to draw
    VAR num thickThin := 0;     ! is line thick of thin
    VAR num size := 0;          ! This is the size of the parsed trajectory before lifting
    VAR num DrawPt{500,2};      ! This is all continuous trajectory points 
    
    VAR num comma1;             ! to index between two ,
    VAR num comma2;
    
    VAR string input;           ! read in a string
    VAR bool mandatory;         ! otherwise StrToVal doesnt work
    VAR robtarget Pos1 := pZeros; 
    VAR num Table_upOff := zTab + 50;
    
    ! Data recieving 
    ! - first number of x y points 
    ! - second is thin or thick boolean
    ! - all other values are X, Y pos 
    
    PROC Main()
        TPWrite "-------------- Start";  
        
        startFlag := 1;

        ! drawing loop
        WHILE startFlag = 1 DO
        
            !============================
            ! Get Setup trajectory data 
            ! Get data from networking
            ! - about number of points 
            ! - thin or thick
            
            
            input := "60,1"; !350.798,-179.966,349.028,-180.007,346.150,-180.073"; !,343.240,-180.139,340.982,-180.191,338.945,-180.238,336.908,-180.285,334.871,-180.332,332.834,-180.378,330.798,-180.425,328.761,-180.472,326.724,-180.519,324.687,-180.565,322.650,-180.612,320.613,-180.659,318.613,-180.695,316.818,-180.673,315.391,-180.545,314.448,-180.278,314.058,-179.854,314.244,-179.264,314.985,-178.515,316.209,-177.628,317.800,-176.636,319.597,-175.585,321.431,-174.523,323.264,-173.462,325.097,-172.400,326.931,-171.339,328.764,-170.277,330.597,-169.216,332.430,-168.155,334.263,-167.093,336.096,-166.032,337.929,-164.971,339.726,-163.920,341.319,-162.926,342.547,-162.034,343.296,-161.277,343.497,-160.673,343.125,-160.229,342.205,-159.939,340.806,-159.784,339.042,-159.730,337.075,-159.734,335.071,-159.748,333.068,-159.761,331.064,-159.775,329.061,-159.789,327.057,-159.803,325.054,-159.816,323.050,-159.830,321.046,-159.844,319.043,-159.858,317.039,-159.871,314.818,-159.887,311.955,-159.906,309.124,-159.926,307.383,-159.938,307.021,-159.940";
            ! - first value is number of points 
            comma1 :=  1;
            comma2 :=  StrFind(input,comma1+1,",");
            mandatory := StrToVal(StrPart(input,comma1,comma2-1),size);
            ! - second value is thick or thin 
            comma1 :=  comma2;
            comma2 :=  StrFind(input,comma1+1,",");
            mandatory := StrToVal(StrPart(input,comma1+1,comma2 - comma1 -1),thickThin);
            !============================
            
            
            !============================
            ! Get trajectory data 
            ! populate an array. 2 at a time 
            ! FOR i FROM 1 TO size DO
            !--------
            ! testing
            input := "350.798,-179.966,349.028,-180.007,346.150,-180.073, 343.240,-180.139"; !,340.982,-180.191,338.945,-180.238,336.908,-180.285,334.871,-180.332,332.834,-180.378,330.798,-180.425,328.761,-180.472,326.724,-180.519,324.687,-180.565,322.650,-180.612,320.613,-180.659,318.613,-180.695,316.818,-180.673,315.391,-180.545,314.448,-180.278,314.058,-179.854,314.244,-179.264,314.985,-178.515,316.209,-177.628,317.800,-176.636,319.597,-175.585,321.431,-174.523,323.264,-173.462,325.097,-172.400,326.931,-171.339,328.764,-170.277,330.597,-169.216,332.430,-168.155,334.263,-167.093,336.096,-166.032,337.929,-164.971,339.726,-163.920,341.319,-162.926,342.547,-162.034,343.296,-161.277,343.497,-160.673,343.125,-160.229,342.205,-159.939,340.806,-159.784,339.042,-159.730,337.075,-159.734,335.071,-159.748,333.068,-159.761,331.064,-159.775,329.061,-159.789,327.057,-159.803,325.054,-159.816,323.050,-159.830,321.046,-159.844,319.043,-159.858,317.039,-159.871,314.818,-159.887,311.955,-159.906,309.124,-159.926,307.383,-159.938,307.021,-159.940";

            
            ! First number is differently indexed
            comma1 :=  1;
            comma2 :=  StrFind(input,comma1+1,",");
            mandatory := StrToVal(StrPart(input,comma1,comma2-1),DrawPt{1,1});
            comma1 :=  comma2;
            comma2 :=  StrFind(input,comma1+1,",");
            mandatory := StrToVal(StrPart(input,comma1+1,comma2 - comma1 -1),DrawPt{1,2});
            
            ! testing 4 values at a time
            FOR i FROM 2 TO 4 DO
                ! Get data from networking
                
                ! - feed values into array 
                comma1 :=  comma2;
                comma2 :=  StrFind(input,comma1+1,",");
                mandatory := StrToVal(StrPart(input,comma1+1,comma2 - comma1 -1),DrawPt{i,1});
                comma1 :=  comma2;
                comma2 :=  StrFind(input,comma1+1,",");
                mandatory := StrToVal(StrPart(input,comma1+1,comma2 - comma1 -1),DrawPt{i,2});
            ENDFOR
            !--------
            dataRecvFlag := 1;
            
            ! move to the starting position 
            
            Pos1.trans := [aDrawpt{1,1},aDrawpt{1,2},Table_upOff];
            ! Move just above the object
            MoveJ Offs(Pos1,0,0,0), v100, fine, tSCup;
            
            ! Begin Ink tracing 
            PrintLetter dataRecvFlag, thickThin; 
            
            startFlag := 0;
            
        ENDWHILE
        
        
        !============================
        ! ALL Done move back to home
        MoveToCalibPos;
        
        TPWrite "-------------- Done";
    ENDPROC

    PROC PrintLetter(num data_recv_flag, num thick_thin)
        VAR robtarget Pos1 := pZeros; 
        VAR num Table_upOff := zTab + 50;
        ! if data_recv_flag =  1 there is data -> move 
        IF data_recv_flag = 1 THEN             
            ! if 1 = thick line 
            IF thick_thin = 1 THEN 
                FOR i FROM 1 TO 4 DO
                    Pos1.trans := [aDrawpt{i,1},aDrawpt{i,2}, Table_upOff];
                    MoveL Pos1, v20, fine, tSCup;
                ENDFOR
            ELSEIF thick_thin = 2 THEN
               FOR i FROM 1 TO 4 DO
                    Pos1.trans := [aDrawpt{i,1},aDrawpt{i,2}, Table_upOff];
                    MoveL Pos1, v40, fine, tSCup;
                ENDFOR
            ENDIF
            
            ! A point just above the target 
            MoveJ Offs(Pos1,0,0,50), v100, fine, tSCup;
            ! MoveToCalibPos;
        ENDIF
    ENDPROC
    
ENDMODULE