MODULE ErrorHandling
    
    !DO_ESTOP 1 ready TO run 0 when needs resetting (green button to reset)
    !DO_ESTOP2 1 when emergency stopped (press white button to reset to 0)
    !DO_EXEC_ERR 0 rn? motion task not running, 
    !DO_HOLD_TO_ENABLE 0 rn, 1 when flex pendant held
    !DO_LIGHT_CURTAIN 1 when running, 0 when triggered
    !DO_MOTION_SUP_TRIG 0 rn, 1 when motion suspended
    !DO_MOTOR_ON_STATE 0rn, 1 when motor running
    !DO_TROB_RUNNING 0rn, 1 when robot moving
    
    VAR intnum sigLightCurtain_1;
    VAR intnum sigLightCurtain_0;
    VAR intnum sigEstop_1;
    VAR intnum sigEstop_0;
    VAR intnum sigEstop2_1;
    VAR intnum sigEstop2_0;
    VAR intnum sigMotionSuspended_1;
    VAR intnum sigConveyor_0;
    
    TRAP trapLightCurtain_1
        ! light curtain ready to run
        TPWrite "light curtain ready to run";
    ENDTRAP
    
    TRAP trapLightCurtain_0
        ! light curtain TRIGGERED, needs resetting
        TPWrite "light curtain TRIGGERED, needs resetting";
        SendMessage "LIGHTCUR";
    ENDTRAP
    
    TRAP trapEstop_1
        ! estop ready to run
        TPWrite "estop ready to run";
    ENDTRAP
    
    TRAP trapEstop_0
        ! estop TRIGGERED, needs resetting
        TPWrite "estop triggered, needs resetting";
        SendMessage "ESTOP";
    ENDTRAP
    
    TRAP trapEstop2_1
        ! estop2 TRIGGERED (press white button to reset to 0
        TPWrite "estop2 triggered, needs resetting";
        SendMessage "ESTOP2";
    ENDTRAP
    
    TRAP trapEstop2_0
        ! estop2 ready to run
        TPWrite "estop2 ready to run";
    ENDTRAP
    
    TRAP flexPendantHold_0
        ! flex pendant not held down
        TPWrite "flex pendant not held";
        SendMessage "FLEXP";
    ENDTRAP
    
    TRAP flexPendantHold_1
        ! flex pendant held down
        TPWrite "flex pendant held";
    ENDTRAP
    
    TRAP trapMotionSuspended_1
        ! motion suspended triggered!
        TPWrite "motion suspension triggered";
        SendMessage "MOTIONSUS";
    ENDTRAP
    
    TRAP trapMotionSuspended_0
    ENDTRAP
    
    TRAP trapConveyor_1
    ENDTRAP
    
    TRAP trapConveyor_0
        ! conveyor interlock triggered!
        TPWrite "Conveyor Open!";
        SendMessage "CONVEY";
    ENDTRAP
    
    
    !DO_EXEC_ERR 0 rn? motion task not running, 
    TRAP execErr_0
        ! don't quite know what this is...        
    ENDTRAP
    
    TRAP execErr_1
        ! don't know hwat this is either...
    ENDTRAP
    
    PROC SendMessage(String MSG)
        SocketSend client_socket \Str:=(MSG + "\0A");
    ENDPROC
        
    PROC SetupInterrupts()
        
        CONNECT sigLightCurtain_1 WITH trapLightCurtain_1;
        ISignalDO DO_LIGHT_CURTAIN,1,sigLightCurtain_1;
        CONNECT sigLightCurtain_0 WITH trapLightCurtain_0;
        ISignalDO DO_LIGHT_CURTAIN,0,sigLightCurtain_0;
        CONNECT sigEstop_1 WITH trapEstop_1;
        ISignalDO DO_ESTOP,1,sigEstop_1;
        CONNECT sigEstop_0 WITH trapEstop_0;
        ISignalDO DO_ESTOP,1,sigEstop_0;
        CONNECT sigEstop2_1 WITH trapEstop2_1;
        ISignalDO DO_ESTOP2,1,sigEstop2_1;
        CONNECT sigEstop2_0 WITH trapEstop2_0;
        ISignalDO DO_ESTOP2,1,sigEstop2_0;
        CONNECT sigMotionSuspended_1 WITH trapMotionSuspended_1;
        ISignalDO DO_MOTION_SUP_TRIG,1,sigMotionSuspended_1;
        CONNECT sigConveyor_0 WITH trapConveyor_0;
        ISignalDI DI10_1,0,sigConveyor_0;
        
    ENDPROC
    
!    PROC SendUpdate()
!        SocketSend client_socket \Str:=(
!            "STT: " +
!            NumToStr(DOutput(DO_ESTOP), 0) +
!            NumToStr(DOutput(DO_ESTOP2), 0) +
!            NumToStr(DOutput(DO_EXEC_ERR), 0) +
!            NumToStr(DOutput(DO_HOLD_TO_ENABLE), 0) +
!            NumToStr(DOutput(DO_LIGHT_CURTAIN), 0) +
!            NumToStr(DOutput(DO_MOTION_SUP_TRIG), 0) +
!            NumToStr(DOutput(DO_MOTOR_ON_STATE), 0) +
!            NumToStr(DOutput(DO_TROB_RUNNING), 0) + 
!            "\0A");
        
!!        estop_ready := DOutput(DO_ESTOP);
!!        estop_trigg := DOutput(DO_ESTOP2);
!!        exec_error := DOutput(DO_EXEC_ERR);
!!        flex_held := DOutput(DO_HOLD_TO_ENABLE);
!!        light_c_ready := DOutput(DO_LIGHT_CURTAIN);
!!        motion_suspended :=DOutput(DO_MOTION_SUP_TRIG);
!!        motor_on := DOutput(DO_MOTOR_ON_STATE);
!!        motor_running := DOutput(DO_TROB_RUNNING);      
!    ENDPROC    
    
ENDMODULE