MODULE Imt342JointServer
    RECORD ImtJointOrder
        num seq;
        jointtarget target;
    ENDRECORD

    RECORD ImtJointReply
        num seq;
        num phase;
    ENDRECORD

    CONST num rbVersion := 1;
    PERS ImtJointOrder pcOrder := [0,[[0,0,0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]]];
    PERS ImtJointReply rbReply := [0,0];

    ! Select this routine explicitly after configuring the lab motion task.
    ! After a stop, relocate PP here before Start. Never Continue a stopped MoveAbsJ.
    PROC ServeJointOrders()
        VAR ImtJointOrder requested;
        ! Discard unresolved commands on PP restart. Preserve a terminal result.
        IF pcOrder.seq <> rbReply.seq OR rbReply.phase = 1 OR rbReply.phase = 2 THEN
            rbReply := [pcOrder.seq,5];
        ENDIF
        WHILE TRUE DO
            requested := pcOrder;
            IF requested.seq > rbReply.seq THEN
                rbReply := [requested.seq,1];
                RunJointOrder requested;
            ENDIF
            WaitTime 0.05;
        ENDWHILE
    ENDPROC

    LOCAL PROC RunJointOrder(ImtJointOrder requested)
        rbReply := [requested.seq,2];
        MoveAbsJ requested.target,v10,fine,tool0;
        WaitRob \InPos;
        rbReply := [requested.seq,3];
    ERROR
        rbReply := [requested.seq,4];
        RAISE;
    ENDPROC
ENDMODULE
