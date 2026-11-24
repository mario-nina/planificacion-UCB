"""Render an independent RAPID candidate for review in a virtual controller."""

from .planning import rapid_number, rapid_target

DRAFT_MODULE = "Imt342TrajectoryDraft"
SERVER_MODULE = "Imt342TrajectoryRapid"


def render_module(draft):
    record = draft.record()
    allow = "FALSE" if any(item.short_for_model_stop for item in draft.waypoints) else "TRUE"
    lines = [
        f"MODULE {DRAFT_MODULE}",
        "    ! EXPERIMENTAL OFFLINE - Compile and review only in a virtual controller.",
        "    ! No main, no RWS binding and no validated physical trajectory.",
        f'    CONST string trjPlanId := "{record["plan_id"]}";',
        "    CONST num trjDraftVersion := 1;",
        f"    CONST bool trjTimingReviewOK := {allow};",
        f"    CONST jointtarget trjStart := {rapid_target(record['start_deg'])};",
        "    PERS bool trjVirtualArmed := FALSE;",
        "    PERS bool trjCancelRequested := FALSE;",
        '    PERS string trjRunId := "";',
        "    PERS num trjState := 0;",
        "    PERS num trjLastWaypoint := 0;",
    ]
    lines.extend(
        f"    CONST jointtarget trjP{item.waypoint_id} := {rapid_target(item.target_deg)};"
        for item in draft.waypoints if not item.stationary
    )
    comparisons = (
        f"Abs(actual.robax.rax_{axis} - target.robax.rax_{axis}) <= 0.05"
        for axis in range(1, 7)
    )
    lines.extend([
        "",
        "    LOCAL FUNC bool trjAt(jointtarget target)",
        "        VAR jointtarget actual;",
        "        actual := CJointT();",
        "        RETURN " + " AND\n               ".join(comparisons) + ";",
        "    ENDFUNC",
        "",
        "    PROC RunTrajectoryDraft(string trialId)",
        "        IF RobOS() THEN",
        "            RETURN;",
        "        ENDIF",
        "        IF trjState <> 0 OR NOT trjVirtualArmed OR StrLen(trialId) = 0 THEN",
        "            RETURN;",
        "        ENDIF",
        "        trjVirtualArmed := FALSE;",
        "        trjRunId := trialId;",
        "        IF NOT trjTimingReviewOK OR NOT trjAt(trjStart) THEN",
        "            trjState := 5;",
        "            RETURN;",
        "        ENDIF",
        "        trjState := 1;",
    ])
    for item in draft.waypoints:
        name = f"trjP{item.waypoint_id}"
        lines.extend([
            "        IF trjCancelRequested THEN",
            "            trjState := 3;",
            "            RETURN;",
            "        ENDIF",
        ])
        if not item.stationary:
            lines.extend([
                f"        MoveAbsJ {name},v10\\T:={rapid_number(item.time_request_s)},fine,tool0;",
                "        WaitRob \\InPos;",
                f"        IF NOT trjAt({name}) THEN",
                "            trjState := 4;",
                "            RETURN;",
                "        ENDIF",
            ])
        lines.append(f"        trjLastWaypoint := {item.waypoint_id};")
    lines.extend([
        "        IF trjCancelRequested THEN",
        "            trjState := 3;",
        "        ELSE",
        "            trjState := 2;",
        "        ENDIF",
        "    ERROR",
        "        trjState := 4;",
        "        RAISE;",
        "    ENDPROC",
        "ENDMODULE",
    ])
    return "\n".join(lines) + "\n"
