package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.*;

public class Macros {

    // reset to intake mode
    public static final BobState SHOOT_ALL_THREE_FINAL = new BobState(0, 60.0, null, null, null);
    // shoot 3 ball
    public static final BobState SHOOT_ALL_THREE8 = new BobState(null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE_FINAL));
    public static final BobState SHOOT_ALL_THREE7 = new BobState(null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE8));
    // spin to next ball
    public static final BobState SHOOT_ALL_THREE6 = new BobState(null, 120.0, null, null, new LinkedState(Link.LinkType.WAIT, 400, SHOOT_ALL_THREE7));
    //shoot 2 ball
    public static final BobState SHOOT_ALL_THREE5 = new BobState(null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE6));
    public static final BobState SHOOT_ALL_THREE4 = new BobState(null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE5));
    // spin to next ball
    public static final BobState SHOOT_ALL_THREE3 = new BobState(null, 120.0, null, null, new LinkedState(Link.LinkType.WAIT, 400, SHOOT_ALL_THREE4));
    // shoot 1 ball
    public static final BobState SHOOT_ALL_THREE2 = new BobState(null, null, TRANSFER_DOWN, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE3));
    public static final BobState SHOOT_ALL_THREE = new BobState(null, null, TRANSFER_UP, null, new LinkedState(Link.LinkType.WAIT, 260, SHOOT_ALL_THREE2));

    // spin shooter zone1
    public static final BobState SHOOTER_ZONE1 = new BobState(RPM_ZONE1, null, null);
    //spin zone1 w/ shooter mode
    public static final BobState SHOOTER_ZONE2_MATIC = new BobState(RPM_ZONE2, 60.0, null);
    public static final BobState SHOOTER_ZONE1_MATIC = new BobState(RPM_ZONE1, 60.0, null);

    // spin shooter zone2
    public static final BobState SHOOTER_ZONE2 = new BobState(RPM_ZONE2, null, null);

    // shooter off
    public static final BobState SHOOTER_OFF = new BobState(RPM_OFF, null, null);

    //spin right
    public static final BobState SPINDEXER_RIGHT = new BobState(null, 120.0, null);
    //spin left
    public static final BobState SPINDEXER_LEFT = new BobState(null, -120.0, null);
    //toggle shooter vs. intake
    public static final BobState SPINDEXER_SIXTY = new BobState(null, 60.0, null);

    //intake
    public static final BobState INTAKE_IN = new BobState(null, null, null, INTAKE_POWER_IN, null);
    //outtake
    public static final BobState INTAKE_OUT = new BobState(null, null, null, INTAKE_POWER_OUT, null);
    //stop intake
    public static final BobState INTAKE_STOP = new BobState(null, null, null, INTAKE_POWER_OFF, null);


    public static final BobState READY_TO_SHOOT_ZONE1 = new BobState(
            RPM_ZONE1,
            null,
            TRANSFER_DOWN,
            INTAKE_POWER_OFF,
            null
    );

    public static final BobState READY_TO_SHOOT_ZONE2 = new BobState(
            RPM_ZONE2,
            null,
            TRANSFER_DOWN,
            INTAKE_POWER_OFF,
            null
    );

    public static final BobState COLLECTING = new BobState(
            RPM_OFF,
            null,
            TRANSFER_DOWN,
            INTAKE_POWER_IN,
            null
    );

    public static final BobState IDLE = new BobState(
            RPM_OFF,
            null,
            TRANSFER_DOWN,
            INTAKE_POWER_OFF,
            null
    );

    // ==================== AUTO SEQUENCES ====================

    // Declare these first so we can reference them
    public static BobState SHOOT_ZONE1_TRANSFER;
    public static BobState SHOOT_ZONE2_TRANSFER;

    // Example: Shoot sequence (spin up -> wait -> transfer)
    public static final BobState SHOOT_ZONE1_START = new BobState(
            RPM_ZONE1,
            null,
            TRANSFER_DOWN,
            INTAKE_POWER_OFF,
            new LinkedState(Link.LinkType.WAIT, 1000, getShootZone1Transfer())
    );

    private static BobState getShootZone1Transfer() {
        if (SHOOT_ZONE1_TRANSFER == null) {
            SHOOT_ZONE1_TRANSFER = new BobState(
                    RPM_ZONE1,
                    null,
                    TRANSFER_UP,
                    INTAKE_POWER_OFF,
                    new LinkedState(Link.LinkType.WAIT, 300, IDLE)
            );
        }
        return SHOOT_ZONE1_TRANSFER;
    }


    private static BobState SHOOT_ZONE2_START2() {
        if (SHOOT_ZONE2_TRANSFER == null) {
            SHOOT_ZONE2_TRANSFER = new BobState(
                    RPM_ZONE2,
                    null,
                    TRANSFER_UP,
                    INTAKE_POWER_OFF,
                    new LinkedState(Link.LinkType.WAIT, 300, IDLE)
            );
        }
        return SHOOT_ZONE2_TRANSFER;
    }
}