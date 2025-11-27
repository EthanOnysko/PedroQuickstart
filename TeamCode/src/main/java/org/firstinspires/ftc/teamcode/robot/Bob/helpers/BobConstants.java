package org.firstinspires.ftc.teamcode.robot.Bob.helpers;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class BobConstants {

    public static double TICKS_PER_REV_SHOOTER = 28;

    // Shooter RPM presets
    public static int RPM_ZONE1 = 2500;
    public static int RPM_ZONE2 = 3000;
    public static int RPM_OFF = 0;

    // ==================== SPINDEXER ====================
    public static double TICKS_PER_REV_SPINDEXER = 8192;

    // Spindexer PID constants (normal mode)
//    public static double SPINDEX_KP = 0.001;
//    public static double SPINDEX_KI = 0.000001;
//    public static double SPINDEX_KD = 0.00011;
    public static double SPINDEX_KP = 0.00032;
    public static double SPINDEX_KI = 0;
    public static double SPINDEX_KD = 0.00005;

    // Spindexer PID constants (emergency mode)
    public static double SPINDEX_EMERGENCY_KP = 0.00032;
    public static double SPINDEX_EMERGENCY_KI = 0;
    public static double SPINDEX_EMERGENCY_KD = 0.00005;

    // Spindexer positions (in degrees)
    public static double SPINDEX_POSITION_0 = 0;
    public static double SPINDEX_POSITION_1 = 120;
    public static double SPINDEX_POSITION_2 = 240;
    public static double SPINDEX_HALF_STEP = 60;

    // Spindexer power limit (when in emergency mode)
    public static double SPINDEX_EMERGENCY_POWER_LIMIT = 0.6;

    // ==================== TRANSFER ====================
    public static double TRANSFER_UP = 0.3;
    public static double TRANSFER_DOWN = 0.1;
    public static double TRANSFER_PULSE_TIME = 300; // milliseconds

    // ==================== INTAKE ====================
    public static double INTAKE_POWER_IN = -0.6;
    public static double INTAKE_POWER_OUT = 0.6;
    public static double INTAKE_POWER_OFF = 0;

    // ==================== DRIVE ====================
    public static double DRIVE_SLOW_MODE_MULTIPLIER = 0.3;

    // ==================== LIGHT ====================
    public static double LIGHT_ON = 1.0;
    public static double LIGHT_OFF = 0.0;

    // ==================== MISC ====================
    public static int INFINITY = 2000000000;
}