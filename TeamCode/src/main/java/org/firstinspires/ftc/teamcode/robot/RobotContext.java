package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.geometry.Pose;

/**
 * Holds shared robot state (not hardware) so autonomous and teleop can reuse values like last pose.
 */
public final class RobotContext {
    public static Pose lastPose;
    public static Integer lastSpindexerTicks;

    private RobotContext() {
        // Utility class
    }
}
