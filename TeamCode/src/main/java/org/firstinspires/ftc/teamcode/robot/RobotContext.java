package org.firstinspires.ftc.teamcode.robot;

import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

/**
 * Holds shared robot instances so autonomous and teleop can reuse state such as last pose.
 */
public final class RobotContext {
    public static final Bob bob = new Bob();

    private RobotContext() {
        // Utility class
    }
}
