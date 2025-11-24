package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDSpindexer {

    // PID gains
    private double kP, kI, kD;

    // Encoder geometry
    private final double ticksPerRev;

    // Raw target ticks (can be any double, not wrapped)
    private double targetTicks = 0;

    // PID state
    private double integralSum = 0;
    private double lastError = 0;

    private Double lastAngleDeg = null;

    private final ElapsedTime timer = new ElapsedTime();

    private static final double maxIntegral = 2000;

    // --- Constructors ---

    public PIDSpindexer(double ticksPerRev) {
        this(ticksPerRev, 0, 0, 0);
    }

    public PIDSpindexer(double ticksPerRev, double kP, double kI, double kD) {
        this.ticksPerRev = ticksPerRev;
        setConsts(kP, kI, kD);
        reset(0);
    }

    // --- Config ---

    public void setConsts(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Reset PID state and lock target to the current position.
     */
 
    public void reset(double currentTicks) {
        integralSum = 0;
        lastError = 0;
        targetTicks = currentTicks;
        lastAngleDeg = null;
        timer.reset();
    }


    /**
     * Set the target using an angle in degrees (0–360 input).
     * This chooses the RAW target tick value (base + k * ticksPerRev)
     * that is CLOSEST to the currentTicks (shortest path around the circle).
     *
     * Examples with ticksPerRev = 100:
     *  angle=90  -> base=25
     *  angle=270 -> base=75
     *  if current=75 and angle=0:
     *     base=0, closest is 100 (k=+1), so targetTicks=100.
     */
    public void setTargetAngle(double angleDeg, double currentTicks) {

        if (lastAngleDeg == null || Math.abs(angleDeg - lastAngleDeg) > 1e-3) {
            lastAngleDeg = angleDeg;

            // base position for this angle (same for every revolution)
            double baseTicks = (angleDeg / 360.0) * ticksPerRev;

            // choose k so that baseTicks + k*ticksPerRev is closest to currentTicks
            double k = Math.rint((currentTicks - baseTicks) / ticksPerRev);
            targetTicks = baseTicks + k * ticksPerRev;
        }
    }

    /**
     * Optional: set target directly in raw ticks.
     */
    public void setTargetTicks(double ticks) {
        targetTicks = ticks;
    }

    // --- Getters for telemetry ---

    /** Raw target ticks (the exact value PID is chasing). */
    public double getTargetTicks() {
        return targetTicks;
    }

    /** Target angle in [0, 360) based on targetTicks. */
    public double getTargetAngle() {
        double wrapped = targetTicks % ticksPerRev;
        if (wrapped < 0) wrapped += ticksPerRev;
        return (wrapped / ticksPerRev) * 360.0;
    }

    // --- Main PID update ---

    /**
     * Call every loop with the current raw encoder ticks.
     * Returns a power in [-1, 1] for the CR servo.
     */
    public double update(double currentTicks) {
        double dt = timer.seconds();
        timer.reset();
        if (dt <= 0) dt = 1e-3;

        double error = targetTicks - currentTicks; // **RAW** error in ticks

        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -maxIntegral, maxIntegral);

        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kP * error + kI * integralSum + kD * derivative;

        return Range.clip(output, -1.0, 1.0);
    }
}
