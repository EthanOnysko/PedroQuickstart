package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;

import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.INTAKE_IN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.INTAKE_OUT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.INTAKE_STOP;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_OFF;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE1_MATIC;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOTER_ZONE2;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SHOOT_ALL_THREE;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_LEFT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_RIGHT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_SIXTY;

import java.util.Deque;
import java.util.LinkedList;

@TeleOp(name = "RUN THIS TeleOp")
public class MacroTeleOp extends OpMode {

    Bob bob = new Bob();
    Limelight3A limelight;

    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();

    // Limelight rotation PID
    private PID rotationPID = new PID(.5,0,.05);
    private boolean rotationCorrectionOn = false;
    private double rotationPower = 0;
    private double currentAngle = 0;

    private double rotationTarget = 0; // Target angle offset (0 = center on target)
    private double rotationErrorThresh = 0.05; // Radians
    private double rotationDerivativeThresh = 0.1;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0);
        bob.init(hardwareMap);
        rotationPID.init(0);
        rotationPID.setTarget(rotationTarget);
        rotationPID.setDoneThresholds(rotationErrorThresh, rotationDerivativeThresh);
    }

    @Override
    public void start() {
        bob.transferController.setDown();
        bob.spindexerController.updateLight();
        limelight.start();

    }

    @Override
    public void loop() {
        // Safety check
        if (gamepad2.start || gamepad1.start) return;

        updateRotationCorrection();

        //TODO: GAMEPAD1 CONTROLS (DRIVER)

        // Drive controls
        if (!gamepad1.right_bumper && gamepad1.right_trigger <= 0.1) {
            // Normal driving
            bob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_bumper) {
            // Slow mode
            bob.motorDriveXYVectors(0.7 * -gamepad1.left_stick_x, 0.7 * gamepad1.left_stick_y, 0.3 * -gamepad1.right_stick_x);
            rotationCorrectionOn = false;
        }
        else if (gamepad1.right_trigger > 0.1) {
            // Limelight auto-rotation mode - hold right trigger to align with target
            rotationCorrectionOn = true;
            bob.motorDriveXYVectors(-gamepad1.left_stick_x, gamepad1.left_stick_y, rotationPower);

            // Haptic feedback when aligned
            if (rotationPID.isFinished()) {
                gamepad1.rumble(100);
            }
        }

        //shooting all 3 balls
        if (gamepad1.y && !lastGamepad1.y) bob.runMacro(SHOOT_ALL_THREE);
        if (gamepad1.left_bumper && !lastGamepad1.left_bumper) bob.runMacro(SHOOTER_ZONE1_MATIC);

        //intake in
        if (gamepad1.a) bob.intakeController.intake();
        else if (gamepad1.b) bob.intakeController.outtake();
        else bob.intakeController.stopIntake();

        //we got ball into spindexer
        if (gamepad1.x && !lastGamepad1.x) bob.runMacro(SPINDEXER_RIGHT);

        // TODO: GAMEPAD2 CONTROLS (GUNNER)
        //zone 1
        if (gamepad2.a && !lastGamepad2.a) bob.runMacro(SHOOTER_ZONE1);
        //zone 2
        if (gamepad2.y && !lastGamepad2.y) bob.runMacro(SHOOTER_ZONE2);

        //stop shooter
        if (gamepad2.b && !lastGamepad2.b) bob.runMacro(SHOOTER_OFF);
        // spin 120
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) bob.runMacro(SPINDEXER_RIGHT);
        // spin -120
        if (gamepad2.dpad_left && !lastGamepad2.dpad_left) bob.runMacro(SPINDEXER_LEFT);
        // spin 60
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) bob.runMacro(SPINDEXER_SIXTY);

        bob.tick();

        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);

        if (gamepad1History.size() > 100) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }

        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    private void updateRotationCorrection() {
        if (rotationCorrectionOn) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                // Get horizontal angle offset from limelight (tx)
                currentAngle = Math.toRadians(result.getTx());
                rotationPower = rotationPID.tick(currentAngle);

            } else {
                // No valid target detected
                rotationPower = 0;
            }
        } else {
            rotationPower = 0;
        }

        telemetry.update();
    }
}