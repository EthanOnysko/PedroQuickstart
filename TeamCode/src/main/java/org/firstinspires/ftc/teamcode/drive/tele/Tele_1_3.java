package org.firstinspires.ftc.teamcode.drive.tele;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.robot.Bob.Bob;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.INTAKE_POWER_IN;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.BALL_PROX;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_RIGHT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.Macros.SPINDEXER_SIXTY;

import java.util.Deque;
import java.util.LinkedList;
@TeleOp(name = "Judging Interview")
public class Tele_1_3 extends OpMode {

    Bob bob = new Bob();

    TelemetryManager telemetryM;

    Limelight3A limelight;
    private Timer macroTimer, actionTimer, opmodeTimer;
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    private PID rotationPID = new PID(2,0,.05);
    private boolean rotationCorrectionOn = false;
    private double rotationPower = 0;
    private double currentAngle = 0;
    private boolean transfer = false;
    private boolean intake = false;
    private double rotationTarget = 0;
    private double rotationErrorThresh = 0.05;
    private double rotationDerivativeThresh = 0.1;

    // teleOp State
    private boolean intakeOn = false;
    private boolean isZoneOne = true;
    private boolean isMacroing = false;
    private int numBalls = 0;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        bob.init(hardwareMap);
        rotationPID.init(0);
        rotationPID.setTarget(rotationTarget);
        rotationPID.setDoneThresholds(rotationErrorThresh, rotationDerivativeThresh);
        actionTimer = new Timer();
        macroTimer = new Timer();
    }

    @Override
    public void start() {
        bob.transferController.setDown();
        limelight.start();
        macroTimer.resetTimer();
    }

    @Override
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;

        bob.updateLight(numBalls);
        //TODO: GAMEPAD1 CONTROLS (DRIVER)

        if (intakeOn && numBalls < 3) bob.intakeController.intake();
        else bob.intakeController.stopIntake();

        if (gamepad2.right_bumper && !lastGamepad2.right_bumper) {
            intakeOn = !intakeOn;
        }
        if (bob.getProx() < BALL_PROX &&
                bob.intakeController.getIntake() == INTAKE_POWER_IN &&
                actionTimer.getElapsedTimeSeconds() > .5 &&
                numBalls < 3
        ){
            numBalls++;
            actionTimer.resetTimer();
            bob.runMacro(SPINDEXER_RIGHT);
        }

        if (numBalls == 3){
            numBalls = 4;
            bob.runMacro(SPINDEXER_SIXTY);
        }

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

}