package org.firstinspires.ftc.teamcode.pedroPathing;


//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.PIDSpindexer;


import java.util.Deque;
import java.util.LinkedList;

@TeleOp
public class REALScrimTele extends OpMode {
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();
    // Dashboard-tunable things
    public static double targetAngle = 0;          // 0–360 input
    public static double TICKS_PER_REV = 8192;     // your encoder
    public static double TICKS_PER_REV_S = 28;
    public static double kP = 0.001, kI = .000001, kD = .00011;

    public static double ekP = 0.00032, ekI = 0, ekD = 0.00005;
    private Telemetry tele;

    private CRServo spindexer;
    private DcMotorEx spincoder;

    private PIDSpindexer spinPID;
    private int RPM = 0;
    double velocity;
    long transferTimer = 0;
    boolean transferPulse = false;
    private Servo light;
    public static int lightTresh = 150;

    private int zone1 = 2500;
    private int zone2 = 3000;

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor intake;
    DcMotorEx shooterRight;
    DcMotorEx shooterLeft;
    Servo transfer;

    private double transferUnder = 0.1;
    private double transferUp = 0.3;
    double offset = 0;
    boolean toggleState = false;
    boolean wasButtonPressed = false;
    boolean toggleState2 = false;
    boolean wasButtonPressed2 = false;
    public static double pulseTime = 300;

    @Override
    public void init() {
        //tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        tele = telemetry;
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");


        transfer = (Servo) hardwareMap.servo.get("transfer");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        light = hardwareMap.servo.get("light");

        // Using a drive motor as encoder in your example
        spincoder = hardwareMap.get(DcMotorEx.class, "bl");
//        spincoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        spincoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create PID with current TPR and starting gains
        spinPID = new PIDSpindexer(TICKS_PER_REV, kP, kI, kD);
        spinPID.reset(0); // encoder was just reset to 0

    }

    @Override
    public void start() {
        transfer.setPosition(transferUnder);
        if (targetAngle % 120 == 0) {
            light.setPosition(1);
        } else {
            light.setPosition(0);
        }
        spinPID.setConsts(kP, kI, kD);
    }

    @Override
    public void loop() {
        // Update PID gains live from Dashboard
        if (gamepad2.start || gamepad1.start) return;
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (!gamepad1.right_bumper) {
            motorFrontLeft.setPower(y + x + rx);
            motorBackLeft.setPower(y - x + rx);
            motorFrontRight.setPower(y - x - rx);
            motorBackRight.setPower(y + x - rx);
        } else {
            motorFrontLeft.setPower(0.3 * (y + x + rx));
            motorBackLeft.setPower(0.3 * (y - x + rx));
            motorFrontRight.setPower(0.3 * (y - x - rx));
            motorBackRight.setPower(0.3 * (y + x - rx));
        }

        if (gamepad2.y) {
            RPM = zone2;
        } else if (gamepad2.x) {
            RPM = zone1;
        } else if (gamepad2.b) {
            RPM = 0;
        }

        if (gamepad1.a) {
            intake.setPower(-.6);
        } else if (gamepad1.b) {
            intake.setPower(.6);
        } else {
            intake.setPower(0);
        }

        velocity = RPMtoVelocity(RPM);
        shooterLeft.setVelocity(velocity);
        shooterRight.setVelocity(velocity);

        double currentRPM1 = (shooterRight.getVelocity()/TICKS_PER_REV_S)*60.0;
        double currentRPM2 = (shooterLeft.getVelocity()/TICKS_PER_REV_S)*60.0;

        tele.addData("CurrRPM1", currentRPM1);
        tele.addData("CurrRPM2", currentRPM1);


        if (Math.abs(RPM-((currentRPM1+currentRPM2)/2)) < lightTresh && RPM > 1) {
            light.setPosition(.5);
        } else {
            if (targetAngle % 120 == 0) light.setPosition(1);
            else light.setPosition(0);
        }

        /*if (gamepad1.left_bumper) {
            if (!wasButtonPressed) {
                toggleState = !toggleState;
                wasButtonPressed = true;
            }
        } else {
            wasButtonPressed = false;
        }

        if (toggleState) {
            transfer.setPosition(transferUp);
        } else {
            transfer.setPosition(transferUnder);
        }*/


        if (gamepad1.left_bumper && !wasButtonPressed) {
            transferPulse = true;
            transferTimer = System.currentTimeMillis();
            transfer.setPosition(transferUp);
            wasButtonPressed = true;
        } else if (!gamepad1.left_bumper) {
            wasButtonPressed = false;
        }

        if (transferPulse) {
            if (System.currentTimeMillis() - transferTimer > pulseTime) { // 150 ms pulse
                transfer.setPosition(transferUnder);
                transferPulse = false;
            }
        }

        if (gamepad2.dpad_down) {
            if (!wasButtonPressed2) {
                toggleState2 = !toggleState2;
                wasButtonPressed2 = true;
            }
        } else {
            wasButtonPressed2 = false;
        }

        if (toggleState2) {
            spinPID.setConsts(kP, kI, kD);
        } else {
            spinPID.setConsts(ekP, ekI, ekD);
        }


        // bomba


        double currentTicks = spincoder.getCurrentPosition();

        //if (transfer.getPosition() == transferUnder) {
        if (gamepad2.dpad_right && !lastGamepad2.dpad_right) {
            targetAngle += 120;
        }
        if (gamepad2.dpad_left && !lastGamepad2.dpad_left) {
            targetAngle -= 120;
        }
        if (gamepad2.dpad_up && !lastGamepad2.dpad_up) {
            if (targetAngle % 120 == 0) light.setPosition(1);
            else light.setPosition(0);
            targetAngle += 60;
        }
        // }

        spinPID.setTargetAngle(targetAngle, currentTicks);

        double power = -spinPID.update(currentTicks);
        if (power > 0.1 || power < -0.1 && toggleState2) {
            spindexer.setPower(power * 0.6);
        } else if (!toggleState2) {
            spindexer.setPower(power);
        }
        // spindexer.setPower(power);
        // Compute a nice 0–360 angle from raw ticks for display
        double wrappedTicks = currentTicks % TICKS_PER_REV;
        if (wrappedTicks < 0) wrappedTicks += TICKS_PER_REV;
        double currentAngle = (wrappedTicks / TICKS_PER_REV) * 360.0;

        // Telemetry: raw + angles + error + power
        double targetTicks = spinPID.getTargetTicks();
        double targetAngleWrapped = spinPID.getTargetAngle();
        double errorTicks = targetTicks - currentTicks;

        tele.addData("kP", kP);
        tele.addData("kI", kI);
        tele.addData("kD", kD);

        tele.addData("Power", power);

        tele.addData("Target Angle (cmd)", targetAngle);        // what you typed in
        tele.addData("Target Angle (wrapped)", targetAngleWrapped); // based on targetTicks

        tele.addData("Current Angle", currentAngle);

        tele.addData("Target Ticks (raw)", targetTicks);
        tele.addData("Current Ticks (raw)", currentTicks);
        tele.addData("Error Ticks", errorTicks);

        tele.addData("CurrRPM", RPM);
        tele.addData("thresh", Math.abs(RPM-(currentRPM1+currentRPM2)/2));
        tele.update();
        gamepad1History.add(gamepad1);
        gamepad2History.add(gamepad2);
        // delete everything in gamepad histories with a 500 cycle delay (prob would
        // make a memory leak if not?)
        if (gamepad1History.size() > 100) {
            gamepad1History.removeLast();
            gamepad2History.removeLast();
        }
        // keep last gamepad in because its useful for simple button presses
        lastGamepad1.copy(gamepad1);
        lastGamepad2.copy(gamepad2);
    }

    public double RPMtoVelocity(int targetRPM) {
        return (targetRPM * TICKS_PER_REV_S) / 60;
    }
}


