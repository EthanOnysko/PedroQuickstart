/*package org.firstinspires.ftc.teamcode.opmodes.tele;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Deque;
import java.util.LinkedList;
@TeleOp
public class Scrimmage25Tele extends OpMode {
    public static int RPM;
    double velocity;
    Telemetry telemetry;

    public static double TICKS_PER_REV = 28;


    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor intake;
    DcMotorEx shooterRight;
    DcMotorEx shooterLeft;
    Servo transfer;
    CRServo spindexer;
    Limelight3A limelight;
    GoBildaPinpointDriver pinpoint;

//    Follower follower;

    private double transferUnder = 0.1;
    private double transferUp = 0.3;

    boolean toggleState = false;
    boolean wasButtonPressed = false;
    boolean startShooting = false;

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");

        transfer = (Servo) hardwareMap.servo.get("transfer");
        spindexer = hardwareMap.get(CRServo.class, "spindexer");

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

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();
        pinpoint.recalibrateIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

//        follower = Constants.createFollower(hardwareMap);
//        follower.setStartingPose(startPose);
//        follower.update();
//
//        // 2) Keep Pinpoint aligned to the same field pose
//        pinpoint.setPosition(new Pose2D(
//                DistanceUnit.INCH,
//                startPose.getX(),
//                startPose.getY(),
//                AngleUnit.DEGREES,
//                Math.toDegrees(startPose.getHeading())));
    }

    @Override
    public void start() {
        transfer.setPosition(transferUnder);
    }

    @Override
    // loops after start press
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

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

        if (gamepad2.y){
            startShooting = true;
        }
        else if (gamepad2.x){
            startShooting = false;
        }

        if (gamepad2.a) {
            intake.setPower(-.5);
        }
        else if (gamepad2.b) {
            intake.setPower(.5);
        }
        else {
            intake.setPower(0);
        }
        if (gamepad2.dpad_left) {
            spindexer.setPower(-.5);
        }
        else if (gamepad2.dpad_right) {
            spindexer.setPower(.5);
        }
        else {
            spindexer.setPower(0);
        }

        // the shit

        if (startShooting) {
            velocity = RPMtoVelocity(RPM);
            shooterLeft.setVelocity(velocity);
            shooterRight.setVelocity(velocity);

            double currentRPM1 = (shooterLeft.getVelocity()/TICKS_PER_REV)*60.0;
            double currentRPM2 = (shooterRight.getVelocity()/TICKS_PER_REV)*60.0;
            double targetRPM = RPM;

            telemetry.addData("Launcher Target RPM", targetRPM);

            telemetry.addData("Launcher 1 Current RPM", currentRPM1);
            telemetry.addData("Launcher 2 Current RPM", currentRPM2);
        }

        telemetry.update();



        // bombaclat toggle transfer

        if (gamepad1.a) {
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
        }


    }
    public double RPMtoVelocity (int targetRPM) {
        return (targetRPM * TICKS_PER_REV)/60;
    }
}*/