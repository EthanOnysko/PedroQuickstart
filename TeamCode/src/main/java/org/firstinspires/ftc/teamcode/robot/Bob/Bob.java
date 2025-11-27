package org.firstinspires.ftc.teamcode.robot.Bob;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobConstants.*;
import static java.lang.Math.*;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.helpers.PID;
import org.firstinspires.ftc.teamcode.helpers.PIDSpindexer;
import org.firstinspires.ftc.teamcode.robot.Bob.Meccanum.Meccanum;
import org.firstinspires.ftc.teamcode.robot.Bob.Robot;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.BobState;
import org.firstinspires.ftc.teamcode.robot.Bob.helpers.Link;

@Configurable
public class Bob extends Meccanum implements Robot {
    protected HardwareMap hw = null;

    // Controllers
    public ShooterController shooterController = new ShooterController();
    public SpindexerController spindexerController = new SpindexerController();
    public IntakeController intakeController = new IntakeController();
    public TransferController transferController = new TransferController();

    // Motors
    public DcMotorEx intake;
    public DcMotorEx shooterRight;
    public DcMotorEx shooterLeft;

    // Servos
    public CRServo spindexer;
    public DcMotorEx spincoder; // Motor used as encoder for spindexer
    public Servo transfer;
    public Servo light;

    // Pedro Pathing
    public Follower follower;

    // Voltage sensor
    private VoltageSensor vs;

    public Telemetry tele;
    public boolean inited = false;
    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init(HardwareMap hardwareMap) {
        super.init(hardwareMap);

        vs = hardwareMap.voltageSensor.get("Control Hub");

        // Define drive motors (new names)
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("fl");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("bl");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("fr");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("br");

        // Reverse right side motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set braking
        setZeroPowerBehavior(BRAKE);

        // Define shooter motors
        shooterRight = (DcMotorEx) hardwareMap.dcMotor.get("sr");
        shooterLeft = (DcMotorEx) hardwareMap.dcMotor.get("sl");

        // Configure shooters
        shooterLeft.setZeroPowerBehavior(FLOAT);
        shooterLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterRight.setZeroPowerBehavior(FLOAT);
        shooterRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define intake
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(BRAKE);

        // Define spindexer and encoder
        spindexer = hardwareMap.get(CRServo.class, "spindexer");
        spincoder = hardwareMap.get(DcMotorEx.class, "bl"); // Using back left as encoder
        spincoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spincoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define transfer servo
        transfer = hardwareMap.get(Servo.class, "transfer");

        // Define light
        light = hardwareMap.get(Servo.class, "light");

        // Start controllers
        shooterController.start();
        spindexerController.start();
        intakeController.start();
        transferController.start();

        hw = hardwareMap;
        runtime.reset();
        inited = true;
    }

    public void tick() {
        if (follower != null) follower.update();
        tickMacros();
        shooterController.shooterTick();
        spindexerController.spindexerTick();
        intakeController.intakeTick();
        transferController.transferTick();
//        tele.addData("voltage", vs.getVoltage());
//        tele.update();
    }

    // ==================== SHOOTER CONTROLLER ====================

    public class ShooterController {
        private int targetRPM = 0;
        private double velocity = 0;

        public void start() {
            // Initialize shooter
        }

        public void setRPM(int rpm) {
            targetRPM = rpm;
            velocity = RPMtoVelocity(rpm);
        }

        public void setZone1() {
            setRPM(RPM_ZONE1);
        }

        public void setZone2() {
            setRPM(RPM_ZONE2);
        }

        public void stop() {
            setRPM(RPM_OFF);
        }

        public void shooterTick() {
            shooterLeft.setVelocity(velocity);
            shooterRight.setVelocity(velocity);

//            tele.addData("Shooter RPM", targetRPM);
//            tele.addData("Shooter Velocity", velocity);
        }

        private double RPMtoVelocity(int targetRPM) {
            return (targetRPM * TICKS_PER_REV_SHOOTER) / 60.0;
        }

        public int getCurrentRPM() {
            return targetRPM;
        }
    }

    // ==================== SPINDEXER CONTROLLER ====================

    public class SpindexerController {
        private PIDSpindexer spinPID;
        private double targetAngle = 0;
        private boolean emergencyMode = false;

        public void start() {
            spinPID = new PIDSpindexer(TICKS_PER_REV_SPINDEXER, SPINDEX_KP, SPINDEX_KI, SPINDEX_KD);
            spinPID.reset(0);
        }

        public void setTargetAngle(double angle) {
            targetAngle = angle;
            double currentTicks = spincoder.getCurrentPosition();
            spinPID.setTargetAngle(targetAngle, currentTicks);
        }

        public void incrementAngle(double increment) {
            setTargetAngle(targetAngle + increment);
        }

        public void nextPosition() {
            incrementAngle(120);
        }

        public void previousPosition() {
            incrementAngle(-120);
        }

        public void halfStep() {
            incrementAngle(60);
        }

        public void setEmergencyMode(boolean emergency) {
            emergencyMode = emergency;
            if (emergency) {
                spinPID.setConsts(SPINDEX_EMERGENCY_KP, SPINDEX_EMERGENCY_KI, SPINDEX_EMERGENCY_KD);
            } else {
                spinPID.setConsts(SPINDEX_KP, SPINDEX_KI, SPINDEX_KD);
            }
        }

        public void updateLight() {
            if (targetAngle % 120 == 0) {
                light.setPosition(LIGHT_ON);
            } else {
                light.setPosition(LIGHT_OFF);
            }
        }

        public void spindexerTick() {
            double currentTicks = spincoder.getCurrentPosition();
            spinPID.setTargetAngle(targetAngle, currentTicks);

            double power = -spinPID.update(currentTicks);

            // Apply power limits in emergency mode
            if (emergencyMode && (power > 0.1 || power < -0.1)) {
                spindexer.setPower(power * SPINDEX_EMERGENCY_POWER_LIMIT);
            } else {
                spindexer.setPower(power);
            }

            // Calculate current angle for telemetry
            double wrappedTicks = currentTicks % TICKS_PER_REV_SPINDEXER;
            if (wrappedTicks < 0) wrappedTicks += TICKS_PER_REV_SPINDEXER;
            double currentAngle = (wrappedTicks / TICKS_PER_REV_SPINDEXER) * 360.0;
//
//            tele.addData("Spindex Target Angle", targetAngle);
//            tele.addData("Spindex Current Angle", currentAngle);
//            tele.addData("Spindex Power", power);
//            tele.addData("Spindex Emergency Mode", emergencyMode);
        }

        public double getCurrentAngle() {
            double currentTicks = spincoder.getCurrentPosition();
            double wrappedTicks = currentTicks % TICKS_PER_REV_SPINDEXER;
            if (wrappedTicks < 0) wrappedTicks += TICKS_PER_REV_SPINDEXER;
            return (wrappedTicks / TICKS_PER_REV_SPINDEXER) * 360.0;
        }
    }

    // ==================== PID SPINDEXER CLASS ====================



    // ==================== INTAKE CONTROLLER ====================

    public class IntakeController {
        private double intakePower = 0;

        public void start() {
            // Initialize intake
        }

        public void intake() {
            intakePower = INTAKE_POWER_IN;
        }

        public void outtake() {
            intakePower = INTAKE_POWER_OUT;
        }

        public void stopIntake() {
            intakePower = INTAKE_POWER_OFF;
        }

        public void intakeTick() {
            intake.setPower(intakePower);
//            tele.addData("Intake Power", intakePower);
        }
    }

    // ==================== TRANSFER CONTROLLER ====================

    public class TransferController {
        private boolean transferPulse = false;
        private long transferTimer = 0;
        private double transferPosition = TRANSFER_DOWN;

        public void start() {
            transfer.setPosition(TRANSFER_DOWN);
        }

        public void pulse() {
            transferPulse = true;
            transferTimer = System.currentTimeMillis();
            transferPosition = TRANSFER_UP;
        }

        public void setUp() {
            transferPosition = TRANSFER_UP;
            transferPulse = false;
        }

        public void setDown() {
            transferPosition = TRANSFER_DOWN;
            transferPulse = false;
        }

        public void transferTick() {
            if (transferPulse) {
                if (System.currentTimeMillis() - transferTimer > TRANSFER_PULSE_TIME) {
                    transferPosition = TRANSFER_DOWN;
                    transferPulse = false;
                }
            }

            transfer.setPosition(transferPosition);
//            tele.addData("Transfer Position", transferPosition);
//            tele.addData("Transfer Pulsing", transferPulse);
        }

        public boolean isPulsing() {
            return transferPulse;
        }
    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motorBackLeft.setZeroPowerBehavior(zeroPowerBehavior);
        motorBackRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontRight.setZeroPowerBehavior(zeroPowerBehavior);
        motorFrontLeft.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void strafe(double power) {
        motorFrontLeft.setPower(power);
        motorBackLeft.setPower(-power);
        motorFrontRight.setPower(-power);
        motorBackRight.setPower(power);
    }

    // TODO: MACRO SHIT

    public BobState macroState = null;
    public boolean MACROING = false;
    public ElapsedTime macroTimer = new ElapsedTime();
    public int macroTimeout = INFINITY;

    public void runMacro(BobState m) {
        if (macroTimer.milliseconds() < macroTimeout)
            macroTimeout = INFINITY;
        macroState = m;
        MACROING = true;
    }

    public void cancelMacros() {
        MACROING = false;
        macroTimeout = INFINITY;
    }

    public void tickMacros() {
        if (macroTimer.milliseconds() > macroTimeout) {
            macroTimeout = INFINITY;
            MACROING = true;
        }

        if (MACROING) {
            BobState m = macroState;
            if (m.shooterRPM != null) shooterController.setRPM(m.shooterRPM);
//            if (m.spindexerAngle != null) spindexerController.setTargetAngle(m.spindexerAngle);
            if (m.spindexerAngle != null) spindexerController.incrementAngle(m.spindexerAngle);

            if (m.transferPosition != null) {
                if (m.transferPosition == TRANSFER_UP) {
                    transferController.setUp();
                } else {
                    transferController.setDown();
                }
            }
            if (m.intakePower != null) {
                if (m.intakePower == INTAKE_POWER_IN) {
                    intakeController.intake();
                } else if (m.intakePower == INTAKE_POWER_OUT) {
                    intakeController.outtake();
                } else {
                    intakeController.stopIntake();
                }
            }

            if (m.linkedState != null) {
                if (m.linkedState.type == Link.LinkType.WAIT) {
                    macroTimer.reset();
                    macroTimeout = m.linkedState.trigger;
                    macroState = m.linkedState.nextState;
                    macroState = m.linkedState.nextState;
                }
            }
            MACROING = false;
        }
    }

    public void tickWithMacros() {
        if (follower != null) follower.update();
        tickMacros();
        shooterController.shooterTick();
        spindexerController.spindexerTick();
        intakeController.intakeTick();
        transferController.transferTick();
//        tele.addData("voltage", vs.getVoltage());
//        tele.update();
    }
}