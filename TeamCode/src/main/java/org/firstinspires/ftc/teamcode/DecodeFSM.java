package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import java.util.Timer;

enum GameState {
    GAME_START,
    GAME_INTAKE,
    GAME_SCORE_CLOSE,
    GAME_SCORE_FAR,
    GAME_BASE,
    GAME_RESET,
    GAME_SUPPORTING,
}

public class DecodeFSM {
    GameState gamestate;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    LimitSwitchIntake limitSwitchIntake = new LimitSwitchIntake();
    boolean targetSet = false;
    boolean endingTask = false;
    boolean resetEncoder = false;
    boolean holding = false;
    boolean driveOn = true;
    boolean intakeReversed = false;
    boolean intakeComplete = false;
    boolean intakeStopped = false;
    boolean launcherPhase = false;

    double leftFrontpower;
    double rightFrontpower;
    double leftBackpower;
    double rightBackpower;
    double leftsticky;
    double leftstickx;
    double rightstickx;
    double r;
    double robotangle;
    double rightX;
    double divide;

    boolean previousRightBumper = false;
    boolean previousLeftBumper = false;
    boolean previousA = false;
    boolean previousY = false;
    boolean previousX = false;
    boolean previousB = false;

    ElapsedTime scoringTimer;
    ElapsedTime intakeTimer;
    ElapsedTime resetTimer;
    ElapsedTime reverseTimer;
    ElapsedTime supportingTimer;

    Servo transferServo;  //Port C4
    Servo hoodServo1; //Port C3    // Use Tetrix Servo Y Connector to make the two servos linked
    Servo hoodServo2; //Port C2
    Servo turretServo1; //Port C1
    Servo turretServo2; //Port C0

    DcMotor intake; //Port E1
    DcMotor launcher; //Port E2
    DcMotor spindexer; //Port E3
    DcMotor supporting;
    DcMotor rightBack; //Port C0
    DcMotor rightFront;  //Port C1
    DcMotor leftBack; //Port C2
    DcMotor leftFront;  //Port C3

    RevColorSensorV3 color1;
    RevColorSensorV3 color2;

    final int TS_NO_CONTACT = 2; // get no contact value
    final int TS_CONTACT = 2; //get contact value;

    final int POWER_CLOSE = 5; // get power for launching up to goal
    final int LOWEST_HOOD = 5; // get servo position for hood at lowest position
    final int SPINDEXER_SPEED = 5; //get ideal spindexer speed
    final int SUPPORTED_RAISED = 5; // get encoder counts to raise robot
    final int SUPPORTED_LOWERED = 5; // get encoder counts to lower robot, could be 0

    public void init(){
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        gamestate = GameState.GAME_START;
        limitSwitchIntake.init(hardwareMap);

        scoringTimer.reset();
        intakeTimer.reset();
        resetTimer.reset();
        reverseTimer.reset();
        supportingTimer.reset();

        color1.initialize();
        color2.initialize();

        transferServo = hardwareMap.servo.get("TS");
        transferServo.setPosition(TS_NO_CONTACT);
        hoodServo1 = hardwareMap.servo.get("HS");
        hoodServo1.setPosition(LOWEST_HOOD);

        rightBack = hardwareMap.dcMotor.get("RB");
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setPower(0);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = hardwareMap.dcMotor.get("RF");
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setPower(0);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack = hardwareMap.dcMotor.get("LB");
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setPower(0);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront = hardwareMap.dcMotor.get("LF");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setPower(0);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.dcMotor.get("I");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setPower(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher = hardwareMap.dcMotor.get("L");
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setPower(0);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        spindexer = hardwareMap.dcMotor.get("SP");
        spindexer.setDirection(DcMotor.Direction.FORWARD);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setPower(0);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        supporting = hardwareMap.dcMotor.get("SU");
        supporting.setDirection(DcMotor.Direction.FORWARD);
        supporting.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        supporting.setTargetPosition(0);
        supporting.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        supporting.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop() {

        switch (gamestate) {
            case GAME_START:
                // confirm previous state has finished
                if (scoringTimer.time() > .05 && intakeTimer.time() > .05 && endingTask) {
                    endingTask = false;
                }
                // entering reset state
                if (resetEncoder) {
                    gamestate = GameState.GAME_RESET;
                    resetTimer.reset();
                    resetEncoder = false;
                    holding = true;
                    break;
                }
                // entering intake state
                if (gamepad1.x) {
                    gamestate = GameState.GAME_INTAKE;
                    intakeTimer.reset();
                    previousX = true;
                    intakeComplete = false;
                }
                // entering launch close state
                if (gamepad1.right_bumper) {
                    gamestate = GameState.GAME_SCORE_CLOSE;
                    scoringTimer.reset();
                    previousRightBumper = true;
                    launcherPhase = true;
                }
                //entering launch far state
                if (gamepad1.left_bumper) {
                    gamestate = GameState.GAME_SCORE_FAR;
                    scoringTimer.reset();
                    previousLeftBumper = true;
                    launcherPhase = true;
                }
                //entering supporting state
                if (gamepad1.y) {
                    gamestate = GameState.GAME_SUPPORTING;
                    previousY = true;
                }
                break;

            case GAME_INTAKE:
                //move the transfer mechanism out of the way
                if (transferServo.getPosition() != TS_NO_CONTACT) {
                    transferServo.setPosition(TS_NO_CONTACT);
                }
                intake.setPower(1);
                spindexer.setPower(SPINDEXER_SPEED);

                // when all balls are in the intake, press a
                if (gamepad1.a) {
                    intakeComplete = true;
                    previousA = true;
                }
                // eventually stop the spindexer at limit switch position
                if (limitSwitchIntake.getLimitSwitchIntake() && intakeComplete) {
                    spindexer.setPower(0);
                    intakeStopped = true;
                }
                //reverse when intaking if needed
                if (gamepad1.b && intakeTimer.time() > .25 && !intakeComplete) {
                    intake.setPower(-1);
                    intakeReversed = true;
                    reverseTimer.reset();
                    previousB = true;
                }
                //exit intake stage after intaking all three balls
                if (previousX && limitSwitchIntake.getLimitSwitchIntake()) {
                    gamestate = GameState.GAME_START;
                    previousX = false;
                    previousB = false;
                    endingTask = true;
                    intakeComplete = false;
                    intakeStopped = false;
                    intakeTimer.reset();
                }
                break;

            case GAME_SCORE_CLOSE:
                if (scoringTimer.time() > .25) {
                    hoodServo1.setPosition(LOWEST_HOOD);
                    launcher.setPower(POWER_CLOSE);
                    targetSet = true;
                }
                if (gamepad1.a && launcherPhase) {
                    transferServo.setPosition(TS_CONTACT);
                    previousA = true;
                }
                //leave the score close state and move the transfer out of the way
                if (gamepad1.x) {
                    transferServo.setPosition(TS_NO_CONTACT);
                    gamestate = GameState.GAME_START;
                    previousA = false;
                    endingTask = true;
                    previousRightBumper = false;
                    launcherPhase = false;
                    scoringTimer.reset();
                    intake.setPower(0);
                    spindexer.setPower(0);
                }
                break;

            case GAME_SUPPORTING:
                if (gamepad1.dpad_up) {
                    supporting.setTargetPosition(SUPPORTED_RAISED);
                }
                if (gamepad1.dpad_down) {
                    supporting.setTargetPosition(SUPPORTED_LOWERED);
                }
                if (gamepad1.y && previousY) {
                    gamestate = GameState.GAME_START;
                    previousY = false;
                    endingTask = true;
                }
                break;
        }
        telemetry();
        drive(); // use this until roadrunner


    }
    public void telemetry(){
        telemetry.addData("Color Sensor 1R:",color1.red());
        telemetry.addData("Color Sensor 1G:",color1.green());
        telemetry.addData("Color Sensor 1B:",color1.blue());

        telemetry.addData("Color Sensor 2R", color2.red());
        telemetry.addData("Color Sensor 2G", color2.green());
        telemetry.addData("Color Sensor 2B", color2.blue());
        telemetry.update();
    }
    // This function has the driver controlled mecanum drive code
    public void drive(){
        leftsticky = -gamepad1.left_stick_y;
        leftstickx = gamepad1.left_stick_x;
        rightstickx = gamepad1.right_stick_x*.5;
        r = Math.hypot(leftstickx, leftsticky);
        robotangle = Math.atan2(leftsticky, leftstickx) - Math.PI / 4;
        rightX = rightstickx;
        leftFrontpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) + rightX;
        leftBackpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) + rightX;
        rightFrontpower = (r * Math.sin(robotangle)) * (2 / Math.sqrt(2)) - rightX;
        rightBackpower = (r * Math.cos(robotangle)) * (2 / Math.sqrt(2)) - rightX;
        if (leftFrontpower > 1 || leftFrontpower < -1) {
            divide = Math.abs(leftFrontpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (leftBackpower > 1 || leftBackpower < -1) {
            divide = Math.abs(leftBackpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (rightFrontpower > 1 || rightFrontpower < -1) {
            divide = Math.abs(rightFrontpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        } else if (rightBackpower > 1 || rightBackpower < -1) {
            divide = Math.abs(rightBackpower);
            leftFrontpower = leftFrontpower / divide;
            leftBackpower = leftBackpower / divide;
            rightFrontpower = rightFrontpower / divide;
            rightBackpower = rightBackpower / divide;
        }
        leftFront.setPower(leftFrontpower);
        rightFront.setPower(rightFrontpower);
        leftBack.setPower(leftBackpower);
        rightBack.setPower(rightBackpower);

    }
}
