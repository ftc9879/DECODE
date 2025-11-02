package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.LimelightTest;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorLimelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


import java.util.List;



@TeleOp
public class TurretTest extends OpMode {
    private Limelight3A limelight;
    CRServo turret;
    DcMotor base1;
    DcMotor base2;

    int current_position;
    int new_position;
    double tx;
    double output;
    boolean turret_mode;
    boolean seen = false;

    PIDController turretPIDController;
    RevColorSensorV3 ballColor1;
    RevColorSensorV3 ballColor2;


    public void init() {
        turret = hardwareMap.crservo.get("turret"); //device name in configuration
        // need to set servo mode to continuous
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        base1 = hardwareMap.get(DcMotor.class,"base1");
        base2 = hardwareMap.get(DcMotor.class,"base2");
        ballColor1 = hardwareMap.get(RevColorSensorV3.class,"ballColor1");
        ballColor2 = hardwareMap.get(RevColorSensorV3.class,"ballColor2");


        // limelight
        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        base1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base1.setDirection(DcMotorSimple.Direction.FORWARD);
        base2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        base2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        base2.setDirection(DcMotorSimple.Direction.FORWARD);

        current_position = 0;
        turret_mode = true;


        turretPIDController = new PIDController(.009,0.0,5.0,0.0);
        turretPIDController.setSetPoint(0);
        ballColor1.enableLed(true);
        ballColor2.enableLed(true);
    }
    public void loop() {
    //    turret.setPower(.5);

        //telemetry.addData("Seen: ", String.valueOf(seen));
        /*
        if (gamepad1.a) {
            turret_mode = true;
        } else if (gamepad1.b) {
            turret_mode = false;

        }
         */

        //color sensor prototype
        telemetry.addData("Green Angled: ", String.valueOf(ballColor1.green()));
        telemetry.addData("Red Angled: ", String.valueOf(ballColor1.red()));
        telemetry.addData("Blue Angled: ", String.valueOf(ballColor1.blue()));

        telemetry.addData("Angled Distance:", String.valueOf(ballColor1.getDistance(DistanceUnit.CM)));

        if (ballColor1.green() > 100 && ballColor1.blue() > 20) {
            telemetry.addData("Angled Color Sensor Seen:","Green");
        } else if (ballColor1.green() < 100 && ballColor1.blue() > 30) {
            telemetry.addData("Angled Color Sensor Seen:","Purple");
        }

        if (ballColor1.getDistance(DistanceUnit.CM) < 5) {
            telemetry.addData("Angled Distance Sensor: ", "Artifact Seen");
        }


        telemetry.addData("Green Straight: ", String.valueOf(ballColor2.green()));
        telemetry.addData("Red Straight: ", String.valueOf(ballColor2.red()));
        telemetry.addData("Blue Straight: ", String.valueOf(ballColor2.blue()));

        telemetry.addData("Straight Distance:", String.valueOf(ballColor2.getDistance(DistanceUnit.CM)));

        if (ballColor2.green() > 2000 && ballColor2.blue() < 2500) {
            telemetry.addData("Straight Color Sensor Seen:","Green");
        } else if (ballColor2.green() < 2000 && ballColor2.blue() > 2500) {
            telemetry.addData("Straight Color Sensor Seen:","Purple");
        }

        if (ballColor2.getDistance(DistanceUnit.CM) < 5) {
            telemetry.addData("Straight Distance Sensor: ", "Artifact Seen");
        }


        // base zone prototype
        if (gamepad1.x) {
            base1.setPower(.6);
            base2.setPower(.6);
        } else {
            base1.setPower(0);
            base2.setPower(0);

        }



        // turret prototype
        if (turret_mode) {
            limelight_process();
            if(seen) {
                while (tx > -10 && tx < 10) {
                    telemetry.addData("Seen: ", "Is working");
                    output = turretPIDController.calculate(tx);
                    turret.setPower(output); //needs to be set power from a continuous servo
                    limelight_process();
                    if (!seen) {
                        break;
                    }
                }


            } else {
                turret.setPower(0); // needs to be set power

            }
        }
        /*
        OR
        if (turret_mode) {
            if(!seen) {
                turret.setPower(.2);
            } else {

            }
        }
        */




        telemetry.update();
    }

    public void limelight_process(){
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(),(int)status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            // Access general information
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            telemetry.addData("tx", result.getTx());
            telemetry.addData("txnc", result.getTxNC());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("tync", result.getTyNC());

            tx = result.getTx();

            telemetry.addData("Botpose", botpose.toString());



            // Access fiducial results
            List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fr : fiducialResults) {
                telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
            }
            seen = true;
        } else {
            telemetry.addData("Limelight", "No data available");
            seen = false;
        }

        telemetry.update();
    }


}
