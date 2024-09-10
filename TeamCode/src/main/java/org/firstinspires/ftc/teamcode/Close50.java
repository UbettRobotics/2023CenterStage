package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.claw;
import static org.firstinspires.ftc.teamcode.Robot.clawflipper;
import static org.firstinspires.ftc.teamcode.Robot.clawgrab;
import static org.firstinspires.ftc.teamcode.Robot.clawretreat;
import static org.firstinspires.ftc.teamcode.Robot.frontldist;
import static org.firstinspires.ftc.teamcode.Robot.frontrdist;
import static org.firstinspires.ftc.teamcode.Robot.initMotors;
import static org.firstinspires.ftc.teamcode.Robot.led;
import static org.firstinspires.ftc.teamcode.Robot.leftback;
import static org.firstinspires.ftc.teamcode.Robot.leftfront;
import static org.firstinspires.ftc.teamcode.Robot.pixelplaced;
import static org.firstinspires.ftc.teamcode.Robot.pixelplacer;
import static org.firstinspires.ftc.teamcode.Robot.placepixel;
import static org.firstinspires.ftc.teamcode.Robot.rightback;
import static org.firstinspires.ftc.teamcode.Robot.rightfront;
import static org.firstinspires.ftc.teamcode.Robot.slidemotor;
import static org.firstinspires.ftc.teamcode.Robot.led;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.CubeVisionPipeline.Side;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;


import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous(name = "Close50 (check ALLIANCE!)", preselectTeleOp = "Teleop1")

public class Close50 extends LinearOpMode {
    int new90 = 87;
    int isRed = 0;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    boolean forward = false;



    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        final double DESIRED_DISTANCE = 4.0; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN = 0.02;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN = 0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN = 0.01;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)


        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double Adrive = 0.;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);


        initMotors(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        pixelplacer.setPosition(placepixel);


        /// CAMERA STUFF
        int parking = 1; // read by camera
        int isPark = 0;

        CubeVisionPipeline pipeline = new CubeVisionPipeline(telemetry, 0, 0);
        webcam.setPipeline(pipeline);

        AprilTagProcessor TagProcess;
        TagProcess = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .build();

        VisionPortal.Builder VisionBuilder;
        VisionPortal AprilPortal;

        // Create a new VisionPortal Builder object.
        VisionBuilder = new VisionPortal.Builder();
        AprilTagDetection desiredTag = null;

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }


            @Override
            public void onError(int errorCode) {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }
            /*
             * This will be called if the camera could not be opened
             */

        });


////////Program start////////////////////////////////////////////////////////////////////////
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                pipeline.setColor(0);//BLUE
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                pipeline.setColor(1);//RED
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
            }
        }
        waitForStart();
        isRed = pipeline.getColor();

        telemetry.update();
        Side side = pipeline.getSide();


        telemetry.update();
        ////Move on start/init
        claw.setPosition(clawgrab);
        clawflipper.setPosition(clawretreat);
        sleep(200);
        slidemotor.setTargetPosition(5);
        pixelplacer.setPosition(placepixel);
        slidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidemotor.setPower(.25);

        Pose2d startPose = null;
        if (isRed == 1) {
            startPose = new Pose2d(12, -66, Math.toRadians(90));

        } else {
            startPose = new Pose2d(12, 66, Math.toRadians(270));
        }
        drive.setPoseEstimate(startPose);
        clawflipper.setPosition(clawretreat);


        if (isRed == 1) { //<REDRED>

            Trajectory trajl = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(8, -10), Math.toRadians(170))
                    .build();
            Trajectory trajl2 = drive.trajectoryBuilder(trajl.end())
                    .lineTo(new Vector2d(25, -24))
                    .build();
            Trajectory trajl3 = drive.trajectoryBuilder(trajl2.end())
                    .lineToLinearHeading(new Pose2d(43, -32, Math.toRadians(10)))
                    .build();


            Trajectory trajr = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(30, -36), Math.toRadians(85))
                    .build();
            Trajectory trajr2 = drive.trajectoryBuilder(trajr.end())
                    .lineTo(new Vector2d(31, -60))
                    .build();
            Trajectory trajr3 = drive.trajectoryBuilder(trajr2.end())
                    .lineToLinearHeading(new Pose2d(52, -36, Math.toRadians(10)))
                    .build();

            Trajectory trajs = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(12, -28))
                    .build();
            Trajectory trajs2 = drive.trajectoryBuilder(trajs.end())
                    .lineTo(new Vector2d(12, -50))
                    .build();
            Trajectory trajs3 = drive.trajectoryBuilder(trajs2.end())
                    .splineToLinearHeading(new Pose2d(50, -36, Math.toRadians(12)), Math.toRadians(90))
                    .build();

            switch (side) {
                case LEFT_SIDE:
                    drive.followTrajectory(trajl);
                    drive.followTrajectory(trajl2);
                    drive.followTrajectory(trajl3);


                    break;
                case RIGHT_SIDE:
                    drive.followTrajectory(trajr);
                    drive.followTrajectory(trajr2);
                    drive.followTrajectory(trajr3);


                    break;
                case MIDDLE_SIDE:
                    drive.followTrajectory(trajs);
                    drive.followTrajectory(trajs2);
                    drive.followTrajectory(trajs3);


            }
            lineup(0.3);
            sleep(1000);


        }

        if (isRed == 0) { //<REDRED>

            Trajectory trajbr = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(3, 36), Math.toRadians(190))
                    .build();
            Trajectory trajbr2 = drive.trajectoryBuilder(trajbr.end())
                    .lineTo(new Vector2d(-5,38.5))
                    .build();
            Trajectory trajbr3 = drive.trajectoryBuilder(trajbr2.end())
                    .lineToLinearHeading(new Pose2d(25, 31.5, Math.toRadians(190)))
                    .build();
            Trajectory trajbr4 = drive.trajectoryBuilder(trajbr3.end())
                    .lineToLinearHeading(new Pose2d(40, 29.5, Math.toRadians(-10)))
                    .build();
            Trajectory trajbr5 = drive.trajectoryBuilder(trajbr4.end())
                    .lineToLinearHeading(new Pose2d(42, 14, Math.toRadians(-10)))
                    .build();


            Trajectory trajbl = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(31, 36), Math.toRadians(90))
                    .build();
            Trajectory trajbl2 = drive.trajectoryBuilder(trajbl.end())
                    .lineTo(new Vector2d(35, 63))
                    .build();
            Trajectory trajbl3 = drive.trajectoryBuilder(trajbl2.end())
                    .lineToLinearHeading(new Pose2d(48, 43, Math.toRadians(0)))
                    .build();

            Trajectory trajbs = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(12, 22))
                    .build();
            Trajectory trajbs2 = drive.trajectoryBuilder(trajbs.end())
                    .lineTo(new Vector2d(12, 50))
                    .build();
            Trajectory trajbs3 = drive.trajectoryBuilder(trajbs2.end())
                    .splineToLinearHeading(new Pose2d(42, 41, Math.toRadians(-10)), Math.toRadians(270))
                    .build();

            switch (side) {
                case LEFT_SIDE:
                    drive.followTrajectory(trajbl);
                    drive.followTrajectory(trajbl2);
                    drive.followTrajectory(trajbl3);

                    break;
                case RIGHT_SIDE:
                    drive.followTrajectory(trajbr);
                    drive.followTrajectory(trajbr2);
                    drive.followTrajectory(trajbr3);
                    drive.followTrajectory(trajbr4);
                    //drive.followTrajectory(trajbr5); NOT needed and is risky

                    break;
                case MIDDLE_SIDE:
                    drive.followTrajectory(trajbs);
                    drive.followTrajectory(trajbs2);
                    drive.followTrajectory(trajbs3);


            }


            lineup(0.3);
            sleep(1000);

        } //</BLUEBLUE>


        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        closeCV(webcam);



        //{{
        VisionBuilder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));      // Other choices are: RC phone camera and "switchable camera name".
        // Add the AprilTag Processor to the VisionPortal Builder.
        VisionBuilder.addProcessor(TagProcess);       // An added Processor is enabled by default.

        VisionBuilder.setCameraResolution(new Size(320, 240));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        VisionBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        VisionBuilder.enableLiveView(true);    // Enable LiveView (RC preview).
        VisionBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.
        AprilPortal = VisionBuilder.build();
        //AprilPortal.setProcessorEnabled(TagProcess, false);


        //Exposure
        telemetry.addData("Camera", "Waiting");
        telemetry.update();

        while (AprilPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !isStopRequested()) {}

        if(!isStopRequested()) {
            ExposureControl exposureControl = AprilPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(6, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = AprilPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        }
        ///

        targetFound = false;
        desiredTag  = null;
        int DESIRED_TAG_ID;
        boolean loopBreak = false;
        boolean loopBreak2 = false;
        if(isRed == 1) {
            switch (side) {
                case LEFT_SIDE:
                    DESIRED_TAG_ID = 4;

                    break;
                case RIGHT_SIDE:
                    DESIRED_TAG_ID = 6;

                    break;
                case MIDDLE_SIDE:
                    DESIRED_TAG_ID = 5;
                    break;
                default:
                    DESIRED_TAG_ID = 5;
            }
        }else {
            switch(side) {
                case LEFT_SIDE:
                    DESIRED_TAG_ID = 1;

                    break;
                case RIGHT_SIDE:
                    DESIRED_TAG_ID = 3;

                    break;
                case MIDDLE_SIDE:
                    DESIRED_TAG_ID = 2;
                    break;
                default:
                    DESIRED_TAG_ID = 2;
            }
        }
        telemetry.addData("Camera", "Entering Control Loop");
        telemetry.update();

        while(!loopBreak && !isStopRequested()) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = TagProcess.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    strafe(0);
                    loopBreak = true;  // don't look any further.
                } else {
                    targetFound = false;
                }
            }
            if (currentDetections.size() != 0 && targetFound == false) {
                AprilTagDetection detection = currentDetections.get(0); //Get first tag, base actions off that
                if (isRed == 0) {
                    if (detection.id == 1) {
                        strafe(0.32);
                    } else if (detection.id == 3) {
                        strafe(-0.32);
                    } else if (detection.id == 2) {
                        if (DESIRED_TAG_ID == 1) {
                            strafe(-0.32);
                        } else {
                            strafe(0.32);
                        }

                    }
                } else {
                    if (detection.id == 4) {
                        strafe(0.32);
                    } else if (detection.id == 6) {
                        strafe(-0.32);
                    } else if (detection.id == 5) {
                        if (DESIRED_TAG_ID == 4) {
                            strafe(-0.32);
                        } else {
                            strafe(0.32);
                        }

                    }
                }
            }


            if (targetFound) {
                telemetry.addData("Target", desiredTag.id + " " + desiredTag.metadata.name);
                telemetry.addData("Range", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData(">", "No Target");
                telemetry.addData(">", DESIRED_TAG_ID);

                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER);

            }
            telemetry.update();
        }


            //MOVE
            if (desiredTag != null && targetFound == true) {
                while (!loopBreak2) {

                    //update AprilTags again
                    List<AprilTagDetection> currentDetections = TagProcess.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) &&
                                ((detection.id == DESIRED_TAG_ID))) {
                            desiredTag = detection;
                        }
                    }

                    double bearing = desiredTag.ftcPose.bearing;
                    double yaw = desiredTag.ftcPose.yaw;
                    double range = desiredTag.ftcPose.range;
                    //telemetry update
                    telemetry.addData("Target", desiredTag.id + " " + desiredTag.metadata.name);
                    telemetry.addData("Range", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw", desiredTag.ftcPose.yaw);
                    telemetry.update();

                    if (bearing > 3) {
                        moveRobot(0, 0.4, 0);
                    } else if (bearing < -3) {
                        moveRobot(0, -0.4, 0);
                    } else {
                        //only reached when everything good
                        loopBreak2 = true;
                        moveRobot(0, 0, 0);
                    }
                }

            }

        telemetry.addData("DONE", "DONE");
        telemetry.update();


        Trajectory final1 = drive.trajectoryBuilder(startPose)
                .strafeRight(15.5)
                .build();
        Trajectory final2r = drive.trajectoryBuilder(startPose)
                .strafeRight(30)
                .build();
        Trajectory final2b = drive.trajectoryBuilder(startPose)
                .strafeLeft(30)
                .build();
        Trajectory final3 = drive.trajectoryBuilder(startPose)
                .back(10)
                .build();



        //sleep(500);


        lineup(0.3);
        goforward(0.3);
        sleep(500);

            drive.followTrajectory(final1);
            setpower(0.3, true);
            sleep(250);
            setpower(0, true);
            sleep(200);
            pixelplacer.setPosition(0.65);
            sleep(150);
            pixelplacer.setPosition(pixelplaced);
            sleep(750);
            drive.followTrajectory(final3);
            if(isRed == 1){
                drive.followTrajectory(final2r);
                drive.followTrajectory(final1);
            }else{
                drive.followTrajectory(final2b);
                drive.followTrajectory(final1);
            }

    }
    private void closeCV(OpenCvCamera wc) {
        wc.stopStreaming();
        wc.closeCameraDevice();

    }
    private void openVisionPortal() {

    }
    public void moveRobot(double x, double y, double yaw) {
        initMotors(this);
        leftfront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftback.setDirection(DcMotorSimple.Direction.REVERSE);
        rightfront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightback.setDirection(DcMotorSimple.Direction.FORWARD);
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        //Scale down speed
        leftFrontPower /= 2;
        rightFrontPower /= 2;
        leftBackPower /= 2;
        rightBackPower /= 2;

        // Send powers to the wheels.
        leftfront.setPower(leftFrontPower);
        rightfront.setPower(rightFrontPower);
        leftback.setPower(leftBackPower);
        rightback.setPower(rightBackPower);
    }
    public void lineup(double y){
        while(true){
            if(frontldist.getDistance(DistanceUnit.INCH) - frontrdist.getDistance(DistanceUnit.INCH) > 0.35){
                setpower(y,false);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            } else if (frontldist.getDistance(DistanceUnit.INCH) - frontrdist.getDistance(DistanceUnit.INCH) < -0.35) {
                setpower(-y,false);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            }else{
                setpower(0,false);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            }
        }
    }
    public void goforward(double y){
        while (true){
            if(frontldist.getDistance(DistanceUnit.INCH)/2 + frontrdist.getDistance(DistanceUnit.INCH)/2 > 6){
                setpower(y, true);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }else{
                setpower(0, true);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                break;
            }
        }
    }

    public void setpower(double x, boolean forward) {
        if (forward = false) {
            leftfront.setPower(x);
            leftback.setPower(x);
            rightfront.setPower(-x);
            rightback.setPower(-x);
        } else {
            leftfront.setPower(x);
            leftback.setPower(x);
            rightfront.setPower(x);
            rightback.setPower(x);
        }
    }
    public void strafe(double x){
        leftfront.setPower(x);
        leftback.setPower(-x);
        rightfront.setPower(-x);
        rightback.setPower(x);

    }
    }
