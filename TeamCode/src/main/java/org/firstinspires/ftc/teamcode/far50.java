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
import static org.firstinspires.ftc.teamcode.Robot.pixelback;
import static org.firstinspires.ftc.teamcode.Robot.pixelplaced;
import static org.firstinspires.ftc.teamcode.Robot.pixelplacer;
import static org.firstinspires.ftc.teamcode.Robot.placepixel;
import static org.firstinspires.ftc.teamcode.Robot.resetMotors;
import static org.firstinspires.ftc.teamcode.Robot.rightback;
import static org.firstinspires.ftc.teamcode.Robot.rightfront;
import static org.firstinspires.ftc.teamcode.Robot.slidemotor;

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


@Autonomous(name = "Far50", preselectTeleOp = "Teleop1")

public class far50 extends LinearOpMode {
    int new90 = 87;
    int isRed = 0;

    int isPark = 1;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag




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
        slidemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        /// CAMERA STUFF
        int parking = 1; // read by camera

        CubeVisionPipeline pipeline = new CubeVisionPipeline(telemetry, 1, 1);
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
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                pipeline.setColor(1);//RED
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);

            }
            if(gamepad1.a || gamepad2.a){
                pipeline.Park(0);
            }
            if(gamepad1.b || gamepad2.b){
                pipeline.Park(1);
            }

        }
        slidemotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        isRed = pipeline.getColor();
        if (isRed == 1) {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
        } else {
            led.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_BLUE);
        }

        telemetry.update();
        Side side = pipeline.getSide();


        telemetry.update();
        ////Move on start/init
        claw.setPosition(clawgrab);
        pixelplacer.setPosition(pixelback);
        clawflipper.setPosition(clawretreat);
        sleep(200);
        slidemotor.setTargetPosition(5);
        slidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidemotor.setPower(.25);


        Pose2d startPose = null;
        if (isRed == 1) {
            startPose = new Pose2d(-36, -66, Math.toRadians(90));
        } else {
            startPose = new Pose2d(-36, 66, Math.toRadians(270));
        }
        drive.setPoseEstimate(startPose);
        clawflipper.setPosition(clawretreat);


        if (isRed == 1) { //<REDRED>

            Trajectory trajl = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(-56.5, -27), Math.toRadians(90))
                    .build();
            Trajectory trajl2 = drive.trajectoryBuilder(trajl.end())
                    .lineTo(new Vector2d(-56.5, -45))
                    .build();
            Trajectory trajl3 = drive.trajectoryBuilder(trajl2.end())
                    .lineTo(new Vector2d(-32.5, -45))
                    .build();

            Trajectory trajl4 = drive.trajectoryBuilder(trajl3.end())
                    .lineToLinearHeading(new Pose2d(-42, 4, Math.toRadians(0)))
                    .build();

            Trajectory trajl5 = drive.trajectoryBuilder(trajl4.end())
                    .lineTo(new Vector2d(42, 4))
                    .build();
            Trajectory trajl6 = drive.trajectoryBuilder(trajl5.end())
                    .splineToConstantHeading(new Vector2d(50, -35), Math.toRadians(-10))
                    .build();


            Trajectory trajr = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-22, -36), Math.toRadians(0))
                    .build();
            Trajectory trajr2 = drive.trajectoryBuilder(trajr.end())
                    .lineTo(new Vector2d(-36, -36))
                    .build();
            Trajectory trajr3 = drive.trajectoryBuilder(trajr2.end())
                    .lineToLinearHeading(new Pose2d(-40, -33, Math.toRadians(5)))
                    .build();
            Trajectory trajr4 = drive.trajectoryBuilder(trajr3.end())
                    .lineTo(new Vector2d(-40, 4))
                    .build();
            Trajectory trajr5 = drive.trajectoryBuilder(trajr4.end())
                    .lineTo(new Vector2d(40, 4))
                    .build();
            Trajectory trajr6 = drive.trajectoryBuilder(trajr5.end())
                    .splineToConstantHeading(new Vector2d(50, -35), Math.toRadians(-10))
                    .build();


            Trajectory trajs = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(-36, -22.5))
                    .build();
            Trajectory trajs2 = drive.trajectoryBuilder(trajs.end())
                    .lineTo(new Vector2d(-40, -38))
                    .build();
            Trajectory trajs3 = drive.trajectoryBuilder(trajs2.end())
                    .lineToLinearHeading(new Pose2d(-55, -40, Math.toRadians(-10)))
                    .build();
            Trajectory trajs4 = drive.trajectoryBuilder(trajs3.end())
                    .lineToLinearHeading(new Pose2d(-58, 3, Math.toRadians(12)))
                    .build();
            Trajectory trajs5 = drive.trajectoryBuilder(trajs4.end())
                    .lineTo(new Vector2d(45, 3))
                    .build();
            Trajectory trajs6 = drive.trajectoryBuilder(trajs5.end())
                    .splineToConstantHeading(new Vector2d(50, -31), Math.toRadians(-10))
                    .build();


            Trajectory trajP = drive.trajectoryBuilder(new Pose2d())
                    .forward(18)
                    .build();


            switch (side) {
                case LEFT_SIDE:
                    drive.followTrajectory(trajl);
                    drive.followTrajectory(trajl2);
                    drive.followTrajectory(trajl3);
                    drive.followTrajectory(trajl4);
                    drive.turn(3);
                    drive.followTrajectory(trajl5);
                    if (isPark == 1) {
                        drive.followTrajectory(trajP);
                    } else {
                        drive.followTrajectory(trajl6);
                    }


                    break;
                case RIGHT_SIDE:
                    pixelplacer.setPosition(0.65);
                    drive.followTrajectory(trajr);
                    drive.followTrajectory(trajr2);
                    drive.followTrajectory(trajr3);
                    drive.followTrajectory(trajr4);
                    pixelplacer.setPosition(placepixel);
                    drive.turn(Math.toRadians(-14));
                    drive.followTrajectory(trajr5);
                    if (isPark == 1) {
                        drive.followTrajectory(trajP);
                    } else {
                        drive.followTrajectory(trajr6);
                    }


                    break;
                case MIDDLE_SIDE:
                    drive.followTrajectory(trajs);
                    drive.followTrajectory(trajs2);
                    drive.followTrajectory(trajs3);
                    drive.followTrajectory(trajs4);
                    drive.turn(Math.toRadians(-21));

                    drive.followTrajectory(trajs5);
                    if (isPark == 1) {
                        drive.followTrajectory(trajP);
                    } else {
                        drive.followTrajectory(trajs6);
                    }


            }
            lineup(0.3);

        }

        if (isRed == 0) { //<REDRED>
            Trajectory trajbr = drive.trajectoryBuilder(startPose)
                    .splineToConstantHeading(new Vector2d(-55.75, 32), Math.toRadians(90))
                    .build();
            Trajectory trajbr2 = drive.trajectoryBuilder(trajbr.end())
                    .lineTo(new Vector2d(-55.5, 50))
                    .build();
            Trajectory trajbr3 = drive.trajectoryBuilder(trajbr2.end())
                    .lineToLinearHeading(new Pose2d(-40, 50, Math.toRadians(12)))
                    .build();

            Trajectory trajbr4 = drive.trajectoryBuilder(trajbr3.end())
                    .lineTo(new Vector2d(-46, -12))
                    .build();

            Trajectory trajbr5 = drive.trajectoryBuilder(trajbr4.end())
                    .lineTo(new Vector2d(42, -13))
                    .build();
            Trajectory trajbr6 = drive.trajectoryBuilder(trajbr5.end())
                    .splineToConstantHeading(new Vector2d(40, 35), Math.toRadians(10))
                    .build();


            Trajectory trajbl = drive.trajectoryBuilder(startPose)
                    .splineTo(new Vector2d(-20.5, 37), Math.toRadians(-5))
                    .build();
            Trajectory trajbl2 = drive.trajectoryBuilder(trajbl.end())
                    .lineTo(new Vector2d(-40, 36))
                    .build();
            Trajectory trajbl3 = drive.trajectoryBuilder(trajbl2.end())
                    .lineToLinearHeading(new Pose2d(-42, 33, Math.toRadians(0)))
                    .build();
            Trajectory trajbl4 = drive.trajectoryBuilder(trajbl3.end())
                    .lineTo(new Vector2d(-42, -12))
                    .build();
            Trajectory trajbl5 = drive.trajectoryBuilder(trajbl4.end())
                    .lineTo(new Vector2d(40, -13))
                    .build();
            Trajectory trajbl6 = drive.trajectoryBuilder(trajbl5.end())
                    .splineToConstantHeading(new Vector2d(45, 32), Math.toRadians(5))
                    .build();


            Trajectory trajbs = drive.trajectoryBuilder(startPose)
                    .lineTo(new Vector2d(-36, 24.5))
                    .build();
            Trajectory trajbs2 = drive.trajectoryBuilder(trajbs.end())
                    .lineTo(new Vector2d(-37, 40))
                    .build();
            Trajectory trajbs3 = drive.trajectoryBuilder(trajbs2.end())
                    .lineToLinearHeading(new Pose2d(-48, 40, Math.toRadians(0)))
                    .build();
            Trajectory trajbs4 = drive.trajectoryBuilder(trajbs3.end())
                    .lineTo(new Vector2d(-55, 13))
                    .build();
            Trajectory trajbs5 = drive.trajectoryBuilder(trajbs4.end())
                    .lineTo(new Vector2d(45, 13))
                    .build();
            Trajectory trajbs6 = drive.trajectoryBuilder(trajbs5.end())
                    .splineToConstantHeading(new Vector2d(43, 35.5), Math.toRadians(13))
                    .build();

            Trajectory trajP = drive.trajectoryBuilder(new Pose2d())
                    .forward(10)
                    .build();


            switch (side) {
                case LEFT_SIDE:
                    pixelplacer.setPosition(0.65);
                    sleep(500);
                    drive.followTrajectory(trajbl);
                    drive.followTrajectory(trajbl2);
                    drive.followTrajectory(trajbl3);
                    pixelplacer.setPosition(placepixel);
                    sleep(500);
                    drive.followTrajectory(trajbl4);
                    drive.followTrajectory(trajbl5);
                    if (isPark == 1) {
                        drive.followTrajectory(trajP);
                    } else {
                        drive.followTrajectory(trajbl6);
                    }
                    break;
                case RIGHT_SIDE:

                    drive.followTrajectory(trajbr);
                    drive.followTrajectory(trajbr2);
                    drive.followTrajectory(trajbr3);
                    drive.followTrajectory(trajbr4);
                    drive.followTrajectory(trajbr5);
                    if (isPark == 1) {
                        drive.followTrajectory(trajP);
                    } else {
                        drive.followTrajectory(trajbr6);
                    }

                    break;
                case MIDDLE_SIDE:
                    ;
                    drive.followTrajectory(trajbs);
                    drive.followTrajectory(trajbs2);
                    drive.followTrajectory(trajbs3);
                    drive.followTrajectory(trajbs4);
                    drive.followTrajectory(trajbs5);
                    if (isPark == 1) {
                        drive.followTrajectory(trajP);
                    } else {
                        drive.followTrajectory(trajbs6);
                    }


            }

            lineup(0.3);
            sleep(500);

        } //</BLUEBLUE>


        if (isPark == 0) {
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

            while (AprilPortal.getCameraState() != VisionPortal.CameraState.STREAMING && !isStopRequested()) {
            }

            if (!isStopRequested()) {
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
            desiredTag = null;
            int DESIRED_TAG_ID;
            boolean loopBreak = false;
            boolean loopBreak2 = false;
            if (isRed == 1) {
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
            } else {
                switch (side) {
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

            while (!loopBreak && !isStopRequested()) {
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
                            strafe(0.2);
                        } else if (detection.id == 3) {
                            strafe(-0.2);
                        } else if (detection.id == 2) {
                            if (DESIRED_TAG_ID == 1) {
                                strafe(-0.2);
                            } else {
                                strafe(0.2);
                            }

                        }
                    } else {
                        if (detection.id == 4) {
                            strafe(0.2);
                        } else if (detection.id == 6) {
                            strafe(-0.2);
                        } else if (detection.id == 5) {
                            if (DESIRED_TAG_ID == 4) {
                                strafe(-0.2);
                            } else {
                                strafe(0.2);
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

                    led.setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);

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
                    .strafeRight(14.5)
                    .build();

            Trajectory final3 = drive.trajectoryBuilder(startPose)
                    .back(10)
                    .build();


            //sleep(500);


            lineup(0.3);
            goforward(0.3);
            sleep(500);

                pixelplacer.setPosition(0.65);
                sleep(200);
                pixelplacer.setPosition(pixelplaced);
                sleep(1000);
                drive.followTrajectory(final3);


                drive.followTrajectory(final1);
                setpower(0.3, true);
                sleep(250);
                setpower(0, true);
                sleep(200);
                pixelplacer.setPosition(0.65);
                sleep(200);
                pixelplacer.setPosition(pixelplaced);
                sleep(1000);
                drive.followTrajectory(final3);

        }else {
            sleep(1000);
            requestOpModeStop();
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
            if(frontldist.getDistance(DistanceUnit.INCH) - frontrdist.getDistance(DistanceUnit.INCH) > 0.25){
                setpower(y,false);
                led.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);

            } else if (frontldist.getDistance(DistanceUnit.INCH) - frontrdist.getDistance(DistanceUnit.INCH) < -0.25) {
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
