package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.claw;
import static org.firstinspires.ftc.teamcode.Robot.clawflipper;
import static org.firstinspires.ftc.teamcode.Robot.clawgrab;
import static org.firstinspires.ftc.teamcode.Robot.clawretreat;
import static org.firstinspires.ftc.teamcode.Robot.initMotors;
import static org.firstinspires.ftc.teamcode.Robot.slidemotor;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

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


@Autonomous(name = "Close55Testing", preselectTeleOp = "Teleop1")
@Disabled


public class TRESTINGSD42135u extends LinearOpMode {
    int new90 = 87;
    int isRed = 0;

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel


    final double DESIRED_DISTANCE = 20.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    double  Adrive           = 0.;        // Desired forward power/speed (-1 to +1)
    double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
    double  turn            = 0;        // Desired turning power/speed (-1 to +1)




    @Override
    public void runOpMode() throws InterruptedException {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);


        initMotors(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        /// CAMERA STUFF
        int parking = 1; // read by camera
        int iPark =0;

        CubeVisionPipeline pipeline = new CubeVisionPipeline(telemetry, 1, 0);
        webcam.setPipeline(pipeline);


        ////////AT
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
        boolean targetFound = false;

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
            }
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                pipeline.setColor(1);//RED
            }
        }
        waitForStart();
        isRed = pipeline.getColor();

        telemetry.update();
        Side side = pipeline.getSide();

        telemetry.update();

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
        int DESIRED_TAG_ID = 2;
        boolean loopBreak = false;

        while(!loopBreak) {
            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = TagProcess.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    targetFound = false;
                }
            }

            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", desiredTag.id + " " + desiredTag.metadata.name);
                telemetry.addData("Range", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", desiredTag.ftcPose.yaw);
            } else {

                telemetry.addData(">", "No Target");
            }
            telemetry.update();




            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double headingError = desiredTag.ftcPose.bearing;
                double yawError = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                Adrive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", Adrive, strafe, turn);
            } else {
                Adrive = 0;
                turn = 0;
                strafe = 0;
            }
            moveRobot(Adrive, strafe, turn);

            if(desiredTag != null) {
                if (Math.abs(desiredTag.ftcPose.yaw - 0) < 1 && Math.abs(desiredTag.ftcPose.bearing - 0) < 4
                        && Math.abs(desiredTag.ftcPose.range - 20) < 4) {
                    loopBreak = true;
                    moveRobot(0, 0, 0);
                }
            }
        }

        telemetry.addData("Done", true);
        telemetry.update();

    }
    private void closeCV(OpenCvCamera wc) {
        wc.stopStreaming();
        wc.closeCameraDevice();
        telemetry.addData("CLOSE",0);
        telemetry.update();

    }

    public void moveRobot(double x, double y, double yaw) {
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

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }
}
