package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "AutoFarBlue", preselectTeleOp = "Teleop1")
@Disabled

public class AutoBlueRight extends LinearOpMode {
    @Override
    public void runOpMode() {
        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);


        initMotors(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int isPark = 1;

        /// CAMERA STUFF
        int parking = 1; // read by camera


        CubeVisionPipeline pipeline = new CubeVisionPipeline(telemetry, 0, 1);
        webcam.setPipeline(pipeline);

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
        waitForStart();

        telemetry.update();

        telemetry.update();
        ////Move on start/init
        claw.setPosition(clawretreat);
        sleep(200);
        slidemotor.setTargetPosition(5);
        slidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidemotor.setPower(.5);

        initMotors(this);
        clawflipper.setPosition(clawretreat);
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(66)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-45,-12,Math.toRadians(90)))
                .forward(95)

                .build();


        waitForStart();

        if(isStopRequested()) return;
        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(120));
        drive.followTrajectory(traj2);






    }
}