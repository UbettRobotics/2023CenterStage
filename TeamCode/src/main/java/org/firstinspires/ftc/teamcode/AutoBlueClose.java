package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name = "AutoClose")
@Disabled

public class AutoBlueClose extends LinearOpMode {
    @Override
    public void runOpMode() {
        initMotors(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        ////////////init camera and motors ////////////////////////////////////////////////////////////////////
        telemetry.addLine("Not initialized");
        telemetry.update();

        OpenCvCamera webcam;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);


        initMotors(this);


        /// CAMERA STUFF
        int parking = 1; // read by camera


        CubeVisionPipeline pipeline = new CubeVisionPipeline(telemetry, 0,1);
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
        slidemotor.setTargetPosition(50);
        slidemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slidemotor.setPower(.5);

        initMotors(this);
        clawflipper.setPosition(clawretreat);



        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(29)
                .build();

        Trajectory trajb = drive.trajectoryBuilder(new Pose2d())
                .back(12)
                .build();



        Trajectory trajl = drive.trajectoryBuilder(new Pose2d())
                .forward(12)
                .build();

        Trajectory trajr = drive.trajectoryBuilder(new Pose2d())
                .forward(15.5)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .forward(13.5)
                .build();



        CubeVisionPipeline.Side side = pipeline.getSide();

        clawflipper.setPosition(clawretreat);
        switch (side){
            case LEFT_SIDE:
                drive.followTrajectory(traj1);
                drive.turn(Math.toRadians(130));
                drive.followTrajectory(trajl);
                drive.followTrajectory(trajb);


                break;
            case RIGHT_SIDE:
                drive.followTrajectory(traj1);
                drive.turn(Math.toRadians(-125));
                drive.followTrajectory(trajr);
                drive.followTrajectory(trajb);

                break;
            case MIDDLE_SIDE:
                drive.followTrajectory(traj1);
                drive.turn(Math.toRadians(-30));
                drive.followTrajectory(traj3);
                drive.followTrajectory(trajb);



        }

        if(isStopRequested()) return;

        clawflipper.setPosition(clawflipped);


    }
}