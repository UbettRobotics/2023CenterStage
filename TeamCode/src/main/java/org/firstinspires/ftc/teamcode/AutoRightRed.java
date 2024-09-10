package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robot.*;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TurnSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name = "AutoDFarRed", preselectTeleOp = "Teleop1")
@Disabled

public class AutoRightRed extends LinearOpMode {
    @Override
    public void runOpMode() {
        initMotors(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        clawflipper.setPosition(clawretreat);
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .forward(62)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(-45,-12,Math.toRadians(90)))
                .forward(50)

                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                        .forward(70)
                                .build();

        waitForStart();

        if(isStopRequested()) return;
        clawflipper.setPosition(clawretreat);
        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(-140));
        drive.followTrajectory(traj2);
        drive.turn(Math.toRadians(-90)); //50
        drive.followTrajectory(traj3);






    }
}