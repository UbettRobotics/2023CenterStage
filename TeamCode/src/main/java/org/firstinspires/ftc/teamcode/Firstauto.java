package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "backup")
@Disabled

public class Firstauto extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(15)
                .build();

        waitForStart();

        drive.followTrajectory(traj1);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj1);
    }

}
