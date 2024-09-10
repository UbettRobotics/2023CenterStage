package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeeptesting2 {
    public static void main(String[] args) {
        // Declare a MeepMeep instance
        // With a field size of 800 pixels
        MeepMeep meepMeep = new MeepMeep(720);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Required: Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(31, 30, Math.toRadians(180), Math.toRadians(180), 17)
                // Option: Set theme. Default = ColorSchemeRedDark()
                .setColorScheme(new ColorSchemeRedDark())

                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, 90))

                                .build()


                );
        // Declare out second bot
        RoadRunnerBotEntity mySecondBot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeRedLight())
                .setConstraints(31, 30, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, 90))
                                .lineTo(new Vector2d(-36,-24))
                                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineTo(new Vector2d(0,-10),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(48,-30,Math.toRadians(0)),Math.toRadians(350))

                                .build()
                );
        // Declare out second bot
        RoadRunnerBotEntity myThirdbot = new DefaultBotBuilder(meepMeep)
                // We set this bot to be red
                .setColorScheme(new ColorSchemeBlueLight())
                .setConstraints(31, 30, Math.toRadians(180), Math.toRadians(180), 17)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -60, 90))
                                .splineToSplineHeading(new Pose2d(-35, -12, Math.toRadians(0)), Math.toRadians(0))
                                .splineTo(new Vector2d(0,-10),Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(48,-30,Math.toRadians(0)),Math.toRadians(350))

                                .build()
                );

        // Set field image
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_OFFICIAL)
                .setDarkMode(true)
                // Background opacity from 0-1
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(mySecondBot)
                .addEntity(myThirdbot)
                .start();
    }
}