package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {

    /*
    Start:
    new Pose2d(35, -72+13.6, Math.toRadians(90))

    .splineTo(new Vector2d(29, -8.5), Math.toRadians(120))
                                .waitSeconds(.4)
                                .lineTo(new Vector2d(42, -14))
                                .lineToSplineHeading(new Pose2d(56, -13, Math.toRadians(0)))
                                .waitSeconds(.4)
                                .splineTo(new Vector2d(42, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(29, -8.5), Math.toRadians(120))
                                .build()
     */


    // Sample code taken from Github MeepMeep repository below
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(53.1324198, 53.1324198, Math.toRadians(276.40123374613705), Math.toRadians(184.02607784577722), 22)
                .followTrajectorySequence(drive ->
                        // TODO notice how it starts at 34 not 36
                        drive.trajectorySequenceBuilder(new Pose2d(29, -9, Math.toRadians(315)))
                                /*
                                Takes: <5 seconds
                                STart: new Pose2d(29, -9, Math.toRadians(315))
                                .lineToSplineHeading(new Pose2d(56, -13.5, Math.toRadians(0)))
                                .waitSeconds(.4)
                                .splineTo(new Vector2d(42, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(29.5, -8), Math.toRadians(125))
                                .build()

                                 */
                                //.splineTo(new Vector2d(29, -8.5), Math.toRadians(120))
                                // Zone 2: .lineToLinearHeading(new Pose2d(36, -12, Math.toRadians(270))) // .8 sec
                                // Zone 1:
                                //.lineToLinearHeading(new Pose2d(12, -14, Math.toRadians(270))) // 1.18 sec
                                // Zone 3 far cone stack
                                .lineToSplineHeading(new Pose2d(56, -13.5, Math.toRadians(0))) // 1.59 sec
                                .build()

                        // 2.2 seconds to go to cone stack
                );

        myBot.setDimensions(16, 13.6);

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}