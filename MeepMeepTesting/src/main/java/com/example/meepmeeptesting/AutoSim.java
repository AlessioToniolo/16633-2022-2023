package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class AutoSim {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(53.1324198, 53.1324198, toRadians(276.40123374613705), toRadians(184.02607784577722), 22)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35, -72+13.6, toRadians(-90)))
                                .setReversed(true)

                                // Preload Deposit
                                .splineToSplineHeading(new Pose2d(26, -4, toRadians(-60)), toRadians(120))
                                .waitSeconds(1)

                                // Cone stack #1
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), toRadians(5))
                                .waitSeconds(.5)

                                // Deposit #1
                                .setReversed(true)
                                // TODO EDITS ARE BEELOW ON THIS CYCLE
                                .splineToSplineHeading(new Pose2d(24, -2, toRadians(-60)), toRadians(165))


                                // Cone stack #2
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(60, -9, Math.toRadians(0)), toRadians(5))
                                .waitSeconds(.5)

                                // Deposit #2
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(24, -2, toRadians(-60)), toRadians(165))

                                // TODO STOP HERE

                                // Cone Stack #3
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), toRadians(5))
                                .waitSeconds(.5)

                                // Deposit #3
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(26, -4, toRadians(-60)), toRadians(120))

                                // Cone Stack #4
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), toRadians(5))
                                .waitSeconds(.5)

                                // Deposit #4
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(26, -4, toRadians(-60)), toRadians(120))

                                // Cone Stack #5
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(0)), toRadians(5))
                                .waitSeconds(.5)

                                // Deposit #5
                                .setReversed(true)
                                .splineToSplineHeading(new Pose2d(26, -4, toRadians(-60)), toRadians(120))

                                /* PARKING */

                                // Zone #1
                                //.setReversed(false)
                                //.splineToLinearHeading(new Pose2d(12, -12, Math.toRadians(-90)), toRadians(-300))
                                /*
                                .turn(toRadians(180))
                                .waitSeconds(.5)

                                 */
                                // Zone #2
                                //.lineToLinearHeading(new Pose2d(36, -12, toRadians(90)))
                                // Zone #3
                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(60, -12, Math.toRadians(90)), toRadians(5))

                                .build()
                );

        myBot.setDimensions(16, 13.6);
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}