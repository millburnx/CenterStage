package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity farSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-34, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-34, -34, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-34, -39, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-60, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -36, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(180)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(40, -34, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(50, -30, Math.toRadians(0)))
                                .waitSeconds(0.75)
                                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -24, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-55, -24, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-60, -24, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -24, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(-57, -10, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(20, -10, Math.toRadians(180)))
                                .waitSeconds(1)
                                .lineToLinearHeading(new Pose2d(40, -34, Math.toRadians(0)))
                                .waitSeconds(0.5)
                                .lineToLinearHeading(new Pose2d(50, -34, Math.toRadians(0)))
                                .waitSeconds(0.75)
                                .build()
                );
        RoadRunnerBotEntity closeSide = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 10)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -62, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(12, -34, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(12, -60, Math.toRadians(90)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(farSide)
                .start();
    }
}