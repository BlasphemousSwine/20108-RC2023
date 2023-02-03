package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-35, 61, Math.toRadians(270)))
                                .forward(30)
                                //Junction
                                .splineTo(new Vector2d(-30, 13), Math.toRadians(325))
                                .setReversed(true)
                                //square2
                                .splineTo(new Vector2d(-35, 40), Math.toRadians(90))
                                .setReversed(false)
                                //Pickup
                                .splineTo(new Vector2d(-40, 20), Math.toRadians(200))
                                .forward(12)
                                .setReversed(true)
                                .back(14)
                                //square2
                                .splineTo(new Vector2d(-35, 40), Math.toRadians(90))
                                .setReversed(false)
                                //Junction
                                .splineTo(new Vector2d(-32, 17), Math.toRadians(315))
                                .setReversed(true)
                                //square2
                                .splineTo(new Vector2d(-35, 40), Math.toRadians(90))
                                .setReversed(false)
                                //Pickup
                                .splineTo(new Vector2d(-40, 20), Math.toRadians(190))
                                .forward(12)
                                .setReversed(true)
                                .back(14)
                                //square2
                                .splineTo(new Vector2d(-35, 37), Math.toRadians(90))
                                .setReversed(false)
                                //Junction
                                .splineTo(new Vector2d(-32, 17), Math.toRadians(315))
                                .setReversed(true)
                                .splineTo(new Vector2d(-35, 19), Math.toRadians(180))
                                .waitSeconds(2)
                                .build()
                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}