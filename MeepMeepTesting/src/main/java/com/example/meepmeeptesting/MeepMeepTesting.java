package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, 0, 0))
                        .lineToX(20.85)
                        .waitSeconds(2.5)
                        //.setTangent(Math.toRadians(-82.45))
                        //.line
                        //.lineToYLinearHeading(-14.49, Math.toRadians(-82.45))
                        .setTangent(0)
                        .splineToSplineHeading(new Pose2d(11.49, -14.84 , -74.25), Math.toRadians(-74.25))
                        .waitSeconds(2.5)
                        .splineTo(new Vector2d(20.85, 0), 0)
                        .waitSeconds(2.5)
                        .splineTo(new Vector2d(25.62, -14.36), Math.toRadians(-86.01))
                        .waitSeconds(2.5)
                        .splineTo(new Vector2d(20.85, 0), 0)
                        .waitSeconds(2.5)
                        .splineTo(new Vector2d(28.13, -16.51), Math.toRadians(-65.68))
                        .waitSeconds(2.5)
                        .splineTo(new Vector2d(20.85, 0), 0)
                        .waitSeconds(2.5)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.GRID_BLUE)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}