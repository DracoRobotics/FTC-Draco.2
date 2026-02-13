package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(170, 200,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        15)
                .build();


//        myBot.runAction(
//                myBot.getDrive().actionBuilder(new Pose2d(-51, -52, Math.toRadians(225)))
//                        .strafeTo( new Vector2d(-15, -15))
//                        .turnTo(270)
//                        .splineTo(new Vector2d(12, -32), Math.toRadians(270))
//                        .lineToY(-60)
//                        .lineToY(-30)
//                        .splineTo( new Vector2d(-15, -15), Math.toRadians(45))
//                        .turnTo(270)
//                        .lineToY(-60)
//                        .build()
//        );



//        Tartus
        myBot.runAction(
                    myBot.getDrive().actionBuilder(new Pose2d(-57, 57, Math.toRadians(135)))
                    .strafeTo(new Vector2d(-16, 16))
                    .waitSeconds(6)
                            .splineTo(new Vector2d(36, 34), Math.toRadians(-270))
                    .lineToY(60)
                    .lineToY(30)
                .splineTo(new Vector2d(-16, 16), Math.toRadians(-45))
                .turnTo(Math.toRadians(-225))
                .waitSeconds(6)
                            .splineTo(new Vector2d(12, 34), Math.toRadians(-270))
                .lineToY(60)
                .lineToY(30)
                .splineTo(new Vector2d(-16,16), Math.toRadians(-45))
                .turnTo(Math.toRadians(-225))
                .waitSeconds(5.9)
                            .splineTo(new Vector2d(38, 33), Math.toRadians(-270))
                .build()
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
