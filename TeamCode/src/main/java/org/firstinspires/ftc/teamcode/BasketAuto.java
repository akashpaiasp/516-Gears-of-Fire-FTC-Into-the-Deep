package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import java.lang.Math;

@Config
@Autonomous(name="Basket Auto")
public class BasketAuto extends LinearOpMode {
    Hardware robot;
        public class SlideDown implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.vertLift.setTargetPosition(0);
            robot.vertLift.setPower(1);
            robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
            //robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
            //robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULLDOWN);
            //robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULLDOWN);
            return false;
        }
    }

    public class Slide50 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.vertLift.setTargetPosition(15);
            robot.vertLift.setPower(1);
            robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
            //robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
            //robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULLDOWN);
            //robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULLDOWN);
            return false;
        }
    }



    public class SlideUp implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.vertLift.setTargetPosition(robot.VERT_HIGH);
            robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vertLift.setPower(1);
            return false;
        }
    }

    public class ExtenderUp implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_UP);
            robot.clawExtenderR.setPosition(robot.EXTENDER_R_UP);
            return false;
        }
    }
    public class ExtenderDown implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_DOWN);
            robot.clawExtenderR.setPosition(robot.EXTENDER_R_DOWN);
            return false;
        }
    }
    public class ExtenderBack implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULL_UP);
            robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULL_UP);
            return false;
        }
    }
    public class AngleSideways implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.angle.setPosition(robot.ANGLE_SIDEWAYS);
            return false;
        }
    }

    public class AngleStraight implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.angle.setPosition(robot.ANGLE_FORWARD);
            return false;
        }
    }
    public class Angle45L implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.angle.setPosition(robot.ANGLE_L);
            return false;
        }
    }
    public class Hangers implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.hangL.setTargetPosition(robot.hangAutoL);
            robot.hangL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.hangL.setPower(1);
            return false;
        }
    }

    public class HoriOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.horizontalOut();
            return false;
        }
    }

    public class HoriIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.horizontalIn();
            return false;
        }
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.claw.setPosition(robot.CLAW_CLOSE);
            return false;
        }
    }
    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.claw.setPosition(robot.CLAW_FULL_OPEN);
            return false;
        }
    }
    public Action hangers() {return new Hangers();}
    public Action slideUp() {return new SlideUp();}
    public Action slideDown() {return new SlideDown();}
    public Action extenderDown() {return new ExtenderDown();}
    public Action extenderUp() {return new ExtenderUp();}
    public Action extenderBack() {return new ExtenderBack();}
    public Action horiOut() {return new HoriOut();}
    public Action horiIn() {return new HoriIn();}
    public Action clawClose() {return new ClawClose();}
    public Action clawOpen() {return new ClawOpen();}
    public Action angleForward() {return new AngleStraight();}

    public Action angleSideways() {return new AngleSideways();}

    public Action angle45() {return new Angle45L();}
    public Action slide50() {return new Slide50();}


    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new PinpointDrive(hardwareMap, initialPose);
        robot = new Hardware();
        robot.init(hardwareMap);
        robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULL_UP);
        robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULL_UP);
        robot.claw.setPosition(robot.CLAW_CLOSE);



        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .waitSeconds(1.5)
                .lineToX(21);
                //.lineToX(27);
                //.setTangent(Math.toRadians(-82.45))
                //.line
                //.lineToYLinearHeading(-14.49, Math.toRadians(-82.45))
                //.setTangent(Math.atan2(25.99, 15.14))
                //.lineToXLinearHeading(15.14, Math.toRadians(-78.6))
                //.setTangent(Math.toRadians(-150))
        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .waitSeconds(.3)
                .afterTime(.2, extenderBack())
                .afterTime(.4, slide50())
                .afterTime(.4, extenderDown())
                .afterTime(.4, angle45())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(/*8.49*/7.5, -28/*-29*/, Math.toRadians(-50)), Math.toRadians(-90))
                .waitSeconds(.5);
        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .waitSeconds(.3)
                .afterTime(.3, slideUp())
                .afterTime(.4, extenderUp())
                .afterTime(.4, angleSideways())
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(24, -9, Math.toRadians(53.5)), Math.toRadians(104));
                //.waitSeconds(1.3);
        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .waitSeconds(.4)
                .afterTime(.2, extenderBack())
                .afterTime(.4, slideDown())
                .afterTime(.4, extenderDown())
                .afterTime(.4, angleForward())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(/*15.4*/14.6, -35.2, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(.5);
        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .waitSeconds(.3)
                .afterTime(.3, slideUp())
                .afterTime(.4, extenderUp())
                .afterTime(.4, angleSideways())
                .waitSeconds(1.5)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(25, -8, Math.toRadians(51)), Math.toRadians(100));
                //.waitSeconds(1.3);
        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()
                .waitSeconds(.3)
                .afterTime(.2, extenderBack())
                .afterTime(.4, slide50())
                .afterTime(.4, extenderDown())
                .afterTime(.4, angleForward())
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(/*24*/24.3, -37.6, Math.toRadians(0)), Math.toRadians(0))
                .waitSeconds(.5);
        TrajectoryActionBuilder tab7 = tab6.endTrajectory().fresh()
                .waitSeconds(.3)
                .afterTime(.3, slideUp())
                .afterTime(.4, extenderUp())
                .afterTime(.4, angleSideways())
                .setTangent(0)
                .lineToX(21)
                .waitSeconds(1)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(25, -8, Math.toRadians(51)), Math.toRadians(100));
                //.waitSeconds(1.3);
        TrajectoryActionBuilder trajectoryActionCloseOut = tab7.endTrajectory().fresh()
                .waitSeconds(.3)
                .setTangent(Math.toRadians(45))
                .lineToX(19)
                .afterTime(.2, extenderBack())
                .afterTime(.4, slideDown())
                .waitSeconds(.5);
        TrajectoryActionBuilder end = trajectoryActionCloseOut.endTrajectory().fresh()
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-10, -61.7, 0), Math.toRadians(180))
                .lineToX(-11, new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10, 10));
                        //.waitSeconds();
        waitForStart();
        // actionBuilder builds from the drive steps passed to it


        //Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        slideUp(),
                        extenderUp(),
                        tab1.build(),
                        clawOpen(),
                        tab2.build(),
                        clawClose(),
                        tab3.build(),
                        clawOpen(),
                        tab4.build(),
                        clawClose(),
                        tab5.build(),
                        clawOpen(),
                        tab6.build(),
                        clawClose(),
                        tab7.build(),
                        clawOpen(),
                        trajectoryActionCloseOut.build(),
                        hangers(),
                        end.build()
                )
        );
    }
}
