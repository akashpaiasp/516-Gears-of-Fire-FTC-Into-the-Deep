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
import java.util.ArrayList;
import java.util.List;

@Config
@Autonomous(name="Specimen Auto")
public class AutoSpecimen extends LinearOpMode {
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
            robot.vertLift.setTargetPosition(50);
            robot.vertLift.setPower(1);
            robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
            //robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
            //robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULLDOWN);
            //robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULLDOWN);
            return false;
        }
    }
    public class SlideSpecimen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.vertLift.setTargetPosition(robot.VERT_SPECIMEN_AKASH);
            robot.vertLift.setPower(1);
            robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            /*robot.clawExtenderR.setPosition(robot.EXTENDER_R_SPECIMEN_UP);
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_SPECIMEN_UP); */
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
    public class ExtenderMiddle implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_MIDDLE);
            robot.clawExtenderR.setPosition(robot.EXTENDER_R_MIDDLE);
            return false;
        }
    }
    public class ExtenderSpecimen implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_SPECIMEN_UP);
            robot.clawExtenderR.setPosition(robot.EXTENDER_R_SPECIMEN_UP);
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
    public class Angle45R implements Action {
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.angle.setPosition(robot.ANGLE_R);
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

    public Action slideUp() {return new SlideUp();}
    public Action slideSpecimen() {return new SlideSpecimen();}

    public Action slideDown() {return new SlideDown();}
    public Action extenderDown() {return new ExtenderDown();}
    public Action extenderUp() {return new ExtenderUp();}
    public Action extenderBack() {return new ExtenderBack();}
    public Action extenderMiddle() {return new ExtenderMiddle();}
    public Action extenderSpecimen() {return new ExtenderSpecimen();}
    public Action horiOut() {return new HoriOut();}
    public Action horiIn() {return new HoriIn();}
    public Action clawClose() {return new ClawClose();}
    public Action clawOpen() {return new ClawOpen();}
    public Action angleForward() {return new AngleStraight();}

    public Action angleSideways() {return new AngleSideways();}

    public Action angle45L() {return new Angle45L();}
    public Action angle45R() {return new Angle45R();}

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
                .waitSeconds(.5)
                .lineToX(26.5);
        TrajectoryActionBuilder tab2 = tab1.endTrajectory().fresh()
                .afterTime(.2, horiIn())
                .afterTime(1, slideDown())
                .afterTime(1, extenderMiddle())

                .waitSeconds(.4)
                .setTangent(Math.toRadians(76))
                .lineToX(19, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                //.setTangent(Math.toRadians(-90))
                .setTangent(0)
                .lineToXLinearHeading(48.9, Math.toRadians(0), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-34.7, 0, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                //.splineToSplineHeading(new Pose2d(48.9, -37.8, Math.toRadians(0)), Math.toRadians(180))
                //.waitSeconds(.0000001)
                .setTangent(0)
                .lineToXLinearHeading(10, 0, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                .lineToXLinearHeading(48.9, 0, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                //.splineToSplineHeading(new Pose2d(49.98, -47.89, Math.toRadians(0)), Math.toRadians(-60))
                .setTangent(Math.toRadians(90))
                .lineToYLinearHeading(-47.8, 0, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                //.waitSeconds(.0001)
                .setTangent(0)
                .lineToX(20, new TranslationalVelConstraint(80), new ProfileAccelConstraint(-80, 80))
                .turnTo(Math.toRadians(-180))
                .setTangent(0)
                .lineToX(9, new TranslationalVelConstraint(50));
        TrajectoryActionBuilder tab3 = tab2.endTrajectory().fresh()
                .afterTime(.2, slideSpecimen())
                .waitSeconds(.35)
                .afterTime(.4, extenderSpecimen())
                .afterTime(1.4, horiOut())
                .setTangent(Math.toRadians(0))
                .lineToXLinearHeading(19, Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(28.5, -1, Math.toRadians(0)), Math.toRadians(0));
        TrajectoryActionBuilder tab4 = tab3.endTrajectory().fresh()
                .waitSeconds(.2)
                .afterTime(.35, horiIn())
                .afterTime(.9, slideDown())
                .afterTime(.2, extenderMiddle())
                .setTangent(0)
                .lineToX(14)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(13, -24, Math.toRadians(180)), Math.toRadians(270))
                .setTangent(0)
                .lineToX(9, new AngularVelConstraint(Math.PI));
        TrajectoryActionBuilder tab5 = tab4.endTrajectory().fresh()
                .afterTime(.2, slideSpecimen())
                .waitSeconds(.35)
                .afterTime(.4, extenderSpecimen())
                .afterTime(1.4, horiOut())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(31, -2, Math.toRadians(0)), Math.toRadians(0));
        TrajectoryActionBuilder tab6 = tab5.endTrajectory().fresh()
                .waitSeconds(.2)
                .afterTime(.35, horiIn())
                .afterTime(.9, slideDown())
                .afterTime(.2, extenderMiddle())
                .setTangent(0)
                .lineToX(14)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(13, -26, Math.toRadians(180)), Math.toRadians(270))
                .setTangent(0)
                .lineToX(9, new AngularVelConstraint(Math.PI));
        TrajectoryActionBuilder tab7 = tab6.endTrajectory().fresh()
                .afterTime(.2, slideSpecimen())
                .waitSeconds(.35)
                .afterTime(.4, extenderSpecimen())
                .afterTime(1.4, horiOut())
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(29, -3, Math.toRadians(0)), Math.toRadians(0));

        Action trajectoryActionCloseOut = tab6.endTrajectory().fresh()
                //.waitSeconds(1)
                .build();
        waitForStart();
        // actionBuilder builds from the drive steps passed to it
        robot.vertLift.setTargetPosition(robot.VERT_SPECIMEN_AKASH);
        robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertLift.setPower(1);
        robot.clawExtenderR.setPosition(robot.EXTENDER_R_SPECIMEN_UP);
        robot.clawExtenderL.setPosition(robot.EXTENDER_L_SPECIMEN_UP);
        robot.horizontalOut();



        //Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
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
                        trajectoryActionCloseOut
                )
        );
    }
}
