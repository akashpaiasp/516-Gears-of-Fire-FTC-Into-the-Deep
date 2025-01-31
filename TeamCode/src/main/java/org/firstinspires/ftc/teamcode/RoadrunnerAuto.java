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
@Autonomous(name="RoadrunnerAuto")
public class RoadrunnerAuto extends LinearOpMode {
    Hardware robot;
        public class GroundPickup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.vertLift.setTargetPosition(100);
                //robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
                //robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
                //robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULLDOWN);
                //robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULLDOWN);
                return false;
            }

        }
    public Action groundPickup() {
        return new GroundPickup();
    }
        public class Basket implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.vertLift.setTargetPosition(robot.VERT_HIGH);
                robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertLift.setPower(1);
                //robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
                //robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_UP);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_UP);
                return false;
            }
        }

    public class Basket45 implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.vertLift.setTargetPosition(robot.VERT_HIGH);
            robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.vertLift.setPower(1);
            //robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
            //robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_UP);
            robot.clawExtenderR.setPosition(robot.EXTENDER_R_UP);
            robot.angle.setPosition(robot.ANGLE_FORWARD);
            return false;
        }
    }


    public class Claw implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.claw.setPosition(robot.CLAW_FULL_OPEN);
                resetRuntime();
                while (getRuntime() < 0.5) {}
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULL_UP);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULL_UP);
                resetRuntime();
                while (getRuntime() < .3) {}
                return false;
            }
        }
        public Action claw() {
            return new Claw();
        }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULLDOWN);
            robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULLDOWN);
            resetRuntime();
            while (getRuntime() < 0.4) {}
            robot.claw.setPosition(robot.CLAW_CLOSE);
            resetRuntime();
            while (getRuntime() < 0.2) {}
            return false;
        }
    }
    public Action clawClose() {
        return new ClawClose();
    }
    public Action basket() {
        return new Basket();
    }
    public Action basket45() {
        return new Basket45();
    }

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
                //.setTangent(Math.toRadians(-82.45))
                //.line
                //.lineToYLinearHeading(-14.49, Math.toRadians(-82.45))
                //.setTangent(Math.atan2(25.99, 15.14))
                //.lineToXLinearHeading(15.14, Math.toRadians(-78.6))
                //.setTangent(Math.toRadians(-150))
        TrajectoryActionBuilder backUp = drive.actionBuilder(new Pose2d(22, 0, 0))
                .lineToX(18);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(18, 0, 0))
                .waitSeconds(.5)
                .setTangent(Math.toRadians(-105))
                .splineToSplineHeading(new Pose2d(16.75, -25.99, Math.toRadians(-85)), Math.toRadians(-105));
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(16.75, -25.99, Math.toRadians(-85)))
                .waitSeconds(2.1)
                .setTangent(Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(23.5, -9, Math.toRadians(45)), Math.toRadians(50));
        TrajectoryActionBuilder backUp2 = drive.actionBuilder(new Pose2d(23.5, -9, Math.toRadians(45)))
                .lineToX(20);
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(20, -9, Math.toRadians(45)))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(27, -25, Math.toRadians(-100)), Math.toRadians(100));
        TrajectoryActionBuilder tab5 = drive.actionBuilder(new Pose2d(27, -25, Math.toRadians(-100)))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(new Pose2d(23.5, -9, Math.toRadians(45)), Math.toRadians(-100));
        TrajectoryActionBuilder backUp3 = drive.actionBuilder(new Pose2d(23.5, -9, Math.toRadians(45)))
                .lineToX(20);
        TrajectoryActionBuilder tab6 = drive.actionBuilder(new Pose2d(20, -9, Math.toRadians(45)))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(-100))
                .splineToSplineHeading(new Pose2d(21.2, -25.18, Math.toRadians(-35.5)), Math.toRadians(-100));
        TrajectoryActionBuilder tab7 = drive.actionBuilder(new Pose2d(21.2, -25.18, Math.toRadians(-35.5)))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(23.5, -9, Math.toRadians(45)), Math.toRadians(100))
                .waitSeconds(2.5);
        TrajectoryActionBuilder backUp4 = drive.actionBuilder(new Pose2d(23.5, -9, Math.toRadians(45)))
                .lineToX(20);
        Action trajectoryActionCloseOut = backUp4.endTrajectory().fresh()
                .splineTo(new Vector2d(-2.58, -59.71), Math.toRadians(59.95))
                .build();
        waitForStart();
        // actionBuilder builds from the drive steps passed to it


        //Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        basket45(),
                        tab1.build(),
                        claw(),
                        backUp.build(),
                        groundPickup(),
                        tab2.build(),
                        clawClose(),
                        basket(),
                        tab3.build(),
                        claw(),
                        backUp2.build(),
                        groundPickup(),
                        tab4.build(),
                        clawClose(),
                        basket(),
                        tab5.build(),
                        claw(),
                        backUp3.build(),
                        groundPickup(),
                        tab6.build(),
                        clawClose(),
                        basket(),
                        tab7.build(),
                        claw(),
                        backUp4.build(),
                        trajectoryActionCloseOut
                )
        );
    }
}
