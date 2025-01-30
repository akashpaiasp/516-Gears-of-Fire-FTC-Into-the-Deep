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
    public class ActionsL {
        public class GroundPickup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.vertLift.setTargetPosition(0);
                robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
                robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_MIDDLE);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_MIDDLE);
                return true;
            }
        }
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
                .lineToX(20.85)
                .waitSeconds(2.5)
                //.setTangent(Math.toRadians(-82.45))
                //.line
                //.lineToYLinearHeading(-14.49, Math.toRadians(-82.45))
                //.setTangent(Math.atan2(25.99, 15.14))
                //.lineToXLinearHeading(15.14, Math.toRadians(-78.6))
                //.setTangent(Math.toRadians(-150))
                .setTangent(Math.toRadians(-125))
                .splineToSplineHeading(new Pose2d(16.5, -25.99, Math.toRadians(-80)), Math.toRadians(-125))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(50))
                .splineToSplineHeading(new Pose2d(23.5, -9, Math.toRadians(45)), Math.toRadians(50))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(100))
                .splineToSplineHeading(new Pose2d(25.62, -25, Math.toRadians(-95)), Math.toRadians(100))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(-180))
                .splineToSplineHeading(new Pose2d(23.5, -9, Math.toRadians(45)), Math.toRadians(0))
                .waitSeconds(2.5)
                .setTangent(Math.toRadians(75))
                .splineToSplineHeading(new Pose2d(25.62, -12, Math.toRadians(-57)), Math.toRadians(75))
                .waitSeconds(1)
                .splineTo(new Vector2d(20.85, 0), 0)
                .waitSeconds(2.5);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .splineTo(new Vector2d(-2.58, -59.71), Math.toRadians(59.95))
                .build();
        waitForStart();
        // actionBuilder builds from the drive steps passed to it


        Action trajectoryActionChosen = tab1.build();
        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        trajectoryActionCloseOut
                )
        );
    }
}
