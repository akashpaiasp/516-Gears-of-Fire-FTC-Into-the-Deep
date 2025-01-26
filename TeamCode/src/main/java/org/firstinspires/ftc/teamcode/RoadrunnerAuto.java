package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Config
@Autonomous(name="RoadrunnerAuto")
public class RoadrunnerAuto extends LinearOpMode {
    Hardware robot;
    public class Actions {
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
        robot = new Hardware();
        robot.init(hardwareMap);
        robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULL_UP);
        robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULL_UP);
        robot.claw.setPosition(robot.CLAW_CLOSE);
        waitForStart();
    }
}
