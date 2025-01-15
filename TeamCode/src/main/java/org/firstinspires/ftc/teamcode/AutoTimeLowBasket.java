package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Low Basket")
public class AutoTimeLowBasket extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot;
    public void runOpMode() {
        robot = new Hardware();
        robot.init(hardwareMap);
        robot.clawExtenderR.setPosition(robot.EXTENDER_R_FULL_UP);
        robot.clawExtenderL.setPosition(robot.EXTENDER_L_FULL_UP);
        robot.claw.setPosition(robot.CLAW_CLOSE);
        waitForStart();
        robot.vertLift.setTargetPosition(robot.RUNG);
        robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertLift.setPower(1);
        robot.clawExtenderL.setPosition(robot.EXTENDER_L_UP);
        robot.clawExtenderR.setPosition(robot.EXTENDER_R_UP);
        robot.clawExtenderL.setPosition(robot.EXTENDER_L_MIDDLE);
        sleep(1000);
        robot.lf.setPower(0.25);
        robot.rf.setPower(0.25);
        robot.lb.setPower(0.25);
        robot.rb.setPower(0.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 1)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.setPower(0, 0, 0, 0);
        sleep(250);
        robot.claw.setPosition(robot.CLAW_OPEN);
        sleep(750);

        robot.lf.setPower(-0.25);
        robot.rf.setPower(-0.25);
        robot.lb.setPower(-0.25);
        robot.rb.setPower(-0.25);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.setPower(0,0,0,0);
        robot.vertLift.setTargetPosition(0);
        robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertLift.setPower(1);
        sleep(23500);
        robot.setPower(-.75,-.75,-.75,-.75);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 2.7)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //sleep(100);
        /*
        robot.lf.setPower(0.75);
        robot.rf.setPower(-0.75);
        robot.lb.setPower(-0.75);
        robot.rb.setPower(0.75);

        runtime.reset();
        robot.vertLift.setPower(1);
        robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.vertLift.setTargetPosition(0);
        robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_IN);
        robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_IN);
        while (opModeIsActive() && (runtime.seconds() < 4)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        } */


    }
}
