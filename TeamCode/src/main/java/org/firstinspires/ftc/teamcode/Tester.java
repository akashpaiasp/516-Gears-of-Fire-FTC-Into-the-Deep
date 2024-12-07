package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "tester")
public class Tester extends LinearOpMode {
    Hardware robot = Hardware.getInstance();
    private ElapsedTime run = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.vertLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        boolean pressingRB = false;
        double pos1 = 0;
        double pos2 = 1;
        double pos3 = .5;
        boolean pressingLB = false;
        boolean pressingA = false;
        boolean pressingB = false;
        boolean pressingX = false;
        boolean pressingY = false;
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("claw pos", robot.claw.getPosition());
            telemetry.addData("horizontal lift pos", robot.horiLift.getPosition());
            telemetry.addData("angle corrector pos", robot.angleCorrector.getPosition());
            telemetry.update();
            if(gamepad2.right_bumper && !pressingRB) {
                pos1 += 0.01;
                robot.claw.setPosition(pos1);
                pressingRB = true;
            } else if(!gamepad2.right_bumper) {
                pressingRB = false;
            }
            if(gamepad2.left_bumper && !pressingLB) {
                pos1 -= 0.01;
                robot.claw.setPosition(pos1);
                pressingLB = true;
            } else if(!gamepad2.left_bumper) {
                pressingLB = false;
            }
            if(gamepad2.a && !pressingA) {
                pos2 += 0.01;
                robot.horiLift.setPosition(pos2);
                pressingA = true;
            } else if(!gamepad2.a) {
                pressingA = false;
            }
            if(gamepad2.b && !pressingB) {
                pos2 -= 0.01;
                robot.horiLift.setPosition(pos2);
                pressingB = true;
            } else if(!gamepad2.b) {
                pressingB = false;
            }
            if(gamepad2.x && !pressingX) {
                pos3 += 0.01;
                robot.angleCorrector.setPosition(pos3);
                pressingX = true;
            } else if(!gamepad2.x) {
                pressingX = false;
            }
            if(gamepad2.y && !pressingY) {
                pos3 -= 0.01;
                robot.angleCorrector.setPosition(pos3);
                pressingY = true;
            } else if(!gamepad2.y) {
                pressingY = false;
            }
           /* boolean pressingX = false;
            if(gamepad2.x && !pressingX) {
                pos2 += 0.01;
                robot.claw2.setPosition(pos2);
                pressingA = true;
            } else if(!gamepad2.a) {
                pressingA = false;
            }
            if(gamepad2.b && !pressingB) {
                pos2 -= 0.01;
                robot.claw2.setPosition(pos2);
                pressingB = true;
            } else if(!gamepad2.b) {
                pressingB = false;
            }
            boolean leftTrigger = false;
            double pos3 = 0;
            if(gamepad2.left_trigger > 0 && !leftTrigger) {
                pos3 += 0.01;
                robot.launcher.setPosition(pos3);
                leftTrigger = true;
            } else if(!(gamepad2.left_trigger > 0)) {
                leftTrigger = false;
            }
            boolean leftTrigger2 = false;
            if(gamepad2.left_trigger < 0 && !leftTrigger2) {
                pos3 -= 0.01;
                robot.launcher.setPosition(pos3);
                leftTrigger2 = true;
            } else if(!(gamepad2.left_trigger < 0)) {
                leftTrigger2 = false;
            }*/
            //rf good
            //lf good
            //rb good
            //lb good
          /* if(gamepad1.a) {
               robot.liftExtender.setPower(0.4);
               robot.liftExtender.setTargetPosition(-1747);
               robot.liftExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
           }*/

        }
    }
}