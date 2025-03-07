package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Autonomous(name="HangReset")
public class HangReset extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    Hardware robot;
    public void runOpMode() {
        robot = new Hardware();
        robot.init(hardwareMap);
        waitForStart();
        robot.hangR.setPower(-1);
        robot.hangL.setPower(-1);
        robot.hangR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.hangL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(100);
        while (robot.hangR.getCurrent(CurrentUnit.AMPS) < 3) {
            sleep(10);
        }
        robot.hangR.setPower(0);
        while (robot.hangL.getCurrent(CurrentUnit.AMPS) < 3) {
            sleep(10);
        }
        robot.hangL.setPower(0);
        robot.hangR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.hangL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }
}
