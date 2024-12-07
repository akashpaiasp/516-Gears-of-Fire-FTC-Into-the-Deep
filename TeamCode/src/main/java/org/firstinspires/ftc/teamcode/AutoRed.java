
package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

//shows what name appears on the control hub for the class (line below)
@Autonomous (name = "AutoFirstRedFar")
// defines class and lets me use code in the SDK (line below)
public class AutoRed extends LinearOpMode {
    private Hardware robot;

    public void runOpMode() throws InterruptedException {
        robot = new Hardware();
        robot.initAuto(hardwareMap);
        waitForStart();


    }

    public void Forward(int encoderPosition, double speedMoving) {

        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // finds the circumference of the wheel(line below)
        /*double wheelCircumference = 3 * Math.PI;
        //tells the gear ratio of the motor(line below);
        double wheelMotor = 560;
        //total ticks required to move your specified distance (line below)
        double ticks = (distance * (wheelMotor / wheelCircumference));
        //automatically rounds the number of ticks needed to move a certain distance to the nearest integer (1 line below )
        *///int encoderPosition = ((int) Math.round(ticks));
            robot.setPower(speedMoving, speedMoving, speedMoving, speedMoving);

            robot.lf.setTargetPosition(encoderPosition);
            robot.lb.setTargetPosition(encoderPosition);
            robot.rf.setTargetPosition(encoderPosition);
            robot.rb.setTargetPosition(encoderPosition);

            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        robot.setPower(0, 0, 0, 0);

        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void TurnLeft(int encoderPosition, double turnSpeed) {
        //robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        //robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*double wheelCircumference = 3 * Math.PI;

        double wheelMotor = 560;

        double ticks = (distance * (wheelMotor / wheelCircumference)); */

        robot.lf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lb.setDirection(DcMotorSimple.Direction.FORWARD);

        //int encoderPosition = ((int) Math.round(ticks));

            //double oppositeturnSpeed = -1.0 * turnSpeed;

            robot.setPower(turnSpeed, turnSpeed, -turnSpeed, -turnSpeed);
            //robot.lf.setPower(oppositeturnSpeed);
            //robot.rf.setPower(turnSpeed);
            //robot.lb.setPower(turnSpeed);
            //robot.rb.setPower(oppositeturnSpeed);
        /*
            telemetry.addData("rf", robot.rf.getPower());
            telemetry.addData("lf", robot.lf.getPower());
            telemetry.addData("lb", robot.lb.getPower());
            telemetry.addData("rb", robot.rb.getPower());
            telemetry.update(); */

            robot.rf.setTargetPosition(encoderPosition);
            robot.lf.setTargetPosition(encoderPosition);
            robot.rb.setTargetPosition(encoderPosition);
            robot.lb.setTargetPosition(encoderPosition);

            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        robot.setPower(0, 0, 0, 0);

        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void TurnRight(int encoderPosition, double TurnSpeed) {

        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*double wheelCircumference = 3 * Math.PI;

        double wheelMotor = 560;

        double ticks = (distance * (wheelMotor / wheelCircumference)); */

        robot.rf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rb.setDirection(DcMotorSimple.Direction.REVERSE);

        //int encoderPosition = ((int) Math.round(ticks));

        //while (!(robot.lf.getCurrentPosition() <= encoderPosition + CHANGE && robot.lf.getCurrentPosition() >= encoderPosition - CHANGE)) {
        robot.setPower(TurnSpeed, -TurnSpeed, -TurnSpeed, TurnSpeed);

        robot.rf.setTargetPosition(encoderPosition);
        robot.lf.setTargetPosition(encoderPosition);
        robot.rb.setTargetPosition(encoderPosition);
        robot.lb.setTargetPosition(encoderPosition);

        robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //}

        robot.setPower(0, 0, 0, 0);

        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
