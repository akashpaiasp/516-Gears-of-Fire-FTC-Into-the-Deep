/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */



@TeleOp(name="The 420 point teleop program", group="Linear OpMode")
public class Teleop extends LinearOpMode {
    private Hardware robot;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private double scaleFactor;
    boolean pressingRB = false;
    boolean pressingY = false;
    boolean pressingA = false;
    boolean clawOpen = true;
    boolean extenderDown = false;
    boolean horiIn = true;
    boolean slideIn = true;
    boolean manual = true;
    boolean pressingX = false;
    boolean pressingLT = false;
    boolean pressingRT = false;
    boolean pressingLB = false;
    boolean pressingDpad = false;
    boolean timer = false;
    boolean timer2 = false;
    double time = 0;
    boolean rung = true;
    boolean pressingBack = false;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() {
        robot = new Hardware();
        robot.init(hardwareMap);
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            scaleFactor = 1;
            scaleFactor *= Math.max(Math.abs(1 - gamepad1.right_trigger), .4);

            // Send calculated power to wheels
            robot.lf.setPower(leftFrontPower * scaleFactor);
            robot.rf.setPower(rightFrontPower * scaleFactor);
            robot.lb.setPower(leftBackPower * scaleFactor);
            robot.rb.setPower(rightBackPower * scaleFactor);

            // Show the elapsed game time and wheel power.
            /*telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Vert lift power", robot.vertLift.getPower());
            telemetry.addData("Vert lift pos", robot.vertLift.getCurrentPosition());
            telemetry.update(); */

            if (gamepad2.right_bumper && !pressingRB) { //&& !pressingRB) {
                pressingRB = true;
                if (clawOpen)
                    robot.claw.setPosition(robot.CLAW_CLOSE);
                else
                    robot.claw.setPosition(robot.CLAW_OPEN);
                pressingRB = true;
                clawOpen = !clawOpen;

            }
            else if(!gamepad2.right_bumper) {
                pressingRB = false;
            }

            //}
            /*else {
                pressingRB = false;
            } */

            if (gamepad2.y && !pressingY) {
                pressingY = true;
                if (horiIn) {
                    robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
                    robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);

                }
                else {
                    robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_IN);
                    robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_IN);
                }
                horiIn = !horiIn;
            }
            else if (!gamepad2.y) {
                pressingY = false;
            }

            if (gamepad2.a && !pressingA) {// && !pressingA) {
                pressingA = true;
                if (!extenderDown) {
                    robot.clawExtenderL.setPosition(robot.EXTENDER_L_DOWN);
                    robot.clawExtenderR.setPosition(robot.EXTENDER_R_DOWN);

                }
                else {
                    robot.clawExtenderL.setPosition(robot.EXTENDER_L_MIDDLE);
                    robot.clawExtenderR.setPosition(robot.EXTENDER_R_MIDDLE);
                }
                    extenderDown = !extenderDown;
            }
            else if (!gamepad2.a) {
                pressingA = false;
            }

            if (gamepad2.b && !pressingLT) {
                pressingLT = true;
                extenderDown = false;
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_UP);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_UP);
            }
            else if (!gamepad2.b) pressingLT = false;

            if (gamepad2.dpad_down && !pressingDpad) {
                pressingDpad = true;
                robot.angle.setPosition(robot.ANGLE_SIDEWAYS);
            }
            else if (gamepad2.dpad_up && !pressingDpad) {
                pressingDpad = true;
                robot.angle.setPosition(robot.ANGLE_FORWARD);
            }
            else if (gamepad2.dpad_left && !pressingDpad) {
                pressingDpad = true;
                robot.angle.setPosition(robot.ANGLE_L);
            }
            else if (gamepad2.dpad_right && !pressingDpad) {
                pressingDpad = true;
                robot.angle.setPosition(robot.ANGLE_R);
            }
            else if (!gamepad2.dpad_right && !gamepad2.dpad_up && !gamepad2.dpad_left && !gamepad2.dpad_down) {
                pressingDpad = false;
            }

            if (gamepad2.x && !pressingX) {
                pressingX = true;
                manual = false;
                if (rung)
                    robot.vertLift.setTargetPosition(robot.RUNG);
                else
                    robot.vertLift.setTargetPosition(robot.RUNG_DOWN);
                robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertLift.setPower(1);
                robot.angle.setPosition(robot.ANGLE_SIDEWAYS);
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_MIDDLE);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_MIDDLE);
                extenderDown = false;
                rung = !rung;
            }
            else if (!gamepad2.x) {
                pressingX = false;
            }

            //else {
            //    pressingA = false;
            //}


            if (gamepad2.left_stick_y > 0) {
                robot.vertLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.vertLift.setPower(Math.min(gamepad2.left_stick_y * 1.2, 1));
                manual = true;
            }
            else if (gamepad2.left_stick_y < 0) {
                robot.vertLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.vertLift.setPower(Math.max(gamepad2.left_stick_y * 1.2, -1));
                manual = true;
            }
            else if(manual) robot.vertLift.setPower(0);

            if (gamepad2.left_trigger > 0.5 && !pressingLT) {
                manual = false;
                pressingLT = true;
                robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertLift.setTargetPosition(robot.SUBMERSIBLE_PICKUP);
                robot.vertLift.setPower(1);
                robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
                robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_DOWN);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_DOWN);
                extenderDown = true;
            }
            else if(gamepad2.left_trigger < .5) pressingLT = false;

            if (gamepad2.right_trigger > 0.5 && !pressingRT) {
                robot.claw.setPosition(robot.CLAW_OPEN);
                clawOpen = true;
                robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertLift.setTargetPosition(0);
                robot.vertLift.setPower(1);
                pressingRT = true;
                manual = false;
                timer = true;
                time = getRuntime();
            }
            else if(gamepad2.right_trigger < .5) {
                pressingRT = false;
            }

            if (timer && getRuntime() > time + .2) {
                manual = false;
                timer = false;
                robot.claw.setPosition(robot.CLAW_CLOSE);
                clawOpen = false;
                time = getRuntime();
                timer2 = true;
            }
            if (timer2 && getRuntime() > time + .1) {
                timer2 = false;
                manual = false;
                robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertLift.setTargetPosition(robot.SUBMERSIBLE_PICKUP);
                robot.vertLift.setPower(1);
            }

            if (gamepad2.left_bumper && !pressingLB) {
                manual = false;
                pressingLB = true;
                robot.vertLift.setPower(1);
                robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertLift.setTargetPosition(0);
                robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_IN);
                robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_IN);
                robot.angle.setPosition(robot.ANGLE_SIDEWAYS);
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_MIDDLE);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_MIDDLE);
                extenderDown = false;
                horiIn = true;
            }
            else if (!gamepad2.left_bumper) {
                pressingLB = false;
            }

            if (gamepad2.back && !pressingBack) {
                robot.vertLift.setPower(1);
                robot.vertLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.vertLift.setTargetPosition(robot.WALL);
                robot.horiLiftR.setPosition(robot.HORIZONTAL_RIGHT_OUT);
                robot.horiLiftL.setPosition(robot.HORIZONTAL_LEFT_OUT);
                robot.clawExtenderL.setPosition(robot.EXTENDER_L_MIDDLE);
                robot.clawExtenderR.setPosition(robot.EXTENDER_R_MIDDLE);
                extenderDown = false;
                robot.claw.setPosition(robot.CLAW_OPEN);
                robot.angle.setPosition(robot.ANGLE_SIDEWAYS);
                clawOpen = true;
            }

        }
        }
    }
