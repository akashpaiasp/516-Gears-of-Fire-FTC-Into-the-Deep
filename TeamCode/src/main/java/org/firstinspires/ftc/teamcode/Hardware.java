package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

public class Hardware {
    //rf = lf
    //lf = rf
    //rb = lb
    //lb = rb
    //public BNO055IMU gyro;
    public DcMotor rf;
    public DcMotor rb;
    public DcMotor lf;
    public DcMotor lb;
    public DcMotor vertLift;
    public Servo horiLift;
    //public CRServo wheel1;
    //public DcMotorEx liftExtender;
    public Servo claw;
    //public Servo claw2;

    public Servo angleCorrector;


    private static Hardware myInstance = null;
    public double maxSpeed = 1;

    public final double CLAW_CLOSE = 0.41;
    public final double CLAW_OPEN = 0.3;

    public final double HORIZONTAL_IN = 0.73;
    public final double HORIZONTAL_OUT = 0.39;
    public final double ANGLE_UP = 0.7;
    public final double ANGLE_DOWN = 0.21;

    public Hardware() {}

    public static Hardware getInstance() {
        if (myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {

        //vertLift = hwMap.get(DcMotor.class,"lift");
        //vertLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //vertLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //vertLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);
        //vertLift.setPower(0);

        rf = hwMap.get(DcMotor.class, "rf");
        //rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setPower(0);

        lf = hwMap.get(DcMotor.class, "lf");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setPower(0);

        rb = hwMap.get(DcMotor.class, "rb");
        //rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setPower(0);

        horiLift = hwMap.get(Servo.class, "horilift");
        horiLift.setPosition(HORIZONTAL_IN);
        claw = hwMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN);

        lb = hwMap.get(DcMotor.class, "lb");
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setPower(0);

        angleCorrector = hwMap.get(Servo.class, "angle");
        angleCorrector.setPosition(ANGLE_UP);

    }
    public void initAuto(HardwareMap hwMap) {
        vertLift = hwMap.get(DcMotor.class,"lift");
        vertLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        vertLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);
        vertLift.setPower(0);

        rf = hwMap.get(DcMotor.class, "rf");
        //rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setPower(0);

        lf = hwMap.get(DcMotor.class, "lf");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setPower(0);

        rb = hwMap.get(DcMotor.class, "rb");
        //rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setPower(0);

        horiLift = hwMap.get(Servo.class, "horilift");
        horiLift.setPosition(HORIZONTAL_IN);
        claw = hwMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN);

        lb = hwMap.get(DcMotor.class, "lb");
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setPower(0);

        angleCorrector = hwMap.get(Servo.class, "angle");
        angleCorrector.setPosition(ANGLE_UP);
    }

    public void setPower(double fr, double br, double fl, double bl) {
        rf.setPower(Range.clip(fr, -maxSpeed, maxSpeed));
        rb.setPower(Range.clip(br, -maxSpeed, maxSpeed));
        lf.setPower(Range.clip(fl, -maxSpeed, maxSpeed));
        lb.setPower(Range.clip(bl, -maxSpeed, maxSpeed));
    }

    public void liftsetPower(double apow) {
        //vertLift.setPower(apow);
    }

    public void setclaw1Pos(double pos) {
        claw.setPosition(pos);
    }

    public void resetMotors() {
        setPower(0, 0, 0, 0);

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rb.setDirection(DcMotorSimple.Direction.FORWARD);
    }

}