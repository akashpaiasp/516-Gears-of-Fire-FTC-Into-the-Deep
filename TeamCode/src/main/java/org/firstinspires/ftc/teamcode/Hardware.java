package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    public DcMotorEx vertLift;
    public DcMotorEx hangL;
    public DcMotorEx hangR;
    public Servo horiLiftL;
    public Servo horiLiftR;

    //public CRServo wheel1;
    //public DcMotorEx liftExtender;adb connect 192.168.43.1:5555
    public Servo claw;
    public Servo clawExtenderL;
    public Servo clawExtenderR;

    public Servo angle;


    private static Hardware myInstance = null;
    public double maxSpeed = 1;

    public final double CLAW_CLOSE = .45;
    public final double CLAW_OPEN = .05;
    public final double CLAW_FULL_OPEN = 0;

    public final double HORIZONTAL_LEFT_OUT = .11;
    public final double HORIZONTAL_LEFT_IN = .95;
    public final double HORIZONTAL_RIGHT_OUT = .93;
    public final double HORIZONTAL_RIGHT_IN = .1;
    public final double EXTENDER_L_UP = .51;
    public final double EXTENDER_L_DOWN = 1;
    public final double EXTENDER_L_MIDDLE = .72;
    public final double EXTENDER_L_FULL_UP = .42;
    public final double EXTENDER_L_SPECIMEN_UP = .48;
    public final double EXTENDER_L_FULLDOWN = 1;
    public final double EXTENDER_R_UP = .82;
    public final double EXTENDER_R_DOWN = .33;
    public final double EXTENDER_R_FULLDOWN = .315;
    public final double EXTENDER_R_MIDDLE = .6;
    public final double EXTENDER_R_FULL_UP = .91;
    public final double EXTENDER_R_SPECIMEN_UP = .84;
    public final double ANGLE_FORWARD = .488;
    public final double ANGLE_SIDEWAYS = .16;
    public final double ANGLE_L = .658;
    public final double ANGLE_R = .917;
    public final int SUBMERSIBLE_PICKUP = -300;
    public final int VERT_HIGH = -3940;
    public final int WALL = -375;
    public final int RUNG = -2300;
    public final int RUNG_DOWN = -1400;
    public final int HANG_1 = 500;
    public final int HANG_2 = 0;
    public final int VERT_SPECIMEN_AKASH = -1006;
    /*
    public final int hangLLow = -1500;
    public final int hangRLow = -600;
    public final int hangLUp = 4508;
    public final int hangRUp = 3751;
    public final int hangLHang = 39;
    public final int hangRHang = -695; */
    public final int hangAutoL = 7029;








    public Hardware() {}

    public static Hardware getInstance() {
        if (myInstance == null) {
            myInstance = new Hardware();
        }
        return myInstance;
    }

    public void init(HardwareMap hwMap) {

        vertLift = hwMap.get(DcMotorEx.class,"vertLift");
        vertLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        vertLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        vertLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //vertLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);
        vertLift.setPower(0);

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

        lb = hwMap.get(DcMotor.class, "lb");
        lb.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setPower(0);

        horiLiftL = hwMap.get(Servo.class, "horiliftL");
        horiLiftL.setPosition(HORIZONTAL_LEFT_IN);

        horiLiftR = hwMap.get(Servo.class, "horiliftR");
        horiLiftR.setPosition(HORIZONTAL_RIGHT_IN);

        claw = hwMap.get(Servo.class, "claw");
        claw.setPosition(CLAW_OPEN);

        clawExtenderL = hwMap.get(Servo.class, "extenderL");
        clawExtenderL.setPosition(EXTENDER_L_MIDDLE);

        clawExtenderR = hwMap.get(Servo.class, "extenderR");
        clawExtenderR.setPosition(EXTENDER_R_MIDDLE);

        angle = hwMap.get(Servo.class, "angle");
        angle.setPosition(ANGLE_SIDEWAYS);

        hangL = hwMap.get(DcMotorEx.class, "hangL");
        hangL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangL.setPower(0);

        hangR = hwMap.get(DcMotorEx.class, "hangR");
        hangR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hangR.setPower(0);
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

    public void horizontalIn() {
        horiLiftL.setPosition(HORIZONTAL_LEFT_IN);
        horiLiftR.setPosition(HORIZONTAL_RIGHT_IN);
    }
    public void horizontalOut() {
        horiLiftL.setPosition(HORIZONTAL_LEFT_OUT);
        horiLiftR.setPosition(HORIZONTAL_RIGHT_OUT);
    }

}