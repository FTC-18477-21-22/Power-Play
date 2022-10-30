package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareBIGBRAINBOTS {
    public DcMotor FrontLeftDrive = null;
    public DcMotor FrontRightDrive = null;
    public DcMotor RearLeftDrive = null;
    public DcMotor RearRightDrive = null;


    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        FrontLeftDrive = hwMap.get(DcMotor.class, "FL_DCmotor");
        FrontRightDrive = hwMap.get(DcMotor.class, "FR_DCmotor");
        RearLeftDrive = hwMap.get(DcMotor.class, "RL_DCmotor");
        RearRightDrive = hwMap.get(DcMotor.class, "RR_DCmotor");

        FrontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        FrontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        RearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        RearRightDrive.setDirection(DcMotor.Direction.FORWARD);

        FrontLeftDrive.setPower(0);
        FrontRightDrive.setPower(0);
        RearLeftDrive.setPower(0);
        RearRightDrive.setPower(0);

        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}