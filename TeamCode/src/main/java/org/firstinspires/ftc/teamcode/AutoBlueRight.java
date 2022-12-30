package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoBlueRight extends LinearOpMode {

    //static final double COUNTS_PER_MOTOR_REV = 28 ;
    //static final double DRIVE_GEAR_REDUCTION = 13.7;
    //static final double WHEEL_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = 45.2079566;
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();

    private BNO055IMU imu;
    static final double TURN_SPEED = 0.75;
    static final double P_TURN_COEFF_1 = 0.025;
    static final double P_TURN_COEFF_2 = 0.0035;
    static final double HEADING_THRESHOLD = 0.5;
    String signal = "NO_VALUE";
    //imu stuff end

    @Override
    public void runOpMode() throws InterruptedException {
        signal = "two";

        robot.init(this.hardwareMap);
        robot.Intake.setPower(1);
        robot.Arm.setPosition(0.3);
        waitForStart();
        double distance = 100;
        int counts = (int)(COUNTS_PER_INCH*distance);
        robot.LeftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.RightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.drive(0.35, (int)(COUNTS_PER_INCH*1));
        robot.strafe(0.35, (int)(COUNTS_PER_INCH*22));
        robot.notallwait(0.7, (int)-3700);
        robot.drive(0.35, (int)(COUNTS_PER_INCH*27));
        robot.strafe(0.35, (int)(COUNTS_PER_INCH*15));
        // robot.notallwait(0.7, (int)-3700);
        robot.drive(0.35, (int)(COUNTS_PER_INCH*7));
        robot.Arm.setPosition(0.6);
        robot.Intake.setPower(-1);
        sleep(500);
        robot.Intake.setPower(0);
        robot.drive(0.35, (int)-(COUNTS_PER_INCH*6));
        robot.notallwait(0.7, (int)0);
        // End of robot drop
        switch (signal) {
            case "one":
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*-20));
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*2));
                break;
            case "two":
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*-45));
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*2));
                break;
            case "three":
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*-74));
                robot.strafe(0.35, (int)(COUNTS_PER_INCH*2));
                break;
        }
        while(true) {
            //empty
        }

        // robot.gyroTurn(TURN_SPEED, 180, P_TURN_COEFF_1);

        // robot.drive(0.50, (int)(45.2079566*100));
        /*sleep(5000);
        robot.gyroTurn(TURN_SPEED, 0, P_TURN_COEFF_2);
        telemetry.addData("turn", "finished");
        telemetry.update();*/

    }


}

