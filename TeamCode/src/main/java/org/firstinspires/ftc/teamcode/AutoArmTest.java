package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoArmTest extends LinearOpMode {

    static final double COUNTS_PER_INCH = 45.2079566;
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();

    private BNO055IMU imu;
    static final double TURN_SPEED = 0.75;
    static final double P_TURN_COEFF_1 = 0.025;
    static final double P_TURN_COEFF_2 = 0.0035;
    static final double HEADING_THRESHOLD = 0.5;
    String signal = "NO_VALUE";

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);
        //set arm position to 0 during Initialize stage
        robot.Arm.setPosition(0.6);

        //waits until run is pressed
        waitForStart();
        sleep(50);

        //test if servo can be set to negative
        robot.Arm.setPosition(0.2);
        while(true) {
            // nothing
        }
    }
}
