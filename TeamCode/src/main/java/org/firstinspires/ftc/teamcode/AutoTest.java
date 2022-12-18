package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class AutoTest extends LinearOpMode {

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
    //imu stuff end

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(this.hardwareMap);

        waitForStart();
        double distance = 100;
        int counts = (int)(COUNTS_PER_INCH*distance);
        robot.drive(0.50, counts);

        robot.gyroTurn(TURN_SPEED, 180, P_TURN_COEFF_1);

        robot.drive(0.50, (int)(45.2079566*100));
        /*sleep(5000);
        robot.gyroTurn(TURN_SPEED, 0, P_TURN_COEFF_2);
        telemetry.addData("turn", "finished");
        telemetry.update();*/

    }


}

