package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

@Autonomous
public class AutoTest extends LinearOpMode {

    //static final double COUNTS_PER_MOTOR_REV = 28 ;
    //static final double DRIVE_GEAR_REDUCTION = 13.7;
    //static final double WHEEL_DIAMETER_INCHES = 4;
    static final double COUNTS_PER_INCH = 45.2079566;
    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();


    @Override
    public void runOpMode() {
        robot.init(this.hardwareMap);
        waitForStart();
        double distance = 10;
        int counts = (int)(COUNTS_PER_INCH*distance);
        robot.drive(0.50, counts);
    }


}

