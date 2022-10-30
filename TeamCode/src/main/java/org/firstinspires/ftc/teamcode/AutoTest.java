package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoTest extends LinearOpMode {

    HardwareBIGBRAINBOTS robot = new HardwareBIGBRAINBOTS();

    @Override
    public void runOpMode() {
        robot.init(this.hardwareMap);

        waitForStart();
        robot.drive(1.0, 100);
    }
}