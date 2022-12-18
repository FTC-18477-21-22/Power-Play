package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class BBBTeleOp extends LinearOpMode {
    HardwareBIGBRAINBOTS robot  = new HardwareBIGBRAINBOTS();
    @Override
    public void runOpMode() {
        robot.init(this.hardwareMap);

        telemetry.addData("Mode", "waiting");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            double drive = -gamepad1.right_stick_y;
            double strafe = -gamepad1.right_stick_x;
            double turn = -gamepad1.left_stick_x*0.8;
            double slide = gamepad1.left_trigger-gamepad1.right_trigger;

            double FLPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
            double FRPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
            double BLPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
            double BRPower = Range.clip(drive - strafe - turn, -1.0, 1.0);

            robot.FrontLeftDrive.setPower(FLPower);
            robot.FrontRightDrive.setPower(FRPower);
            robot.RearLeftDrive.setPower(BLPower);
            robot.RearRightDrive.setPower(BRPower);
            robot.LeftSlide.setPower(slide);
            robot.RightSlide.setPower(slide);
        }
    }
}
