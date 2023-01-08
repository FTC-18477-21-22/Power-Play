package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
        final int MAX_SLIDE = -4300;
        final int MIN_SLIDE = 0;
        while (opModeIsActive()) {

            double drive = -gamepad1.right_stick_y;
            double strafe = -gamepad1.right_stick_x;
            double turn = -gamepad1.left_stick_x*0.8;
            double slide = gamepad2.left_trigger-gamepad2.right_trigger;

            double FLPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
            double FRPower = Range.clip(drive + strafe - turn, -1.0, 1.0);
            double BLPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
            double BRPower = Range.clip(drive - strafe - turn, -1.0, 1.0);

            robot.FrontLeftDrive.setPower(FLPower);
            robot.FrontRightDrive.setPower(FRPower);
            robot.RearLeftDrive.setPower(BLPower);
            robot.RearRightDrive.setPower(BRPower);
            //robot.LeftSlide.setPower(slide);
            //robot.RightSlide.setPower(slide);

             telemetry.addData("Rotation", robot.LeftSlide.getCurrentPosition());

            if(gamepad2.a) {
                robot.Intake.setPower(1);
            } else if(gamepad2.b) {
                robot.Intake.setPower(-1);
            } else {
                robot.Intake.setPower(0);
            }

            if(gamepad2.left_bumper) {
                robot.Arm.setPosition(0);
            }

            if(gamepad2.right_bumper) {
                robot.Arm.setPosition(1);
            }

            if(robot.LeftSlide.getCurrentPosition()>=MIN_SLIDE) {
                telemetry.addData("Slide", "Min");
                if(slide<=0){
                    robot.LeftSlide.setPower(slide);
                    robot.RightSlide.setPower(slide);
                } else {
                    robot.LeftSlide.setPower(0);
                    robot.RightSlide.setPower(0);
                }
            } else if(robot.LeftSlide.getCurrentPosition()<=MAX_SLIDE) {
                telemetry.addData("Slide", "Max");
                if(slide>=0){
                    robot.LeftSlide.setPower(slide);
                    robot.RightSlide.setPower(slide);
                } else {
                    robot.LeftSlide.setPower(0);
                    robot.RightSlide.setPower(0);
                }
            } else {
                telemetry.addData("Slide", "Normal");
                robot.LeftSlide.setPower(slide);
                robot.RightSlide.setPower(slide);
            }
            telemetry.update();
        }
    }
}
