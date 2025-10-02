package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class ShooterCode extends LinearOpMode {
    public static double motorPower1 = 0.5;
    public static double motorPower2 = 0;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor shooterMotor1 = hardwareMap.dcMotor.get("motor1");
        DcMotor shooterMotor2 = hardwareMap.dcMotor.get("motor2");

        shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        double CPR = 28.0; // Counts per revolution (check your motor spec)
        ElapsedTime timer = new ElapsedTime();

        int lastPosition1 = shooterMotor1.getCurrentPosition();
        int lastPosition2 = shooterMotor2.getCurrentPosition();
        double lastTime = timer.seconds();

        while (opModeIsActive()) {
            double currentTime = timer.seconds();

            int currentPosition1 = shooterMotor1.getCurrentPosition();
            int currentPosition2 = shooterMotor2.getCurrentPosition();

            double deltaTime = currentTime - lastTime;

            double deltaPos1 = currentPosition1 - lastPosition1;
            double deltaPos2 = currentPosition2 - lastPosition2;

            double revolutions1 = deltaPos1 / CPR;
            double revolutions2 = deltaPos2 / CPR;

            double rps1 = revolutions1 / deltaTime;
            double rps2 = revolutions2 / deltaTime;

            double rpm1 = rps1 * 60;
            double rpm2 = rps2 * 60;

            // Update last time and position for next loop
            lastTime = currentTime;
            lastPosition1 = currentPosition1;
            lastPosition2 = currentPosition2;

            // Show the RPM of each motor
            telemetry.addData("Motor 1 RPM", rpm1);
            telemetry.addData("Motor 2 RPM", rpm2);
            telemetry.update();

            shooterMotor1.setPower(motorPower1);
            shooterMotor2.setPower(motorPower2);

            sleep(100); // update every 100ms
        }
    }
}
