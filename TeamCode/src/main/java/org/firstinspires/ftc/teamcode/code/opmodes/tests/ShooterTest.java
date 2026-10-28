package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.code.parts.Pusher;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;



@Config
@TeleOp(name="Shooter Test", group="TeleOp")

public class ShooterTest extends OpMode {
    private BasicHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Shooter pollenShooter;
    //private Shooter nectarShooter;

    public static double shooterVelocity = 0;
    public static double TICKS_PER_SEC = (double) 28 /60;

    private Pusher pusher;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        //nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"));

        Servo servo = hardwareMap.get(Servo.class, "pusher");
        servo.setPosition(50.0 / 300.0); // -100°
        pusher = new Pusher(servo, -45.0, 0.0, 250.0);
    }

    @Override
    public void loop() {
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if (gamepad1.yWasPressed()) {
            shooterVelocity = 3000 * TICKS_PER_SEC;
        } else if (gamepad1.xWasPressed()) {
            shooterVelocity = 0;
        } else if (gamepad1.leftBumperWasPressed()) {
            shooterVelocity -= 250 * TICKS_PER_SEC;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterVelocity += 250 * TICKS_PER_SEC;
        }
        pusher.update(gamepad1.b);

        pollenShooter.setShooterVelocity(shooterVelocity);

        telemetry.addData("target rpm: ", shooterVelocity * 60 / 28);
        telemetry.addData("actual rpm: ", pollenShooter.getShooterVelocity() * 60 / 28);
        telemetry.update();
    }

}
