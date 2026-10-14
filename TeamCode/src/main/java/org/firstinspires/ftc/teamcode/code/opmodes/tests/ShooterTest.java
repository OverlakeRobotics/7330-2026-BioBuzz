package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;
import org.firstinspires.ftc.teamcode.system.BasicHolonomicDrivetrain;


@Config
@TeleOp(name="Shooter Test", group="TeleOp")

public class ShooterTest extends OpMode {
    private BasicHolonomicDrivetrain driveTrain;
    public static double driveVelocity = 2800;

    private Shooter pollenShooter;
    //private Shooter nectarShooter;

    public static double shooterVelocity = 0;
    public static double velocityStep = 400;


    @Override
    public void init() {
        driveTrain = new BasicHolonomicDrivetrain(
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight")
        );

        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        //nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"));
    }

    @Override
    public void loop() {
        driveTrain.setVelocityDrive(
                -gamepad1.left_stick_y * driveVelocity,
                -gamepad1.left_stick_x * driveVelocity,
                -gamepad1.right_stick_x * driveVelocity);
        driveTrain.drive();

        if (gamepad1.yWasPressed()) {
            shooterVelocity += velocityStep;
        } else if (gamepad1.aWasPressed()) {
            shooterVelocity -= velocityStep;
        } else if (gamepad1.xWasPressed()) {
            shooterVelocity = 0;
        }

        pollenShooter.setShooterVelocity(shooterVelocity);

        telemetry.addData("Target", shooterVelocity);
        telemetry.addData("Actual", pollenShooter.getShooterVelocity());
        telemetry.update();
    }

}
