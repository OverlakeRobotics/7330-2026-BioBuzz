package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.code.parts.Shooter;

@Config
@TeleOp(name="Shooter Test", group="TeleOp")

public class ShooterTest extends OpMode {
    Shooter pollenShooter;
    Shooter nectarShooter;

    public static double shooterVelocity = 0;
    public static double velocityStep = 400;


    @Override
    public void init() {
        pollenShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "pollenShooter"));
        //nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"));
    }

    @Override
    public void loop() {
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
