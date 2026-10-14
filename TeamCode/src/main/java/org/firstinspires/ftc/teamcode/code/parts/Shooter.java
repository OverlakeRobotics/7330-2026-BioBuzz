package org.firstinspires.ftc.teamcode.code.parts;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class Shooter {
    public final double MAX_VELOCITY = 2000;

    private final DcMotorEx shooterMotor;

    public Shooter (DcMotorEx shooterMotor) {
        this.shooterMotor = shooterMotor;

        this.shooterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.shooterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void setShooterVelocity(double velocity) {
        double clamped = Range.clip(velocity, 0, MAX_VELOCITY);
        this.shooterMotor.setVelocity(clamped);
    }

    public double getShooterVelocity() {
        return shooterMotor.getVelocity();
    }

    public double getShooterPower() {
        return shooterMotor.getPower();
    }
}
