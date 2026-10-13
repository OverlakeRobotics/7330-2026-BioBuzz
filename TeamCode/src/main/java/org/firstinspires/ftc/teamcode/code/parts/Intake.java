package org.firstinspires.ftc.teamcode.code.parts;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private final DcMotorEx intakeMotor;

    public Intake(DcMotorEx intakeMotor) {
        this.intakeMotor = intakeMotor;

        this.intakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setIntakeVelocity(double velocity) {
        intakeMotor.setVelocity(velocity);
    }
}