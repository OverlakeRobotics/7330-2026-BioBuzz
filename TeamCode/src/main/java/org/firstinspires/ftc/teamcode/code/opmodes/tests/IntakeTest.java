package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.code.parts.Intake;

@Config
@TeleOp(name="Intake Test", group="TeleOp")

public class IntakeTest extends OpMode {
    Intake intake;

    public static double intakeVelocity = 0;


    @Override
    public void init() {
        intake = new Intake(hardwareMap.get(DcMotorEx.class, "intake"));
        //nectarShooter = new Shooter(hardwareMap.get(DcMotorEx.class, "nectarShooter"));
    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            intakeVelocity = 300;
        } else if (gamepad1.aWasPressed()) {
            intakeVelocity = -300;
        } else if (gamepad1.xWasPressed()) {
            intakeVelocity = 0;
        }
        intake.setIntakeVelocity(intakeVelocity);

    }

}
