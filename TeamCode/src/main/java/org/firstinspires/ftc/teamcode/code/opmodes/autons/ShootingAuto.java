package org.firstinspires.ftc.teamcode.code.opmodes.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BasePathPlannerAuto;

@Autonomous(name = "Shooting Path", group = "Autonomous")
public class ShootingAuto extends BasePathPlannerAuto {
    @Override
    protected String routeFile() {
        return "pathJsons/shooting.json";
    }
}
