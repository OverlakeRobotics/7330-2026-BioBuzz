package org.firstinspires.ftc.teamcode.code.opmodes.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.code.helpers.BasePathPlannerAuto;

@Autonomous(name = "Back Corner", group = "Autonomous")
public class BackCornerAuto extends BasePathPlannerAuto {
    @Override
    protected String routeFile() {
        return "pathJsons/backcornernoshoot.json";
    }
}
