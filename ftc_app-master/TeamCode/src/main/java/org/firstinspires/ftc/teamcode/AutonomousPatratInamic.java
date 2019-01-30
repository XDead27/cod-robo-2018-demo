package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Patrat_Inamic", group = "Autonomous")

public final class AutonomousPatratInamic extends AutonomousMode {

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        lasat_mascota();
    }

}
