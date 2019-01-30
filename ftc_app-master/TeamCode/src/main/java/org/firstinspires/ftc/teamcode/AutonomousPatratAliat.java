package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous_Patrat_Aliat", group = "Autonomous")

public final class AutonomousPatratAliat extends AutonomousMode {

    @Override
    public void runOpMode() {
        initialise();

        waitForStart();

        gasit_cub(2);

    }

}
