package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitchIntake {
    private DigitalChannel limitMagnetIntake;
    public void init(HardwareMap hardwareMap){
        limitMagnetIntake = hardwareMap.get(DigitalChannel.class, "MLSI");
        limitMagnetIntake.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getLimitSwitchIntake() {
        return !limitMagnetIntake.getState();
    }
}
