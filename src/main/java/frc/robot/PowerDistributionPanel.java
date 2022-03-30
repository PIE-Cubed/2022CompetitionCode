package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PowerDistributionPanel {
    PowerDistribution powerDistributionPanel;


    //Power channels, not the same as CAN ids
    private int FR_DRIVE = 6;
    private int FL_DRIVE = 19;
    private int BR_DRIVE = 15;
    private int BL_DRIVE = 17;

    private int FR_ROTATE = 7;
    private int FL_ROTATE = 18;
    private int BR_ROTATE = 14;
    private int BL_ROTATE = 16;

    public PowerDistributionPanel() {
        powerDistributionPanel = new PowerDistribution(2, ModuleType.kRev);
    }

    public double getVoltage() {
        return powerDistributionPanel.getVoltage();
    }

    public double getCurrent() {
        return powerDistributionPanel.getTotalCurrent();
    }

    //Returns power (being drawn I assume), which is the product of voltage and current, in watts
    public double getPower() {
        return powerDistributionPanel.getTotalPower();
    }

    //Power * time, in joules
    public double getEnergy() {
        return powerDistributionPanel.getTotalEnergy();
    }

    public double getSwerveCurrent() {
        double driveCurrent  = powerDistributionPanel.getCurrent(FR_DRIVE)  + powerDistributionPanel.getCurrent(FL_DRIVE)  + powerDistributionPanel.getCurrent(BR_DRIVE)  + powerDistributionPanel.getCurrent(BL_DRIVE); 
        double rotateCurrent = powerDistributionPanel.getCurrent(FR_ROTATE) + powerDistributionPanel.getCurrent(FL_ROTATE) + powerDistributionPanel.getCurrent(BR_ROTATE) + powerDistributionPanel.getCurrent(BL_ROTATE);
        return driveCurrent + rotateCurrent;
    }

    public double getMotorCurrent(int channel) {
        return powerDistributionPanel.getCurrent(channel);
    }

    public void testSmartDashboard() {
        SmartDashboard.putNumber("FR Drive Current", powerDistributionPanel.getCurrent(FR_DRIVE));
        SmartDashboard.putNumber("FL Drive Current", powerDistributionPanel.getCurrent(FL_DRIVE));
        SmartDashboard.putNumber("BR Drive Current", powerDistributionPanel.getCurrent(BR_DRIVE));
        SmartDashboard.putNumber("BL Drive Current", powerDistributionPanel.getCurrent(BL_DRIVE));
    }
}
