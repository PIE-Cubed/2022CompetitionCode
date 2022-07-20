package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Not being used yet, but contains useful code
public class PowerDistributionPanel {
    // Object Creation
    PowerDistribution pdp;

    // Power channels, not the same as CAN ids
    private int FR_DRIVE = 6;
    private int FL_DRIVE = 19;
    private int BR_DRIVE = 15;
    private int BL_DRIVE = 17;

    private int FR_ROTATE = 7;
    private int FL_ROTATE = 18;
    private int BR_ROTATE = 14;
    private int BL_ROTATE = 16;

    public PowerDistributionPanel() {
        pdp = new PowerDistribution(2, ModuleType.kRev);
        pdp.setSwitchableChannel(true);
    }

    // Gets the total voltage from power distribution panel
    public double getVoltage() {
        return pdp.getVoltage();
    }

    // Gets the total current drawn by power distribution panel
    public double getCurrent() {
        return pdp.getTotalCurrent();
    }

    // Gets the total power being drawn, which is the product of voltage and current, in watts
    public double getPower() {
        return pdp.getTotalPower();
    }

    // Power * time, in joules
    public double getEnergy() {
        return pdp.getTotalEnergy();
    }

    public double getSwerveCurrent() {
        double driveCurrent  = pdp.getCurrent(FR_DRIVE)  + pdp.getCurrent(FL_DRIVE)  + pdp.getCurrent(BR_DRIVE)  + pdp.getCurrent(BL_DRIVE); 
        double rotateCurrent = pdp.getCurrent(FR_ROTATE) + pdp.getCurrent(FL_ROTATE) + pdp.getCurrent(BR_ROTATE) + pdp.getCurrent(BL_ROTATE);
        return driveCurrent + rotateCurrent;
    }

    public double getMotorCurrent(int channel) {
        return pdp.getCurrent(channel);
    }

    /*
     * Test functions
     */
    public void testSmartDashboard() {
        SmartDashboard.putNumber("FR Drive Current", pdp.getCurrent(FR_DRIVE));
        SmartDashboard.putNumber("FL Drive Current", pdp.getCurrent(FL_DRIVE));
        SmartDashboard.putNumber("BR Drive Current", pdp.getCurrent(BR_DRIVE));
        SmartDashboard.putNumber("BL Drive Current", pdp.getCurrent(BL_DRIVE));
    }
}