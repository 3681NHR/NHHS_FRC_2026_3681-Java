package frc.utils.motorWrappers;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.Constants;

public class TalonFX extends com.ctre.phoenix6.hardware.TalonFX {

    private static String motorsWithIncorrectFirmwareVersion = "";
    private static final Alert motorsWithIncorrectFirmwareVersionAlert = new Alert(
            "Firmware version mismatch on Talons: ", AlertType.kWarning);

    /**
     * Constructs a new Talon FX motor controller object.
     * <p>
     * Constructs the device using the default CAN bus for the system
     * (see {@link CANBus#CANBus()}).
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner
     */
    public TalonFX(int deviceId) {
        this(deviceId, new CANBus());
    }

    /**
     * Constructs a new Talon FX motor controller object.
     *
     * @param deviceId ID of the device, as configured in Phoenix Tuner
     * @param canbus   Name of the CAN bus this device is on. Possible CAN bus
     *                 strings are:
     *                 <ul>
     *                 <li>"rio" for the native roboRIO CAN bus
     *                 <li>CANivore name or serial number
     *                 <li>SocketCAN interface (non-FRC Linux only)
     *                 <li>"*" for any CANivore seen by the program
     *                 <li>empty string (default) to select the default for the
     *                 system:
     *                 <ul>
     *                 <li>"rio" on roboRIO
     *                 <li>"can0" on Linux
     *                 <li>"*" on Windows
     *                 </ul>
     *                 </ul>
     *
     * @deprecated Constructing devices with a CAN bus string is deprecated for
     *             removal
     *             in the 2027 season. Construct devices using a {@link CANBus}
     *             instance instead.
     */
    @Deprecated(since = "2026", forRemoval = true)
    public TalonFX(int deviceId, String canbus) {
        this(deviceId, new CANBus(canbus));
    }

    public TalonFX(int deviceId, CANBus canbus) {
        super(deviceId, canbus);
        Logger.recordOutput(getDeviceID() + " firmware version (TalonFX)", getVersion().getValue());
        if (!DriverStation.isFMSAttached() && getVersion().getValue() != Constants.TALONFX_TARGET_FIRMWARE) {
            motorsWithIncorrectFirmwareVersion += (motorsWithIncorrectFirmwareVersion.isBlank() ? "" : ", ")
                    + getDeviceID();
            motorsWithIncorrectFirmwareVersionAlert
                    .setText("Firmware version mismatch on Talons: " + motorsWithIncorrectFirmwareVersion);
            if (!motorsWithIncorrectFirmwareVersionAlert.get()) {
                motorsWithIncorrectFirmwareVersionAlert.set(true);
            }
        }
    }

    public static Alert getFirmwareAlert() {
        return motorsWithIncorrectFirmwareVersionAlert;
    }
}
