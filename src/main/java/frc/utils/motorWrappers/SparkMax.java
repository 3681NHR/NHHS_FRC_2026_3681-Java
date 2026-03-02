package frc.utils.motorWrappers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.Constants;

public class SparkMax extends com.revrobotics.spark.SparkMax {
  private static String motorsWithIncorrectFirmwareVersion = "";
  private static final Alert motorsWithIncorrectFirmwareVersionAlert = new Alert("Firmware version mismatch on SparkMaxs: ", AlertType.kWarning);
  /**
   * Create a new object to control a SPARK MAX motor Controller
   *
   * @param deviceId The device ID.
   * @param type The motor type connected to the controller. Brushless motor wires must be connected
   *     to their matching colors and the hall sensor must be plugged in. Brushed motors must be
   *     connected to the Red and Black terminals only.
   */
  public SparkMax(int deviceId, MotorType type) {
    super(deviceId, type);
    Logger.recordOutput(getDeviceId() + " firmware version (spark)", this.getFirmwareVersion());
    if (!DriverStation.isFMSAttached() && this.getFirmwareVersion() != Constants.SPARKMAX_TARGET_FIRMWARE) {
      motorsWithIncorrectFirmwareVersion += (motorsWithIncorrectFirmwareVersion.isBlank() ? "" : ", ") + getDeviceId();
      motorsWithIncorrectFirmwareVersionAlert.setText("Firmware version mismatch on SparkMaxs: " + motorsWithIncorrectFirmwareVersion);
      if (!motorsWithIncorrectFirmwareVersionAlert.get()) {
        motorsWithIncorrectFirmwareVersionAlert.set(true);
      }
    }
  }
  public static Alert getFirmwareAlert() {
    return motorsWithIncorrectFirmwareVersionAlert;
  }
}
