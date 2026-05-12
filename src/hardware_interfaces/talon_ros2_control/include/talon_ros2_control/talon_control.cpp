#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <chrono>
#include <cmath>
#include <string>

#define Phoenix_No_WPI // remove WPI dependencies
#include "ctre/Phoenix.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

#include "talon_ros2_control/talon_control.h"

// ── Motor initialization ─────────────────────────────────────────────────────

void initMotor(TalonSRX * motor, const MotorConfig & config, const std::string & can_interface) {
  // Wake up the CAN bus (send a dummy frame so the kernel driver is active)
  std::string cmd = "cansend " + can_interface + " 123#00000000";
  std::system(cmd.c_str());

  motor->ConfigFactoryDefault(config.config_timeout_ms);
  motor->SetInverted(config.inverted);

  motor->ConfigSelectedFeedbackSensor(
    ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder,
    0, config.config_timeout_ms);
  motor->SetSensorPhase(config.sensor_phase);

  motor->Config_kP(0, config.kP, config.config_timeout_ms);
  motor->Config_kI(0, config.kI, config.config_timeout_ms);
  motor->Config_kD(0, config.kD, config.config_timeout_ms);
  motor->Config_kF(0, config.kF, config.config_timeout_ms);
}

// ── Conversion helpers ───────────────────────────────────────────────────────

double convertRevtoTalonUnits(double rev, const MotorConfig & config) {
  double counts_per_revolution = config.encoder_resolution * config.gear_ratio;
  return rev * counts_per_revolution;
}

double convertTalonUnitstoRev(double counts, const MotorConfig & config) {
  double counts_per_revolution = config.encoder_resolution * config.gear_ratio;
  return counts / counts_per_revolution;
}

double convertRevToDistance(double rev, const MotorConfig & config) {
  return rev * config.distance_per_rev;
}

double convertDistanceToRev(double distance, const MotorConfig & config) {
  return distance / config.distance_per_rev;
}

// ── Position getters ─────────────────────────────────────────────────────────

float getPositionTalonUnits(TalonSRX * motor) {
  return motor->GetSelectedSensorPosition(0);
}

float getPositionRevolutions(TalonSRX * motor, const MotorConfig & config) {
  return static_cast<float>(
    convertTalonUnitstoRev(motor->GetSelectedSensorPosition(0), config));
}

float getPositionDistance(TalonSRX * motor, const MotorConfig & config) {
  // Output revolutions -> meters (for prismatic joints)
  return getPositionRevolutions(motor, config) * static_cast<float>(config.distance_per_rev);
}

float getPositionRadians(TalonSRX * motor, const MotorConfig & config) {
  // Output revolutions -> radians (for revolute joints)
  return getPositionRevolutions(motor, config) * static_cast<float>(2.0 * M_PI);
}

// ── Velocity getters ─────────────────────────────────────────────────────────

float getVelocityRPM(TalonSRX * motor, const MotorConfig & config) {
  // TalonSRX reports velocity in encoder units per 100 ms
  float units_per_minute = motor->GetSelectedSensorVelocity(0) * 600.0f;
  return static_cast<float>(convertTalonUnitstoRev(units_per_minute, config));
}

float getLinearVelocity(TalonSRX * motor, const MotorConfig & config) {
  // Talon units/100ms -> units/s -> rev/s -> m/s
  float units_per_second = motor->GetSelectedSensorVelocity(0) * 10.0f;
  float rev_per_second = static_cast<float>(convertTalonUnitstoRev(units_per_second, config));
  return rev_per_second * static_cast<float>(config.distance_per_rev);
}

float getAngularVelocity(TalonSRX * motor, const MotorConfig & config) {
  // Talon units/100ms -> units/s -> rev/s -> rad/s
  float units_per_second = motor->GetSelectedSensorVelocity(0) * 10.0f;
  float rev_per_second = static_cast<float>(convertTalonUnitstoRev(units_per_second, config));
  return rev_per_second * static_cast<float>(2.0 * M_PI);
}

// ── Motor command setters ────────────────────────────────────────────────────

void setDutyCycle(TalonSRX * motor, double dutyCycle, int ms) {
  motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, dutyCycle);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);
}

float setVelocityFromLinearVelocity(TalonSRX * motor, double vel_m_per_s, int ms, const MotorConfig & config) {
  // m/s -> m/100ms -> rev/100ms -> talon units/100ms
  double vel_m_per_100ms = vel_m_per_s / 10.0;
  double rev_per_100ms = convertDistanceToRev(vel_m_per_100ms, config);
  double talon_units_per_100ms = convertRevtoTalonUnits(rev_per_100ms, config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, talon_units_per_100ms);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units_per_100ms);
}

float setVelocityFromAngularVelocity(TalonSRX * motor, double vel_rad_per_s, int ms, const MotorConfig & config) {
  // rad/s -> rev/s -> rev/100ms -> talon units/100ms
  double rev_per_100ms = vel_rad_per_s / (2.0 * M_PI * 10.0);
  double talon_units_per_100ms = convertRevtoTalonUnits(rev_per_100ms, config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, talon_units_per_100ms);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units_per_100ms);
}

float setPositionFromDisplacement(TalonSRX * motor, double displacement_m, int ms, const MotorConfig & config) {
  // m -> rev -> talon units
  double talon_units = convertRevtoTalonUnits(convertDistanceToRev(displacement_m, config), config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, talon_units);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units);
}

float setPositionFromJointCommand(TalonSRX * motor, double position_rad, int ms, const MotorConfig & config) {
  // rad -> rev -> talon units
  double talon_units = convertRevtoTalonUnits(position_rad / (2.0 * M_PI), config);

  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, talon_units);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);

  return static_cast<float>(talon_units);
}

void stopMotor(TalonSRX * motor, int ms) {
  motor->Set(ctre::phoenix::motorcontrol::ControlMode::Disabled, 0.0);
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(ms);
}

// ── Helper functions ────────────────────────────────────────────────────

void statusStringGenerator(int32_t status, std::string & msg)
{
    switch (status)
    {
        case ctre::phoenix::OK: msg = "OK"; break;
        case ctre::phoenix::CAN_MSG_STALE: msg = "CAN Message Stale"; break;
        case ctre::phoenix::CAN_TX_FULL: msg = "CAN TX Full"; break;
        case ctre::phoenix::InvalidParamValue: msg = "Invalid Parameter Value"; break;
        case ctre::phoenix::RxTimeout: msg = "RX Timeout"; break;
        case ctre::phoenix::TxTimeout: msg = "TX Timeout"; break;
        case ctre::phoenix::UnexpectedArbId: msg = "Unexpected Arbitration ID"; break;
        case ctre::phoenix::BufferFull: msg = "Buffer Full"; break;
        case ctre::phoenix::SensorNotPresent: msg = "Sensor Not Present"; break;
        case ctre::phoenix::FirmwareTooOld: msg = "Firmware Too Old"; break;
        case ctre::phoenix::CouldNotChangePeriod: msg = "Could Not Change Period"; break;
        case ctre::phoenix::BufferFailure: msg = "Buffer Failure"; break;
        case ctre::phoenix::FirwmwareNonFRC: msg = "Firmware Non-FRC"; break;

        case ctre::phoenix::GeneralError: msg = "General Error"; break;

        case ctre::phoenix::SIG_NOT_UPDATED: msg = "Signal Not Updated"; break;
        case ctre::phoenix::NotAllPIDValuesUpdated: msg = "Not All PID Values Updated"; break;

        case ctre::phoenix::GEN_PORT_ERROR: msg = "General Port Error"; break;
        case ctre::phoenix::PORT_MODULE_TYPE_MISMATCH: msg = "Port Module Type Mismatch"; break;

        case ctre::phoenix::GEN_MODULE_ERROR: msg = "General Module Error"; break;
        case ctre::phoenix::MODULE_NOT_INIT_SET_ERROR: msg = "Module Not Initialized (Set Error)"; break;
        case ctre::phoenix::MODULE_NOT_INIT_GET_ERROR: msg = "Module Not Initialized (Get Error)"; break;

        case ctre::phoenix::WheelRadiusTooSmall: msg = "Wheel Radius Too Small"; break;
        case ctre::phoenix::TicksPerRevZero: msg = "Ticks Per Revolution is Zero"; break;
        case ctre::phoenix::DistanceBetweenWheelsTooSmall: msg = "Distance Between Wheels Too Small"; break;
        case ctre::phoenix::GainsAreNotSet: msg = "Gains Are Not Set"; break;
        case ctre::phoenix::WrongRemoteLimitSwitchSource: msg = "Wrong Remote Limit Switch Source"; break;
        case ctre::phoenix::DoubleVoltageCompensatingWPI: msg = "Double Voltage Compensating WPI"; break;
        case ctre::phoenix::CANdleAnimSlotOutOfBounds: msg = "CANdle Animation Slot Out Of Bounds"; break;

        case ctre::phoenix::IncompatibleMode: msg = "Incompatible Mode"; break;
        case ctre::phoenix::InvalidHandle: msg = "Invalid Handle"; break;

        case ctre::phoenix::FeatureRequiresHigherFirm: msg = "Feature Requires Higher Firmware"; break;
        case ctre::phoenix::MotorControllerFeatureRequiresHigherFirm: msg = "Motor Controller Feature Requires Higher Firmware"; break;
        case ctre::phoenix::ConfigFactoryDefaultRequiresHigherFirm: msg = "Factory Default Requires Higher Firmware"; break;
        case ctre::phoenix::ConfigMotionSCurveRequiresHigherFirm: msg = "Motion S Curve Requires Higher Firmware"; break;
        case ctre::phoenix::TalonFXFirmwarePreVBatDetect: msg = "TalonFX Firmware Pre VBat Detect"; break;
        case ctre::phoenix::CANdleAnimationsRequireHigherFirm: msg = "CANdle Animations Require Higher Firmware"; break;

        case ctre::phoenix::LibraryCouldNotBeLoaded: msg = "Library Could Not Be Loaded"; break;
        case ctre::phoenix::MissingRoutineInLibrary: msg = "Missing Routine In Library"; break;
        case ctre::phoenix::ResourceNotAvailable: msg = "Resource Not Available"; break;

        case ctre::phoenix::MusicFileNotFound: msg = "Music File Not Found"; break;
        case ctre::phoenix::MusicFileWrongSize: msg = "Music File Wrong Size"; break;
        case ctre::phoenix::MusicFileTooNew: msg = "Music File Too New"; break;
        case ctre::phoenix::MusicFileInvalid: msg = "Music File Invalid"; break;
        case ctre::phoenix::InvalidOrchestraAction: msg = "Invalid Orchestra Action"; break;
        case ctre::phoenix::MusicFileTooOld: msg = "Music File Too Old"; break;
        case ctre::phoenix::MusicInterrupted: msg = "Music Interrupted"; break;
        case ctre::phoenix::MusicNotSupported: msg = "Music Not Supported"; break;

        case ctre::phoenix::kInvalidInterface: msg = "Invalid Interface"; break;
        case ctre::phoenix::kInvalidGuid: msg = "Invalid GUID"; break;
        case ctre::phoenix::kInvalidClass: msg = "Invalid Class"; break;
        case ctre::phoenix::kInvalidProtocol: msg = "Invalid Protocol"; break;
        case ctre::phoenix::kInvalidPath: msg = "Invalid Path"; break;
        case ctre::phoenix::kGeneralWinUsbError: msg = "General WinUSB Error"; break;
        case ctre::phoenix::kFailedSetup: msg = "Failed Setup"; break;
        case ctre::phoenix::kListenFailed: msg = "Listen Failed"; break;
        case ctre::phoenix::kSendFailed: msg = "Send Failed"; break;
        case ctre::phoenix::kReceiveFailed: msg = "Receive Failed"; break;
        case ctre::phoenix::kInvalidRespFormat: msg = "Invalid Response Format"; break;
        case ctre::phoenix::kWinUsbInitFailed: msg = "WinUSB Init Failed"; break;
        case ctre::phoenix::kWinUsbQueryFailed: msg = "WinUSB Query Failed"; break;
        case ctre::phoenix::kWinUsbGeneralError: msg = "WinUSB General Error"; break;
        case ctre::phoenix::kAccessDenied: msg = "Access Denied"; break;
        case ctre::phoenix::kFirmwareInvalidResponse: msg = "Firmware Invalid Response"; break;

        case ctre::phoenix::PulseWidthSensorNotPresent: msg = "Pulse Width Sensor Not Present"; break;

        case ctre::phoenix::GeneralWarning: msg = "General Warning"; break;
        case ctre::phoenix::FeatureNotSupported: msg = "Feature Not Supported"; break;
        case ctre::phoenix::NotImplemented: msg = "Not Implemented"; break;
        case ctre::phoenix::FirmVersionCouldNotBeRetrieved: msg = "Firmware Version Could Not Be Retrieved"; break;
        case ctre::phoenix::FeaturesNotAvailableYet: msg = "Features Not Available Yet"; break;
        case ctre::phoenix::ControlModeNotValid: msg = "Control Mode Not Valid"; break;
        case ctre::phoenix::ControlModeNotSupportedYet: msg = "Control Mode Not Supported Yet"; break;
        case ctre::phoenix::CascadedPIDNotSupporteYet: msg = "Cascaded PID Not Supported Yet"; break;
        case ctre::phoenix::RemoteSensorsNotSupportedYet: msg = "Remote Sensors Not Supported Yet"; break;
        case ctre::phoenix::MotProfFirmThreshold: msg = "Motion Profile Firmware Threshold"; break;
        case ctre::phoenix::MotProfFirmThreshold2: msg = "Motion Profile Firmware Threshold 2"; break;

        case ctre::phoenix::SimDeviceNotFound: msg = "Simulation Device Not Found"; break;
        case ctre::phoenix::SimPhysicsTypeNotSupported: msg = "Simulation Physics Type Not Supported"; break;
        case ctre::phoenix::SimDeviceAlreadyExists: msg = "Simulation Device Already Exists"; break;

        default:
            msg = "Unknown Error Code: " + std::to_string(status);
            break;
    }
}