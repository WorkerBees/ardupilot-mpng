// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Turn some things off
#define CLI_ENABLED ENABLED
#define MOUNT2 DISABLED
#define CAMERA DISABLED


// Units
// -----
//
// Unless indicated otherwise, numeric quantities use the following units:
//
// Measurement | Unit
// ------------+-------------------------------------
// angle       | degrees
// distance    | metres
// speed       | metres per second
// servo angle | microseconds
// voltage     | volts
// times       | seconds
// throttle    | percent
//

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// HARDWARE CONFIGURATION AND CONNECTIONS
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// GPS_PROTOCOL                             REQUIRED
//
// GPS configuration, must be one of:
//
// GPS_PROTOCOL_AUTO        Auto detect GPS type (must be a supported GPS)
// GPS_PROTOCOL_NONE        No GPS attached
// GPS_PROTOCOL_IMU         X-Plane interface or ArduPilot IMU.
// GPS_PROTOCOL_MTK         MediaTek-based GPS running the DIYDrones firmware 1.4
// GPS_PROTOCOL_MTK19       MediaTek-based GPS running the DIYDrones firmware 1.6, 1.7, 1.8, 1.9
// GPS_PROTOCOL_UBLOX       UBLOX GPS
// GPS_PROTOCOL_SIRF        SiRF-based GPS in Binary mode.  NOT TESTED
// GPS_PROTOCOL_NMEA        Standard NMEA GPS.      NOT SUPPORTED (yet?)
//
// AUTO mode will force UBlox to 38400 no matter how it's configured, apparently
#define GPS_PROTOCOL  GPS_PROTOCOL_AUTO

//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_SENSOR                          OPTIONAL
// AIRSPEED_RATIO                           OPTIONAL
//
// Set AIRSPEED_SENSOR to ENABLED if you have an airspeed sensor attached.
// Adjust AIRSPEED_RATIO in small increments to calibrate the airspeed
// sensor relative to your GPS.  The calculation and default value are optimized for speeds around 12 m/s
//
// The default assumes that an airspeed sensor is connected.
//
#define AIRSPEED_SENSOR     ENABLED
//#define AIRSPEED_RATIO      1.9936
//

//////////////////////////////////////////////////////////////////////////////
// MAGNETOMETER                          OPTIONAL
//
// Set MAGNETOMETER to ENABLED if you have a magnetometer attached.
//
// The default assumes that a magnetometer is not connected.
//
#define MAGNETOMETER        ENABLED

//////////////////////////////////////////////////////////////////////////////
// GCS_PROTOCOL                             OPTIONAL
// GCS_PORT                                 OPTIONAL
// MAV_SYSTEM_ID                            OPTIONAL
//
// The GCS_PROTOCOL option determines which (if any) ground control station
// protocol will be used.  Must be one of:
//
// GCS_PROTOCOL_NONE        No GCS output
// GCS_PROTOCOL_MAVLINK     QGroundControl protocol
//
// The GCS_PORT option determines which serial port will be used by the
// GCS protocol.   The usual values are 0 for the console/USB port,
// or 3 for the telemetry port on the oilpan.  Note that some protocols
// will ignore this value and always use the console port.
//
// The MAV_SYSTEM_ID is a unique identifier for this UAV.  The default value is 1.
// If you will be flying multiple UAV's each should be assigned a different ID so
// that ground stations can tell them apart.
//
#define GCS_PROTOCOL        GCS_PROTOCOL_MAVLINK
#define GCS_PORT            0   // Use main console port for now
#define MAV_SYSTEM_ID       1

//////////////////////////////////////////////////////////////////////////////
// Serial port speeds.
//
// SERIAL0_BAUD                             OPTIONAL
//
// Baudrate for the console port.  Default is 115200bps.
//
// SERIAL3_BAUD                             OPTIONAL
//
// Baudrate for the telemetry port.  Default is 57600bps.
//

// Console
#define SERIAL0_BAUD        115200

// GPS
#define SERIAL2_BAUD         38400


//////////////////////////////////////////////////////////////////////////////
// Battery monitoring                       OPTIONAL
//
// See the manual for details on selecting divider resistors for battery
// monitoring via the oilpan.
//
// BATTERY_EVENT                            OPTIONAL
//
// Values: 0:Disabled,3:Voltage Only,4:Voltage and Current
//
// LOW_VOLTAGE                              OPTIONAL if BATTERY_EVENT is set.
//
// Value in volts at which ArduPilot Mega should consider the
// battery to be "low".
//
// VOLT_DIV_RATIO                           OPTIONAL
//
// See the manual for details.  The default value corresponds to the resistors
// recommended by the manual.
//
// CURR_AMPS_PER_VOLT                       OPTIONAL
// CURR_AMPS_OFFSET                         OPTIONAL
//
// The sensitivity of the current sensor.  This must be scaled if a resistor is installed on APM
// for a voltage divider on input 2 (not recommended).  The offset is used for current sensors with an offset
//
//
// HIGH_DISCHARGE                           OPTIONAL if BATTERY_EVENT is set.
//
// Value in milliamp-hours at which a warning should be triggered.  Recommended value = 80% of
// battery capacity.
//

// TODO: Battery level warning should be calculated dynmically and based on distance from HOME
// If you only have enough battery left to RTL, then fricking RTL don't crash in the styx
// TODO: These are not actually built into the code as defaults; the defaults are hardcoded in /library/AP_BattMonitor/AP_BattMonitor.cpp
#define BATTERY_EVENT         ENABLED
#define VOLTAGE_PIN           1
#define LOW_VOLTAGE           10.5
#define VOLT_DIV_RATIO        3.127659 // ((100+47)/47)
//#define CURR_PIN              2
//#define CURR_AMPS_PER_VOLT           27.32
//#define CURR_AMPS_OFFSET      0.0
#define HIGH_DISCHARGE        2400  // (3000 * 80%)

//////////////////////////////////////////////////////////////////////////////
// INPUT_VOLTAGE                            OPTIONAL
//
// In order to have accurate pressure and battery voltage readings, this
// value should be set to the voltage measured at the processor.
//
// See the manual for more details.  The default value should be close if you are applying 5 volts to the servo rail.
//
//#define INPUT_VOLTAGE 4.68    //  4.68 is the average value for a sample set.  This is the value at the processor with 5.02 applied at the servo rail
//


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// RADIO CONFIGURATION
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// FLIGHT_MODE                              OPTIONAL
// FLIGHT_MODE_CHANNEL                      OPTIONAL
//
// Flight modes assigned to the control channel, and the input channel that
// is read for the control mode.
//
// Use a servo tester, or the ArduPilotMega_demo test program to check your
// switch settings.
//
// ATTENTION: Some ArduPilot Mega boards have radio channels marked 0-7, and
// others have them marked the standard 1-8.  The FLIGHT_MODE_CHANNEL option
// uses channel numbers 1-8 (and defaults to 8).
//
// If you only have a three-position switch or just want three modes, set your
// switch to produce 1165, 1425, and 1815 microseconds and configure
// FLIGHT_MODE 1 & 2, 3 & 4 and 5 & 6 to be the same.  This is the default.
//
// If you have FLIGHT_MODE_CHANNEL set to 8 (the default) and your control
// channel connected to input channel 8, the hardware failsafe mode will
// activate for any control input over 1750ms.
//
// For more modes (up to six), set your switch(es) to produce any of 1165,
// 1295, 1425, 1555, 1685, and 1815 microseconds.
//
// Flight mode |  Switch Setting (ms)
// ------------+---------------------
//      1      |     1165
//      2      |     1295
//      3      |     1425
//      4      |     1555
//      5      |     1685
//      6      |     1815   (FAILSAFE if using channel 8)
//
// The following standard flight modes are available:
//
//  Name            | Description
// -----------------+--------------------------------------------
//                  |
//  MANUAL          | Full manual control via the hardware multiplexer.
//                  |
//  STABILIZE       | Tries to maintain level flight, but can be overridden with radio control inputs.
//                  |
//  FLY_BY_WIRE_A   | Autopilot style control via user input, with manual throttle.
//                  |
//  FLY_BY_WIRE_B   | Autopilot style control via user input, aispeed controlled with throttle.
//                  |
//  RTL             | Returns to the Home location and then LOITERs at a safe altitude.
//                  |
//  AUTO            | Autonomous flight based on programmed waypoints.  Use the WaypointWriter
//                  | application or your Ground Control System to edit and upload
//                  | waypoints and other commands.
//                  |
//air
//
// The following non-standard modes are EXPERIMENTAL:
//
//  Name            | Description
// -----------------+--------------------------------------------
//                  |
//  LOITER          | Flies in a circle around the current location.
//                  |
//  CIRCLE          | Flies in a stabilized 'dumb' circle.
//                  |
//
//
// If you are using channel 8 for mode switching then FLIGHT_MODE_5 and
// FLIGHT_MODE_6 should be MANUAL.
//
//
#define FLIGHT_MODE_CHANNEL 5
//
#define FLIGHT_MODE_1         AUTO
#define FLIGHT_MODE_2         AUTO
#define FLIGHT_MODE_3         AUTO
#define FLIGHT_MODE_4         MANUAL
#define FLIGHT_MODE_5         MANUAL
#define FLIGHT_MODE_6         MANUAL


//////////////////////////////////////////////////////////////////////////////
// AUTO_TRIM                                OPTIONAL
//
// ArduPilot Mega can update its trim settings by looking at the
// radio inputs when switching out of MANUAL mode.  This allows you to
// manually trim your aircraft before switching to an assisted mode, but it
// also means that you should avoid switching out of MANUAL while you have
// any control stick deflection.
//
// The default is to disable AUTO_TRIM.
//
#define AUTO_TRIM           DISABLED

//////////////////////////////////////////////////////////////////////////////
// ENABLE_STICK_MIXING                     OPTIONAL
//
// If this option is set to ENABLED, manual control inputs are are respected
// while in the autopilot modes (AUTO, RTL, LOITER, CIRCLE etc.)
//
// The default is to enable stick mixing, allowing the pilot to take
// emergency action without switching modes.
//
#define ENABLE_STICK_MIXING ENABLED


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// STARTUP BEHAVIOUR
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// GROUND_START_DELAY                       OPTIONAL
//
// If configured, inserts a delay between power-up and the beginning of INS
// calibration during a ground start.
//
// Use this setting to give you time to position the aircraft horizontally
// for the INS calibration.
//
// The default is to begin INS calibration immediately at startup.
//
#define GROUND_START_DELAY  0


// We use mode 3 elevon output with both elevon servos "normal"
#define ELEVON_OUTPUT 3
#define RC1_REV 1
#define RC2_REV 1

//////////////////////////////////////////////////////////////////////////////
// ENABLE_AIR_START                         OPTIONAL
//
// If air start is disabled then you will get a ground start (including INS
// calibration) every time the AP is powered up. This means that if you get
// a power glitch or reboot for some reason in the air, you will probably
// crash, but it prevents a lot of problems on the ground like unintentional
// motor start-ups, etc.
//
// If air start is enabled then you will get an air start at power up and a
// ground start will be performed if the speed is near zero when we get gps
// lock.
//
// The default is to disable air start.
//
#define ENABLE_AIR_START    DISABLED


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Altitude measurement and control.
//
// ALT_EST_GAIN                             OPTIONAL
//
// The gain of the altitude estimation function; a lower number results
// in slower error correction and smoother output.  The default is a
// reasonable starting point.
//
//#define ALT_EST_GAIN        0.01

#define MPH_IN_MPS 0.45

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
// HEAD_MAX                                 OPTIONAL
//
// The maximum commanded bank angle in either direction.
// The default is 45 degrees.  Decrease this value if your aircraft is not
// stable or has difficulty maintaining altitude in a steep bank.
//
// PITCH_MAX                                OPTIONAL
//
// The maximum commanded pitch up angle.
// The default is 15 degrees.  Care should be taken not to set this value too
// large, as the aircraft may stall.
//
// PITCH_MIN
//
// The maximum commanded pitch down angle.  Note that this value must be
// negative.  The default is -25 degrees.  Care should be taken not to set
// this value too large as it may result in overspeeding the aircraft.
//
// PITCH_TARGET
//
// The target pitch for cruise flight.  When the APM measures this pitch
// value, the pitch error will be calculated to be 0 for the pitch PID
// control loop.
//
// THROTTLE_MIN                             OPTIONAL
//
// The minimum throttle setting to which the autopilot will reduce the
// throttle while descending.  The default is zero, which is
// suitable for aircraft with a steady power-off glide.  Increase this
// value if your aircraft needs throttle to maintain a stable descent in
// level flight.
//
// THROTTLE_CRUISE                          OPTIONAL
//
// The approximate throttle setting to achieve AIRSPEED_CRUISE in level flight.
// The default is 45%, which is reasonable for a modestly powered aircraft.
//
// THROTTLE_MAX                             OPTIONAL
//
// The maximum throttle setting the autopilot will apply.  The default is 75%.
// Reduce this value if your aicraft is overpowered, or has complex flight
// characteristics at high throttle settings.
//
// THROTTLE_SLEW_LIMIT                      OPTIONAL
//
// Limits the slew rate of the throttle, in percent per second.  Helps
// avoid sudden throttle changes, which can destabilise the aircraft.
// A setting of zero disables the feature.  Range 1 to 100.
// Default is zero (disabled).
//
// AIRSPEED_FBW_MIN                         OPTIONAL (also used for throttle "nudging" in AUTO)
// AIRSPEED_FBW_MAX                         OPTIONAL (also used for throttle "nudging" in AUTO)
//
// Airspeed corresponding to minimum and maximum throttle in Fly By Wire B mode.
// The defaults are 6 and 30 metres per second.
//
// AIRSPEED_FBW_MAX also sets the maximum airspeed that the cruise airspeed can be "nudged" to in AUTO mode when ENABLE_STICK_MIXING is set.
// In AUTO the cruise airspeed can be increased between AIRSPEED_CRUISE and AIRSPEED_FBW_MAX by positioning the throttle
// stick in the top 1/2 of its range.  Throttle stick in the bottom 1/2 provide regular AUTO control.
//
// AIRSPEED_CRUISE                          OPTIONAL
//
// The speed in metres per second to maintain during cruise.  The default
// is 10m/s, which is a conservative value suitable for relatively small,
// light aircraft.
//
// MIN_GNDSPEED                             OPTIONAL
//
// The minimum ground speed in metres per second to maintain during
// cruise. A value of 0 will disable any attempt to maintain a minumum
// speed over ground.
//
//
// ALT_HOLD_FBW                             OPTIONAL
//
// The minimum altitude in meters to allow under FBW, if you go below this, FBW will correct and level out the plane
//
// WP_RADIUS_DEFAULT                        OPTIONAL
//
// When the user performs a factory reset on the APM, set the waypoint radius
// (the radius from a target waypoint within which the APM will consider
// itself to have arrived at the waypoint) to this value in meters.  This is
// mainly intended to allow users to start using the APM without running the
// WaypointWriter first.
//
// LOITER_RADIUS_DEFAULT                    OPTIONAL
//
// When the user performs a factory reset on the APM, set the loiter radius
// (the distance the APM will attempt to maintain from a waypoint while
// loitering) to this value in meters.  This is mainly intended to allow
// users to start using the APM without running the WaypointWriter first.
//
// USE_CURRENT_ALT                          OPTIONAL
// ALT_HOLD_HOME                            OPTIONAL
//
// When the user performs a factory reset on the APM, set the flag for weather
// the current altitude or ALT_HOLD_HOME altitude should be used for Return To Launch.
// Also, set the value of USE_CURRENT_ALT in meters.  This is mainly intended to allow
// users to start using the APM without running the WaypointWriter first.
//

#define HEAD_MAX                30
#define PITCH_MAX               25
#define PITCH_MIN               -25
#define PITCH_TARGET            0
#define THROTTLE_MIN            0 // percent
#define THROTTLE_CRUISE         60
#define THROTTLE_MAX            100
#define THROTTLE_SLEW_LIMIT     75
#define AIRSPEED_FBW_MIN        (15 * MPH_IN_MPS)
#define AIRSPEED_FBW_MAX        (45 * MPH_IN_MPS)
#define AIRSPEED_CRUISE         (30 * MPH_IN_MPS)
#define MIN_GNDSPEED            0
#define ALT_HOLD_FBW            0          // TODO: Set to 0 for now so we can line up to land under FBWB; Can we make a "landing" mode which is basically FBWB but at low altitude with "down" instruction, it flares and lands?
#define WP_RADIUS_DEFAULT       15
#define LOITER_RADIUS_DEFAULT   25
#define USE_CURRENT_ALT         FALSE
#define ALT_HOLD_HOME           15

//////////////////////////////////////////////////////////////////////////////
// THROTTLE_FAILSAFE                        OPTIONAL
// THROTTLE_FS_VALUE                        OPTIONAL
//
// The throttle failsafe allows you to configure a software failsafe activated
// by a setting on the throttle input channel (channel 3).  Enabling this failsafe
// also enables "short failsafe" conditions (see below) based on loss of
// rc override control from the GCS
//
// This can be used to achieve a failsafe override on loss of radio control
// without having to sacrifice one of your FLIGHT_MODE settings, as the
// throttle failsafe overrides the switch-selected mode.
//
// Throttle failsafe is enabled by setting THROTTLE_FAILSAFE to 1.  The default
// is for it to be enabled.
//
// If the throttle failsafe is enabled, THROTTLE_FS_VALUE sets the channel value
// below which the failsafe engages.  The default is 975ms, which is a very low
// throttle setting.  Most transmitters will let you trim the manual throttle
// position up so that you cannot engage the failsafe with a regular stick movement.
//
// Configure your receiver's failsafe setting for the throttle channel to the
// absolute minimum, and use the ArduPilotMega_demo program to check that
// you cannot reach that value with the throttle control.  Leave a margin of
// at least 50 microseconds between the lowest throttle setting and
// THROTTLE_FS_VALUE.
//
// TODO: This will want to be disabled for full-auto wher there never is a RC controller turned on
// Enable it for now during testing
#define THROTTLE_FAILSAFE   ENABLED
#define THROTTLE_FS_VALUE   975


//////////////////////////////////////////////////////////////////////////////

// GCS_HEARTBEAT_FAILSAFE               OPTIONAL
// SHORT_FAILSAFE_ACTION                OPTIONAL
// LONG_FAILSAFE_ACTION                 OPTIONAL

// There are two basic conditions which can trigger a failsafe.  One is a loss of control signal.
// Normally this means a loss of the radio control RC signal.  However if rc override from the
// GCS is in use then this can mean a loss of communication with the GCS.  Such a failsafe will be
// classified as either short (greater than 1.5 seconds but less than 20) or long (greater than 20).
// Also, if GCS_HEARTBEAT_FAILSAFE is enabled and a heartbeat signal from the GCS has not been received
// in the preceeding 20 seconds then this will also trigger a "long" failsafe.
//
// The SHORT_FAILSAFE_ACTION and LONG_FAILSAFE_ACTION settings determines what APM will do when
// a failsafe mode is entered while flying in AUTO or LOITER mode.  This is important in order to avoid
// accidental failsafe behaviour when flying waypoints that take the aircraft
// out of radio range.
//
// If SHORT_FAILSAFE_ACTION is 1, when failsafe is entered in AUTO or LOITER modes,
// the aircraft will head for home in RTL mode.  If the failsafe condition is
// resolved, it will return to AUTO or LOITER mode.

// If LONG_FAILSAFE_ACTION is 1, when failsafe is entered in AUTO or LOITER modes,
// the aircraft will head for home in RTL mode.  If the failsafe condition is
// resolved the aircraft will not be returned to AUTO or LOITER mode, but will continue home

// If XX_FAILSAFE_ACTION is 0 and the applicable failsafe occurs while in AUTO or LOITER
// mode the aircraft will continue in that mode ignoring the failsafe condition.

// Note that for Manual, Stabilize, and Fly-By-Wire (A and B) modes the aircraft will always
// enter a circling mode for short failsafe conditions and will be switched to RTL for long
// failsafe conditions.  RTL mode is unaffected by failsafe conditions.
//
// The default is to have GCS Heartbeat failsafes DISABLED
// The default behaviour is to ignore failsafes in AUTO and LOITER modes.
//
// TODO: This will want to be disabled for full-auto wher there might not be a GCS turned on - but see line just above this
#define GCS_HEARTBEAT_FAILSAFE  ENABLED

// TODO: For now, circle then RTL -- we will want to fix this algorithm, especially for low battery warning which is not remedied
// We want to RTL then glide/land, not circle then glide or circle then RTL/circle
#define SHORT_FAILSAFE_ACTION   1
#define LONG_FAILSAFE_ACTION    1


//////////////////////////////////////////////////////////////////////////////
// Fencing and recovery
//
// FENCE_ACTION
// What to do on fence breach. If this is set to 0 then no action is taken, and geofencing is disabled. If this is set to 1 then the
// plane will enter GUIDED mode, with the target waypoint as the fence return point. If this is set to 2 then the fence breach is reported
// to the ground station, but no other action is taken. If set to 3 then the plane enters guided mode but the pilot retains manual throttle control.
//
#define GEOFENCE_ENABLED        ENABLED
#define FENCE_MINALT            0       // Allow landing
#define FENCE_MAXALT            120     // Under 400 feet for FAA nazis
#define FENCE_ACTION            3       //

#define FENCE_CHANNEL           6       // RIGHT control knob
#define RESET_SWITCH_CHANNEL    6       // Use same RIGHT control knob
#define FENCE_ENABLE_PWM        1200    // A little bit of knob turns on fence
#define RESET_SWITCH_CHAN_PWM   1750    // A lot of knob resets

//////////////////////////////////////////////////////////////////////////////
// Attitude control gains
//
// Tuning values for the attitude control PID loops.
//
// The P term is the primary tuning value.  This determines how the control
// deflection varies in proportion to the required correction.
//
// The I term is used to help control surfaces settle.  This value should
// normally be kept low.
//
// The D term is used to control overshoot.  Avoid using or adjusting this
// term if you are not familiar with tuning PID loops.  It should normally
// be zero for most aircraft.
//
// Note: When tuning these values, start with changes of no more than 25% at
// a time.
//
// SERVO_ROLL_P                             OPTIONAL
// SERVO_ROLL_I                             OPTIONAL
// SERVO_ROLL_D                             OPTIONAL
//
// P, I and D terms for roll control.  Defaults are 0.4, 0, 0.
//
// SERVO_ROLL_INT_MAX                       OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 5 degrees.
//
// ROLL_SLEW_LIMIT                          EXPERIMENTAL
//
// Limits the slew rate of the roll control in degrees per second.  If zero,
// slew rate is not limited.  Default is to not limit the roll control slew rate.
// (This feature is currently not implemented.)
//
// SERVO_PITCH_P                            OPTIONAL
// SERVO_PITCH_I                            OPTIONAL
// SERVO_PITCH_D                            OPTIONAL
//
// P, I and D terms for the pitch control.  Defaults are 0.6, 0, 0.
//
// SERVO_PITCH_INT_MAX                      OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. native flight
// AoA).
// Default is 5 degrees.
//
// PITCH_COMP                               OPTIONAL
//
// Adds pitch input to compensate for the loss of lift due to roll control.
// Default is 0.20 (20% of roll control also applied to pitch control).
//
// SERVO_YAW_P                              OPTIONAL
// SERVO_YAW_I                              OPTIONAL
// SERVO_YAW_D                              OPTIONAL
//
// P, I and D terms for the YAW control.  Defaults are 0., 0., 0.
// Note units of this control loop are unusual.  PID input is in m/s**2.
//
// SERVO_YAW_INT_MAX                        OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 0.
//
// RUDDER_MIX                               OPTIONAL
//
// Roll to yaw mixing.  This allows for co-ordinated turns.
// Default is 0.50 (50% of roll control also applied to yaw control.)
//
//#define SERVO_ROLL_P        1.087
//#define SERVO_ROLL_I        0.0
//#define SERVO_ROLL_D        0.0
//#define SERVO_ROLL_INT_MAX  5
//#define ROLL_SLEW_LIMIT     0//

//#define SERVO_PITCH_P       0.919
//#define SERVO_PITCH_I       0.0
//#define SERVO_PITCH_D       0.0
//#define SERVO_PITCH_INT_MAX 5
//#define PITCH_COMP          0.2//

//#define SERVO_YAW_P         0.0     // Default is zero.  A suggested value if you want to use this parameter is 0.5
//#define SERVO_YAW_I         0.0
//#define SERVO_YAW_D         0.0
//#define SERVO_YAW_INT_MAX   5
//#define RUDDER_MIX          0          // We're a wing.  We have no rudder

//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
//
// Tuning values for the navigation control PID loops.
//
// The P term is the primary tuning value.  This determines how the control
// deflection varies in proportion to the required correction.
//
// The I term is used to control drift.
//
// The D term is used to control overshoot.  Avoid adjusting this term if
// you are not familiar with tuning PID loops.
//
// Note: When tuning these values, start with changes of no more than 25% at
// a time.
//
// NAV_ROLL_P                               OPTIONAL
// NAV_ROLL_I                               OPTIONAL
// NAV_ROLL_D                               OPTIONAL
//
// P, I and D terms for navigation control over roll, normally used for
// controlling the aircraft's course.  The P term controls how aggressively
// the aircraft will bank to change or hold course.
// Defaults are 0.7, 0.0, 0.02.
//
// NAV_ROLL_INT_MAX                         OPTIONAL
//
// Maximum control offset due to the integral.  This prevents the control
// output from being overdriven due to a persistent offset (e.g. crosstracking).
// Default is 5 degrees.
//
// NAV_PITCH_ASP_P                          OPTIONAL
// NAV_PITCH_ASP_I                          OPTIONAL
// NAV_PITCH_ASP_D                          OPTIONAL
//
// P, I and D terms for pitch adjustments made to maintain airspeed.
// Defaults are 0.65, 0, 0.
//
// NAV_PITCH_ASP_INT_MAX                    OPTIONAL
//
// Maximum pitch offset due to the integral.  This limits the control
// output from being overdriven due to a persistent offset (eg. inability
// to maintain the programmed airspeed).
// Default is 5 degrees.
//
// NAV_PITCH_ALT_P                          OPTIONAL
// NAV_PITCH_ALT_I                          OPTIONAL
// NAV_PITCH_ALT_D                          OPTIONAL
//
// P, I and D terms for pitch adjustments made to maintain altitude.
// Defaults are 0.65, 0, 0.
//
// NAV_PITCH_ALT_INT_MAX                    OPTIONAL
//
// Maximum pitch offset due to the integral.  This limits the control
// output from being overdriven due to a persistent offset (eg. inability
// to maintain the programmed altitude).
// Default is 5 meters.
//
//#define NAV_ROLL_P          0.7
//#define NAV_ROLL_I          0
//#define NAV_ROLL_D          0.02
//#define NAV_ROLL_INT_MAX    5//

//#define NAV_PITCH_ASP_P     0.65
//#define NAV_PITCH_ASP_I     0.1
//#define NAV_PITCH_ASP_D     0.0
//#define NAV_PITCH_ASP_INT_MAX 5//

//#define NAV_PITCH_ALT_P     0.65
//#define NAV_PITCH_ALT_I     0.1
//#define NAV_PITCH_ALT_D     0.0
//#define NAV_PITCH_ALT_INT_MAX 5

//////////////////////////////////////////////////////////////////////////////
// Energy/Altitude control gains
//
// The Energy/altitude control system uses throttle input to control aircraft
// altitude.
//
// The P term is the primary tuning value.  This determines how the throttle
// setting varies in proportion to the required correction.
//
// The I term is used to compensate for small offsets.
//
// The D term is used to control overshoot.  Avoid adjusting this term if
// you are not familiar with tuning PID loops.
//
// Note units of this control loop are unusual.  PID input is in m**2/s**2.
//
// THROTTLE_TE_P                            OPTIONAL
// THROTTLE_TE_I                            OPTIONAL
// THROTTLE_TE_D                            OPTIONAL
//
// P, I and D terms for throttle adjustments made to control altitude.
// Defaults are 0.5, 0, 0.
//
// THROTTLE_TE_INT_MAX                      OPTIONAL
//
// Maximum throttle input due to the integral term.  This limits the
// throttle from being overdriven due to a persistent offset (e.g.
// inability to maintain the programmed altitude).
// Default is 20%.
//
// P_TO_T                                   OPTIONAL
//
// Pitch to throttle feed-forward gain.  Default is 0.
//
// T_TO_P                                   OPTIONAL
//
// Throttle to pitch feed-forward gain.  Default is 0.
//
//#define THROTTLE_TE_P       0.50
//#define THROTTLE_TE_I       0.0
//#define THROTTLE_TE_D       0.0
//#define THROTTLE_TE_INT_MAX 20//

//#define P_TO_T              0
//#define T_TO_P              0

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
// XTRACK_GAIN                              OPTIONAL
//
// Crosstrack compensation in degrees per metre off track.
// Default value is 1.0 degrees per metre.  Values lower than 0.001 will
// disable crosstrack compensation.
//
// XTRACK_ENTRY_ANGLE                       OPTIONAL
//
// Maximum angle used to correct for track following.
// Default value is 30 degrees.
//
//#define XTRACK_GAIN         1  // deg/m
//#define XTRACK_ENTRY_ANGLE  30 // deg

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// DEBUGGING
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Dataflash logging control
//
#define LOGGING_ENABLED     ENABLED

//////////////////////////////////////////////////////////////////////////////
// Debugging interface
//
// DEBUG_PORT                               OPTIONAL
//
// The APM will periodically send messages reporting what it is doing; this
// variable determines to which serial port they will be sent.  Port 0 is the
// USB serial port on the shield, port 3 is the telemetry port.
//
//#define DEBUG_PORT            0
//

