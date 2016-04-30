### AutoQuad Flight Control Firmware Changes List

This file describes changes in the firmware, typically since the previous minor version (eg. 7.x).  Changes are listed newest first.  Some changes are summarized, for details see Git commit hitsory and notes.  Information here should be geared more towards users than developers.

	Legend:
	+ : new feature/function
	- : removed feature/function
	* : bug/deficiency fix
	~ : enhancement/non-breaking change
	! : important change, possible danger, change of default behavior, etc.


##### 7.1.1923 - Apr. 30, 2016 - `feature_integration` branch

`*` Fix configurable signaling outputs (LEDs/beeper).  
`*` Remove yaw angle filter (and corresponding CTRL_YAW_ANG_TAU param), fixes mission waypoint headings and orbit-type waypoint with PID attitude controller.  
`*` Enforce orbit waypoint relative heading to target, fixes possible fly-away condition in all firmware since v6.6.  
`*` Fix maximum horizontal speed for orbit waypoint (it used global default instead of wpt setting in all firmware since v6.6).  
`~` Increase tolerance for orbit waypoint arrival detection (still not very reliable at higher orbit speeds).  
`~` On wpt arrival, only freeze heading if it was relative (prevents unnecessary yawing in some cases).  
`*` Prevent all motor PWM/CAN port outputs once HILS has been activated until system restart.  

##### 7.1.1919 - Apr. 15, 2016 - `feature_integration` branch

`*` Fix automatic yaw to north at takeoff with PID controller.  

##### 7.1.1918 - Apr. 14, 2016 - `feature_integration` branch

`~` Aggregate changes from `new_controls`, `sport_telemetry`, `memory_manage`, `simu` branches.  
`*` Fix RC status LED when signal is lost with S-BUS and SUM-D radio types.  
`+` Add parameters for radio failsafe stage 1 and 2 timeouts.  
`~` Increase maximum waypoints to 100.  

##### 7.1.1915 - Apr. 14, 2016 - `simu` branch

`+` Add hooks for external IMU/GPS data.  
`+` Add optional HILS support.  

##### 7.1.1910 - Apr. 13, 2016 - `memory_manage` branch

`~` Increase maximum waypoints to 50.  
`~` Internal structure updates to optimize memory.  
`~` Signaling module now controls onboard LEDs, is no longer optional.  

##### 7.1.1902 - Apr. 13, 2016 - `sport_telemetry` branch

`+` Add native FrSky S-Port telemetry option.  

##### 7.1.1900 - Apr. 13, 2016 - `new_controls` branch

`+` Add rate-of-rotation "Acro" control mode.  
`+` Add Limited rate-of-rotation control mode.  Like acro mode but total tilt angle is limited to the same as it would be in normal angle-control flight modes.  
`+` Heading-free mode is now available in manual angle-control and altitude-hold-only flight modes (but not rate modes).  This is disabled by default.  
`+` Options to proportionally cut throttle if craft attitude is detected as inverted (tilted > 90 degrees on any axis).  Can be enabled in altitude hold and/or manual modes.  
`+` Add option to disable USB Mass Storage Controller (MSC) feature on M4 (allows logging, other SD card access when using USB).  
`!` Add distinct parameter for nav maximum vertical ascent speed (NAV_MAX_ASCENT replaces the use of NAV_ALT_POS_OM for the same purpose).  

##### 7.1.1897 - Apr. 13, 2016

`*` Fix bug in 7.1.1895 preventing gimbal roll/pitch reversing.  
`*` Fix altitude in local position Mavlink message & mission programming response codes.

##### 7.1.1895 - Mar. 18, 2016

`+` Add ability to adjust some parameter values live using RC, e.g. attitude/nav PIDs, all Quatos settings, live gimbal tuning, default nav speed limits, control factors, etc.  
        Adds option to save the resulting adjusted values directly to permanent onboard storage.  
        Use QGC 1.7.B1 or newer for setup, full list of adjustable params, and instructions (or see notes in config_default.h).  
`*` Fix issues with CONFIG_VERSION changes, eg. ignoring all saved params when version changes and vice versa when new params were not properly assigned defaults. New system allows loading defaults selectively vs. all at once.  
        *From here on, please do not manually change the CONFIG_VERSION parameter onboard or in saved files, it is no longer a necessary workaround and may lead to problems later.*  
`~` Improved initial parameter loading sequence will never overwrite params saved in flash (until/unless user requests it).  
`+` Adds some basic paramater value sanity checking (min/max) when loading/assigning values.  
`~` Provides more detailed feedback (messages) during param loading/assignment operations (status/warnings/errors).  

##### 7.1.1887 - Feb. 21, 2016

`*` Fix random yawing on Landing-type waypoint when previous wpt type used relative heading.  
`*` Fix setting of desired yaw/heading for Landing waypoint via MAVLink/QGC.  
`~` Specifically hold current position for Landing waypoint (possibly prevent diagonal descent). (Note that Lat/Lon cannot be specified for landing-type waypoint, which has always been the case but is not obvious when using the QGC mission planner.)  
`+` Add more diagnostic telemetry data (RC switch statuses and RAM usage).  

##### 7.1.1885 - Feb. 9, 2016

`!` Waypoint skip switch will now go back to start of mission after the last waypoint, allows restarting a mission in flight.  
`*` Fix constant waypoint arrival message when last waypoint is reached.  
`*` Do not re-enter mission mode when there is no viable waypoint to load (eg. after end of mission).  
`~` Add text notices when waypoint is loaded and reached.  
`*` Fix waypoint being recorded while clearing them at the same time using stick commands.

##### 7.1.1884 - Feb. 7, 2016

`~` Enable 2nd serial port option on M4 boards (COMM 2). Shared with PWM ports 7 (Serial Tx, J2.10 on expansion header) and 8 (Serial Rx, J1.10 on header).  When using those ports for PWM (motor) outputs, be sure to **disable any protocol on Serial 2** (QGC Misc. Settings/Serial Port 2/Protocol = None).  
`*` Fix possible endless notification loop if sending of waypoints times out.  
`-` Remove State Of Charge (SOC) parameters and functions since they're not used.  
`~` Start up message notification queue early to allow more startup messages to come through.  Increase size of notification queue to avoid loosing startup messages before comm system is active.  
`~` Exclude unused AQ binary telemetry/command interface from default builds (define HAS_AQ_TELEMETRY to include it).

##### 7.1.1871 - Dec. 7, 2015

`+` ESC type selection via param MOT_ESC_TYPE: 0 = STD PWM ESC (KISS, etc); 1 = M4 onboard ESC; 2 = ESC32 . This setting is only relevant when using PWM control (CAN always assumes ESC32).  
`+` Enable/disable Quatos via param: Set QUATOS_ENABLE = 0 or 1 and restart. Obviously you'll need to change your motor mix as well. Quatos is enabled by default if LIC_KEY parameter is not zero.  
`+` Enable use of Quatos with any PWM ESC. The implementation is the same as Menno's HOTT version, so if you're already flying with that, all your settings should work as-is (just make sure QUATOS_ENABLE is set to 1).  
`+` PWM ESC end-point calibration at startup: To enable, set MOT_ESC_TYPE to 8388608. Or in other words, you set the 24th bit to 1. For all practical purposes this only works when the rest of the system is set up for standard ESCs (motor outputs are PWM type, MOT_MIN and MOT_MAX are set correctly, MOT_ESC_TYPE = 0 normally). At next startup, the calibration request bit is cleared, the new MOT_ESC_TYPE value is saved to flash, and if that succeeds then the calibration runs -- PWM output first goes to MOT_MAX for a few seconds, then to MOT_MIN. Watch the communications console for informative messages.  
`+` ESC32 communication is now established via 1Wire (PWM mode) even when using PID controller if MOT_ESC_TYPE = 2. For example to distribute settings from esc32.txt file. Previously this only worked with Quatos enabled (or when using CAN) because otherwise we didn't know what kind of PWM ESC was being used.  
`+` New param to select the GPS accuracy at which altitude control is based on GPS vs. purely pressure sensor. NAV_ALT_GPS_ACC = 0.8 by default. Set it to zero to disable GPS estimates entirely, for example. Or something high but < 100 (eg. 99) to always use GPS altitude when an antenna is connected (this would be like the old behavior before the altitude UKF was introduced).  
`~` When using M4 onboard ESCs (MOT_ESC_TYPE=1), the MOT_ARM/MIN/START/MAX params are not used -- those values are now configured in the board header files. MOT_ESC_TYPE=1 by default for M4 firmware builds.  
`+` Reverted change for voltage compensation on M4 with onboard ESCs (from 7.1.1853).  
`~` For building FW: Macro USE_QUATOS replaced with HAS_QUATOS (aq.h). If undefined (default), Quatos code and all QUATOS_* params are excluded entirely from the build. In this case obviously QUATOS_ENABLE param isn't available.

##### 7.1.1861 - Oct. 2, 2015

`+` Add live waypoint recording feature.  

		Summary of use of LWR:  
		- Tx switch for LWR selectable with NAV_CTRL_WP_REC on QGC (using new scheme!)  
		- Switch has two functions:  
		  1) waypoint recording/skip: switch active for 1s = record waypoint or  
		  2) skip to next wp if already in mission mode  
		- clear waypoints = left stick is down left, right stick is down right,  
		  and flip WP rec/skip switch to active -- hold for at least 2s.  
		  AQ will beep/blink and send a text message.

`+` Add signaling events for waypoint reached, WP recorded, and WPs cleared.  

#### 7.1.1858 - Apr. 16, 2015 - CURRENT MASTER BRANCH

**!!! Due to new control configuration scheme, the Flight Mode and Home Set/RTH switch channels will be reset to defaults. !!!**  
Also note that these changes are NOT directly compatible with AQ QGC 1.6.3 and lower. It will still mostly work, but any changes to the new parameters in this version will need to be done directly in the Onboard Configuration window.  An updated QGC with a proper setup GUI for the new features is underway.

`+` Add ability to completely customize the controls for various functions like Flight Mode (Manual/PH/Mission), Home actions (set/goto), camera trigger and so on. Any command can be assigned to any channel and any pulse value (switch position) on that channel.  Any control can now also be disabled entirely (eg. if Mission mode is not needed).  
`+` Add distinct Altitude Hold mode which does not engage position hold at the same time, nor require any GPS fix.  This feature is disabled by default until a control (channel and pulse value) is assigned to it via the NAV_CTRL_AH parameter.  
`+` Any reason(s) for not being able to arm are now specifically reported via text messages (eg. if PH is engaged, throttle is up, etc).  
`*` Fix heading-free bug which would reset reference heading if user disarms while HF switch is active.  
`~` Do not set home position or return to home w/out 3D GPS fix.  AQ reports when home position is actually set via plain text and special Mavlink messages.  
`+` Add text message notices when switching to mission or to manual modes.  
`~` Voltage reported via Mavlink is now lightly (low pass) filtered instead of sending raw ADC data. Not much practical difference since QGC smooths the voltage readings anyway.  
`+` Add ability to report current (power) sensor data via Mavlink (at this point using a current sensor directly with AQ still requires custom code or the un-released “PDB” expander.)  
`+` Add more concise system status reporting in Mavlink “custom mode” variable, and for other upcoming telemetry options.  
`*` Fix Mavlink nav_controller_output message values for Heading, Course, and Distance to waypoint (were off by factor of 100).  
`~` Change version number format reported via text message.  This may also break some features of QGC 1.6.3 and lower.  

##### 7.1.1853 - EXPR - Apr. 9, 2015

`~` Change how battery voltage compensation is handled on M4 with onboard brushed motors and Quatos.  
`*` Fix precision of parameters saved to onboard SD card and to DIMU EEPROM (issue originally introduced in r204/b1352).  

##### 7.1.1852 - EXPR - Feb. 17, 2015

`~` New version numbering format now excludes the SVN revision (“r”) number.  New format is  Major.Minor.Build – Label.  
`+` Add ESC32 CAN telemetry over MAVLink.  Use with AQ QGC 1.6.3 and up.  
`*` Prevent possible firmware crash when performing multiple consecutive calibration/parameter saving tasks via Mavlink.  
