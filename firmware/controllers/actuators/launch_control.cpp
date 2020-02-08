/*
 * launch_control.cpp
 *
 *  Created on: 10. sep. 2019
 *      Author: Ola Ruud
 */
#include "global.h"
//#if EFI_LAUNCH_CONTROL
#include "boost_control.h"
#include "vehicle_speed.h"
#include "launch_control.h"
#include "tunerstudio_configuration.h"
#include "boost_control.h"
#include "io_pins.h"
#include "engine_configuration.h"
#include "engine_controller.h"
#include "periodic_task.h"
#include "pin_repository.h"
#include "allsensors.h"
#include "engine_math.h"
#include "efi_gpio.h"
#include "advance_map.h"
#include "engine_state.h"
#if defined(HAS_OS_ACCESS)
#error "Unexpected OS ACCESS HERE"
#endif


static Logging *logger;




#if EFI_TUNER_STUDIO
extern TunerStudioOutputChannels tsOutputChannels;
#endif /* EFI_TUNER_STUDIO */

EXTERN_ENGINE;

class LaunchControl: public PeriodicTimerController {
public:

int getPeriodMs() override {
return 10;
	}
void PeriodicTask() override {

	int rpm = GET_RPM_VALUE;
	int speed = getVehicleSpeed();
	int tps = getTPS(PASS_ENGINE_PARAMETER_SIGNATURE);
	int speedtreeshold = engineConfiguration->launchSpeedTreshold;
	int cutRpmRange = engineConfiguration->hardCutRpmRange;
	int tpstreshold = engineConfiguration->launchTpsTreshold;
	bool activateSwitchCondition = false;
	int launchRpmValue = engineConfiguration->launchRpm;

	//if (!engineConfiguration->antiLagEnabled)
	//			return;

	if (engineConfiguration->launchActivationMode == SWITCH_INPUT_LAUNCH) {
				if (CONFIG(launchActivatePin) != GPIO_UNASSIGNED) {
					engine->launchActivatePinState = efiReadPin(CONFIG(launchActivatePin));
				}
				activateSwitchCondition = engine->launchActivatePinState;

			   }
	else if (engineConfiguration->launchActivationMode == CLUTCH_INPUT_LAUNCH) {
			if (CONFIG(clutchDownPin) != GPIO_UNASSIGNED) {
				engine->clutchDownState = efiReadPin(CONFIG(clutchDownPin));
	           	}
			activateSwitchCondition = engine->clutchDownState;

	} 

	activateSwitchCondition = activateSwitchCondition||(engineConfiguration->launchActivationMode == ALWAYS_ACTIVE_LAUNCH);
		
		
}
	bool rpmCondition = (launchRpmValue < rpm);
	bool tpsCondition = (tpstreshold < tps);

	 bool speedCondition = (engineConfiguration->launchDisableBySpeed && (speedtreeshold > speed)) || ! engineConfiguration->launchDisableBySpeed;
	engine->isLaunchCondition = (speedCondition && activateSwitchCondition  && rpmCondition  && tpsCondition);

		if (engineConfiguration->debugMode == DBG_LAUNCH) {
	#if EFI_TUNER_STUDIO
				tsOutputChannels.debugIntField1 =  rpmCondition;
				tsOutputChannels.debugIntField2 = tpsCondition;
				tsOutputChannels.debugIntField3 = speedCondition;
				tsOutputChannels.debugIntField4 = activateSwitchCondition;
				tsOutputChannels.debugIntField5 = engine->clutchDownState;
				tsOutputChannels.debugFloatField1 = engine->launchActivatePinState;
				tsOutputChannels.debugFloatField2  = engine->isLaunchCondition;
	#endif /* EFI_TUNER_STUDIO */
			}



	 engine->setLaunchBoostDuty = engine->isLaunchCondition;
	 engine->applyLaunchControlRetard = engine->isLaunchCondition;



  }


};

static LaunchControl Launch;

void setDefaultLaunchParameters(DECLARE_CONFIG_PARAMETER_SIGNATURE) {
	engineConfiguration->launchRpm = 4000;
	engineConfiguration->launchTimingRetard = 10;
	engineConfiguration->launchTimingRpmRange = 500;
	engineConfiguration->launchSparkCutEnable = false;
	engineConfiguration->launchFuelCutEnable = false;
	engineConfiguration->hardCutRpmRange = 500;
	engineConfiguration->antilagTimeout = 3;
	engineConfiguration->launchSpeedTreshold = 10;
	engineConfiguration->launchRpmTreshold = 3000;
	engineConfiguration->launchFuelAdded = 10;
	engineConfiguration->launchBoostDuty = 70;
	engineConfiguration->launchActivateDelay = 3;
}


void applyLaunchControlLimiting(bool *limitedSpark, bool *limitedFuel DECLARE_ENGINE_PARAMETER_SUFFIX) {
	*limitedSpark = ((engine->isLaunchCondition) && (engineConfiguration->launchSparkCutEnable)) ;
	*limitedFuel = ((engine->isLaunchCondition) && (engineConfiguration->launchFuelCutEnable));
}




void initLaunchControl(Logging *sharedLogger DECLARE_ENGINE_PARAMETER_SUFFIX) {
	logger = sharedLogger;
	Launch.Start();
}
//#endif
