
cd ../../..

set LDSCRIPT = config/boards/NUCLEO_F767/STM32F76xxI.ld

set PROJECT_BOARD=microrusefi
set PROJECT_CPU=ARCH_STM32F7
set EXTRA_PARAMS=-DDUMMY -DDEFAULT_ENGINE_TYPE=MINIMAL_PINS 

call config/boards/common_make.bat

call config/boards/clean_env_variables.bat
