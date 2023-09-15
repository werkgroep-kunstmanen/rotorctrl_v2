# rotorctrl_v2
rotorcontroller version 2, supporting both AVR and ESP controllers.
Note: for AVR this is a replacement of repo 'rotorctrl'. See rotor_spec.h, where all the settings are. For AVR, remove the cpp files since for AVR controolers they cannot be compiled (which is tried/giving errors although they are not used with AVR processor defined...) With ESP, you can control via USB or WiFi. Calculations acn also be done inside the controller, in that case Kepler files need to be uploaded. This is all supported by xtrack, see: 
http://www.alblas.demon.nl/wsat/software/soft_trek.html
