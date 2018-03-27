# Kingst LA1010 support for Sigrok

Known issues:
1. Configuration of PWM output channels isn't provided by GUI;
2. The Spartan firmware can't be downloaded if the device is connected to USBv3 (worked only with USBv2).

The directory "pwm" contains patches for enabling the support of configuration of output PWM channels in "pulseview".
This decision is not very good, even more bad. But at this moment there is no other possibility to implement PWM channels configuration support in "pulseview".
PWM setting (frequency or duty) is applied when corresponding channel is activated in settings in "pulseview". That is why to change settings for active PWN channel the channel must be disabled and again enabled.
