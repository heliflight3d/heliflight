
## Heliflight 3D

Heliflight 3D is flight controller software (firmware) used to fly single-rotor helicopters.

This fork differs from Baseflight and Cleanflight in that it focuses on 3D flight performance for helicopters. Heliflight 3D was forked from Betaflight 4.2.

## Important Notice: During the initial development period, Heliflight 3D is *NOT* optimized for slower hardware.  It is highly suggested to use a STM32F7 based flight controller.


### Installation for End-Users:

Please see the "Releases" page on this Github repo to download the latest official release for your FC board:
* [Click here to go to Releases](https://github.com/heliflight3d/heliflight/releases)

Please ignore any snapshots on this page (github is showing them automatically).

### Installation for Beta-testers:

Please see the "Actions" tab on this Github repo to download the latest build snapshot for your FC board:
* [Click here to go to the Actions tab](https://github.com/heliflight3d/heliflight/actions)

Please note that build snapshots are available for beta-testing, and are otherwise _not_ supported.


## Betaflight News

![Important Notice: Support for STM32F3 based flight controllers was dropped in Betaflight 4.1. (This includes all boards with 'F3' in the name.)](docs/assets/images/stm32f3_retirement_notice.svg)

(Please see the [note](https://github.com/betaflight/betaflight#end-of-active-development-for-stm32f3-based-flight-controllers) below.)

### Requirements for the submission of new and updated targets

The following new requirements for pull requests adding new targets or modifying existing targets are put in place from now on:

1. No new F3 based targets will be accepted;

2. For any new target that is to be added, only a Unified Target config into https://github.com/betaflight/unified-targets/tree/master/configs/default needs to be submitted. See the [instructions](https://github.com/betaflight/betaflight/blob/master/docs/TargetMaintenance/CreatingAUnifiedTarget.md) for how to create a Unified Target configuration. If there is no Unified Target for the MCU type of the new target (see instructions above), then a 'legacy' format target definition into `src/main/target/` has to be submitted as well;

3. For changes to existing targets, the change needs to be applied to the Unified Target config in https://github.com/betaflight/unified-targets/tree/master/configs/default. If no Unified Target configuration for the target exists, a new Unified Target configuration will have to be created and submitted. If there is no Unified Target for the MCU type of the new target (see instructions above), then an update to the 'legacy' format target definition in `src/main/target/` has to be submitted alongside the update to the Unified Target configuration.


### End of active development for STM32F3 based flight controllers

For a while now, development of Betaflight for flight controllers based on the STM32F3 chip has been hampered by a severe limitation that this chip has: Unlike the STM32F4 and STM32F7 models, the STM32F3 versions that are used on flight controllers have only a very limited amount of flash space available to fit the firmware into. This has meant that, starting from around version 3.3, the majority of the new features that were developed for Betaflight could not be added to STM32F3 based boards. Even worse, due to improvement in basic features, other more and more of the less commonly used features had to be removed from these flight controllers, and a number of them are at a point where they only support the bare minimum of functionality required to make them fly.

This means that, even if we kept supporting STM32F3 based boards in future releases, there would only be little advantage in this, as there simply is no space left on STM32F3 to add any of the new features that these releases will contain.

For this reason, and because the effort required to remove features from STM32F3 based flight controllers on a weekly basis is cutting into the time that we have to actually develop new features, we have decided to drop support for STM32F3 based flight controllers after the last release of 4.0.

This does not mean that it won't be possible to use these flight controllers after this point in time - they will still work fine when used with the last release of 4.0, just as there are thousands of users who are still enjoying their STM32F1 based flight controllers with Betaflight 3.2.5. We will also strive to keep these versions supported in new releases of configurator, so that users still using these flight controllers will be able to configure them with the same configurator that they use to configure their STM32F4 and STM32F7 based boards.


## Features

Betaflight has the following features:

* Multi-color RGB LED strip support (each LED can be a different color using variable length WS2811 Addressable RGB strips - use for Orientation Indicators, Low Battery Warning, Flight Mode Status, Initialization Troubleshooting, etc)
* DShot (150, 300, 600 and 1200), Multishot, and Oneshot (125 and 42) motor protocol support
* Blackbox flight recorder logging (to onboard flash or external microSD card where equipped)
* Support for targets that use the STM32 F7, F4 and F3 processors
* PWM, PPM, and Serial (SBus, SumH, SumD, Spektrum 1024/2048, XBus, etc) RX connection with failsafe detection
* Multiple telemetry protocols (CSRF, FrSky, HoTT smart-port, MSP, etc)
* RSSI via ADC - Uses ADC to read PWM RSSI signals, tested with FrSky D4R-II, X8R, X4R-SB, & XSR
* OSD support & configuration without needing third-party OSD software/firmware/comm devices
* In-flight manual PID tuning and rate adjustment
* Rate profiles and in-flight selection of them
* Configurable serial ports for Serial RX, Telemetry, ESC telemetry, MSP, GPS, OSD, Sonar, etc - Use most devices on any port, softserial included
* VTX support for Unify Pro and IRC Tramp
* and MUCH, MUCH more.

## Installation & Documentation

See: https://github.com/betaflight/betaflight/wiki

## Support and Developers Channel

There's a dedicated Slack chat channel here:

https://slack.betaflight.com/

Etiquette: Don't ask to ask and please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately.

## Configuration Tool

To configure Betaflight you should use the Betaflight-configurator GUI tool (Windows/OSX/Linux) which can be found here:

https://github.com/betaflight/betaflight-configurator/releases/latest

## Contributing

Contributions are welcome and encouraged. You can contribute in many ways:

* implement a new feature in the firmware or in configurator (see [below](#Developers));
* documentation updates and corrections;
* How-To guides - received help? Help others!
* bug reporting & fixes;
* new feature ideas & suggestions;
* provide a new translation for configurator, or help us maintain the existing ones (see [below](#Translators)).

The best place to start is the Betaflight Slack (registration [here](https://slack.betaflight.com/)). Next place is the github issue tracker:

https://github.com/betaflight/betaflight/issues
https://github.com/betaflight/betaflight-configurator/issues

Before creating new issues please check to see if there is an existing one, search first otherwise you waste people's time when they could be coding instead!

If you want to contribute to our efforts financially, please consider making a donation to us through [PayPal](https://paypal.me/betaflight).

If you want to contribute financially on an ongoing basis, you should consider becoming a patron for us on [Patreon](https://www.patreon.com/betaflight).

## Developers

Contribution of bugfixes and new features is encouraged. Please be aware that we have a thorough review process for pull requests, and be prepared to explain what you want to achieve with your pull request.
Before starting to write code, please read our [development guidelines](docs/development/Development.md ) and [coding style definition](docs/development/CodingStyle.md).

TravisCI is used to run automatic builds

https://travis-ci.com/betaflight/betaflight

[![Build Status](https://travis-ci.com/betaflight/betaflight.svg?branch=master)](https://travis-ci.com/betaflight/betaflight)

## Translators

We want to make Betaflight accessible for pilots who are not fluent in English, and for this reason we are currently maintaining translations into 18 languages for Betaflight Configurator: Català, Deutsch, Español, Euskera, Français, Galego, Hrvatski, Bahasa Indonesia, Italiano, 日本語, 한국어, Latviešu, Português, Português Brasileiro, polski, Русский язык, Svenska, 简体中文.
We have got a team of volunteer translators who do this work, but additional translators are always welcome to share the workload, and we are keen to add additional languages. If you would like to help us with translations, you have got the following options:
- if you help by suggesting some updates or improvements to translations in a language you are familiar with, head to [crowdin](https://crowdin.com/project/betaflight-configurator) and add your suggested translations there;
- if you would like to start working on the translation for a new language, or take on responsibility for proof-reading the translation for a language you are very familiar with, please head to the Betaflight Slack (registration [here](https://slack.betaflight.com/)), and join the '#team\_translation' channel - the people in there can help you to get a new language added, or set you up as a proof reader.

## Betaflight Releases

https://github.com/betaflight/betaflight/releases

## Open Source / Contributors

Betaflight is software that is **open source** and is available free of charge without warranty to all users.

Betaflight is forked from Cleanflight, so thanks goes to all those whom have contributed to Cleanflight and its origins.

Origins for this fork (Thanks!):
* **Alexinparis** (for MultiWii),
* **timecop** (for Baseflight),
* **Dominic Clifton** (for Cleanflight),
* **borisbstyle** (for Betaflight), and
* **Sambas** (for the original STM32F4 port).

The Betaflight Configurator is forked from Cleanflight Configurator and its origins.

Origins for Betaflight Configurator:
* **Dominic Clifton** (for Cleanflight configurator), and
* **ctn** (for the original Configurator).

Big thanks to current and past contributors:
* Budden, Martin (martinbudden)
* Bardwell, Joshua (joshuabardwell)
* Blackman, Jason (blckmn)
* ctzsnooze
* Höglund, Anders (andershoglund)
* Ledvina, Petr (ledvinap) - **IO code awesomeness!**
* kc10kevin
* Keeble, Gary (MadmanK)
* Keller, Michael (mikeller) - **Configurator brilliance**
* Kravcov, Albert (skaman82) - **Configurator brilliance**
* MJ666
* Nathan (nathantsoi)
* ravnav
* sambas - **bringing us the F4**
* savaga
* Stålheim, Anton (KiteAnton)

And many many others who haven't been mentioned....
