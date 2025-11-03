# sl-micro-controllers

This C++ project aggregates the code and documentation for all Ataraxis Micro Controllers (AXMCs) used in Sun (NeuroAI)
lab data acquisition pipelines.

![c++](https://img.shields.io/badge/C++-00599C?style=flat-square&logo=C%2B%2B&logoColor=white)
![license](https://img.shields.io/badge/license-GPLv3-blue)
___

## Detailed Description

This library builds on top of the general architecture defined by the
[ataraxis-micro-controller](https://github.com/Sun-Lab-NBB/ataraxis-micro-controller) library to implement the hardware
modules used by data acquisition systems developed in the Sun lab. This project is explicitly designed and built
to work with the hardware made and used in the lab, and will likely not work in other contexts without extensive
modification.

Note; the rest of this ReadMe assumes familiarity with the procedures, experiments, and tools used in the Sun lab
to acquire scientific data. See our [publications](https://neuroai.github.io/sunlab/publications) before reading
further, if you are not familiar with the work done in the lab.

Currently, all data acquisition systems can use up to three distinct classes of microcontrollers:
AMC-ACTOR, AMC-SENSOR, and AMC-ENCODER. The Actor is used to interface with the hardware modules that control the
experiment environment, for example, to deliver water, lock running wheel, and activate Virtual Reality screens. The
Sensor is used to monitor most data-acquisition devices, such as torque sensor, lick sensor, and Mesoscope frame
timestamp sensor. The Encoder uses hardware interrupt logic to monitor the animal’s movement using a rotary encoder and,
due to interrupt logic constraints, requires to be segmented into a separate class of microcontrollers. Using this
combination of microcontrollers maximizes the data acquisition speed while avoiding communication channel overloading.
Typically, each acquisition system would use at most a single instance of each microcontroller type.

This project contains both the schematics for assembling the microcontrollers used in the lab and the code that runs on
these microcontrollers. The hardware created and programmed as part of this project is designed to be interfaced through
the bindings available from the [sl-experiment](https://github.com/Sun-Lab-NBB/sl-experiment) library, which is a core
dependency for every Sun lab project.

___

## Table of Contents

- [Dependencies](#dependencies)
- [Hardware Assembly](#hardware-assembly)
- [Software Installation](#software-installation)
- [Usage](#usage)
- [API Documentation](#api-documentation)
- [Versioning](#versioning)
- [Authors](#authors)
- [License](#license)
- [Acknowledgements](#Acknowledgments)

___

## Dependencies

### Main Dependency
- [Platformio](https://platformio.org/install) IDE to upload the code to each microcontroller.

### Additional Dependencies
These dependencies will be automatically resolved whenever the library is installed via Platformio.

- [digitalWriteFast](https://github.com/ArminJo/digitalWriteFast).
- [Encoder](https://github.com/PaulStoffregen/Encoder).
- [ataraxis-micro-controller](https://github.com/Sun-Lab-NBB/ataraxis-micro-controller)
- [ataraxis-transport-layer-mc](https://github.com/Sun-Lab-NBB/ataraxis-transport-layer-mc).

___

## Hardware Assembly

To assemble the microcontroller hardware, consult the
[schematics and instructions](https://drive.google.com/drive/folders/12gDWwI_88usMgt7qVo7e83FKYo45KZwz?usp=drive_link)
reflecting the latest state of our microcontroller hardware.

Note, the provided link only covers the microcontrollers and does not discuss the assembly of other
experiment-facilitating devices, used by each data acquisition system. Consult the main
[sl-experiment](https://github.com/Sun-Lab-NBB/sl-experiment) library or previous lab publications for details on
assembling other data acquisition system components.

___

## Software Installation

1. Download this repository to a local PC with direct USB access to the microcontrollers. Use the latest
   stable release from [GitHub](https://github.com/Sun-Lab-NBB/sl-micro-controllers/releases), as it always
   reflects the current state of our data acquisition hardware.
2. Open the project in the 'Platformio' IDE.
3. Optionally disable all hardware modules not used by the target acquisition system. This project is intended to be
   reused by all acquisition systems, so it contains all hardware modules supported in the lab.
4. Connect a ***single*** microcontroller to the host PC and select the microcontroller type by modifying the
   preprocessor directive on line 26 of the [main.cpp](src/main.cpp). Do ***NOT*** connect more than a single controller
   at a time, as some systems have issues selecting the correct upload target otherwise.
5. After uploading the code, disconnect the microcontroller from the host-PC and connect the next microcontroller.
6. Repeat steps 3 and 4 until all microcontrollers are configured.
7. Connect all microcontrollers to the PC that will manage the data acquisition runtime (the main data-acquisition PC).

___

## Usage

Once the microcontrollers are assembled, configured, and connected to the main data acquisition PC, they can be
accessed via the [sl-experiment](https://github.com/Sun-Lab-NBB/sl-experiment) library.

___

## API Documentation

See the [API documentation](https://sl-micro-controllers-api-docs.netlify.app/) for the detailed description of
the methods and classes exposed by this library.

___

## Versioning

This project uses [semantic versioning](https://semver.org/). See the
[tags on this repository](https://github.com/Sun-Lab-NBB/sl-micro-controllers/tags) for the available project releases.

---

## Authors

- Ivan Kondratyev ([Inkaros](https://github.com/Inkaros))

---

## License

This project is licensed under the GPL3 License: see the [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- All Sun lab [members](https://neuroai.github.io/sunlab/people) for providing the inspiration and comments during the
  development of this library.
- The creators of all other dependencies and projects listed in the [platformio.ini](platformio.ini) file.

---
