# reaction_monitor
An Arduino-based data logger for logging temperature, gas flow, and gas composition data from chemical reactions.

## Motivation

Establishing meaningful trends and building physically meaningful models based on the analysis of nanomaterials requires that those materials be synthesized under carefully controlled, reproducible conditions. In many cases, however, the true reaction conditions are far more complex than is easily conveyed in the simplified narrative form of a publication experimental data section. In our initial trials of standardizing our raw data formats we isolated several key parameters of interest for real-time monitoring: reaction temperature, gas flow rate, and gas composition (for N<sub>2</sub>/O<sub>2</sub> mixtures). While such systems are likely common at scale, we sought to use readily available components (many of which are common in academic research labs or are otherwise low-cost) to automate data logging in a varatile and nominally modular fashion that would be amenable to a small-scale, rapidly changing research environment.

## Overview

<p align="center">
    <img src="assets/rxn_monitor_setup.JPG" alt="reaction monitor setup" width="30%" height="30%">
</p>

The heart of the reaction monitor is an Arduino Mega. The Arduino communicates with an [Omega PID controller](https://www.omega.ca/en/control-monitoring/controllers/pid-controllers/p/CN7200-Series) via its MODBUS serial interface. [Gas flow](https://www.mcmflow.com/product/model-100/) and [oxygen sensors](https://www.apogeeinstruments.com/so-110-soil-response-thermistor-reference-oxygen-sensor/) output 0 - 3.3 V signals which are read by [16-bit ADCs](https://www.adafruit.com/product/1085) and sent to the Arduino via the I2C protocol. 

Two oxygen sensors are available and are intended to be put before and after the reaction. In this initial implementation, a pure or mixed gas is bubbled throiugh through a solution containing the reactive species, such that the first oxygen sensor (OXY1) is calibrated under dry conditions and the second (OXY2) is calibrated under wet conditions. The Arduinio logs raw voltages from the sensors in addition to the calculated oxygen percentages, so oxygen percentages can always be recalculated during data workup (as is done in the [example data workup](sample_data_workup.ipynb)). The calibration procedure for the oxygen sensors are available in the [Apogee manual](assets/SO-100-200-manual.pdf), and our calibration data for both wet and dry conditions are available in [/calibration](calibration). The oxygen percentage calculations account for humidity. In our Arduino code, we assume 0% humidity for OXY1 as it measures a gas mixture directly from cylinders with dry gas. We placed a [temperature, pressure and humidty sensor](https://www.adafruit.com/product/2652) after OXY2 to calculate the post-reaction oxygen percentage. The humidity sensor is placed in a 3D printd enclosure available in [/materials](materials)

The electronics are housed in a 3D printed enclosure, also available in [/materials](materials). The enclosure includes two LCDs displaying basic information on oxygen percentages, gas flow rates, time, PID values (process value, set value, and output), and, when recording data, the name of the file being written. Data is recorded to a micro SD card. Recording is started/stopped by pressing the button on the front of the enclosure. When recording, a red LED will be lit and a blue LED will flash each time a data point is collected (which is every second in the current code).

<p align="center">
    <img src="assets/labeled_reaction_monitor.png" alt="labeled reaction monitor" width="50%" height="50%">
</p>

## Example Usage

The [example data workup notebook](sample_data_workup.ipynb) uses data file [RXN00026.CSV](sample_data/RXN00026.CSV). This data was collected during a reaction followed the procedure detailed in [Kirkpatrick _et al._ Size-Tunable Magnetite Nanoparticles from Well-Defined Iron Oleate Precursors, Chemistry of Materials, 2022](https://pubs.acs.org/doi/10.1021/acs.chemmater.2c02046) ([free copy here](https://drive.google.com/file/d/1hveF1zS2nrNvON7UYBfMMZ2yYlZfvH2g/view)). In short, the synthesis of magnetite nanoparticles uses a reduced pressure atmosphere, then an N<sub>2</sub> atmosphere during the initial heating of the reaction mixture. Once the reaction is under reflux a small amount of oxygen is introduced into the gas mixture. The size, shape, and presence/absence of core / shell structures involving wustite and magnetite can be affected by precise control of the gas flow, composition, and temperature profile. The reaction monitor records every aspect of these reaction conditions, including any user-intervention with the PID controller.

<p align="center">
    <img src="sample_data/RXN00026.png" alt="sample reaction data workup" width="70%" height="70%">
</p>

## Future Work

While our primary goals were to provide automated data logging, initial data indicates the possibility of automating key decision points in chemical reactions, thus eliminating hidden sources of uncontrolled human intervention. In the usage example above, our results indicate that the timing of introducing O<sub>2</sub> into the reaction affects the phase purity of the final particles. Presently, O<sub>2</sub> is introduced when the chemist running the reaction visually detects the onset of solvent reflux. The data provided by the reaction monitor has the potential to solidify this association between O<sub>2</sub>, reflux, and particle quality and allow us to begin the process of optimizing an automatic trigger for flow adjustment. In this specific example, the trigger would be a function of the slope of the output tracking temperature control. This data corresponds to the power being used toward resistive heating (on a 0-100 scale). In the sample data, the start of reflux can be seen at the point where output is persistently at 100 and the process value (PV) is relatively constant and unable to reach the set value (SV) which is chosen to correspond to a temperature above the reflux temperature of the solution. Implementing this change would require replacing the passive gas sensors with mass flow controllers, allowing for both monitoring and control of the reaction conditions.
