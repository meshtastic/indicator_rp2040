<div align="center" markdown="1">

<img src=".github/meshtastic_logo.png" alt="Meshtastic Logo" width="80"/>

  <h1 align="center"> Indicator RP2040 Companion Firmware
</h1>
  <p style="font-size:15px;" align="center">This is a modified companion firmware based on <a href="https://github.com/Seeed-Solution/SenseCAP_Indicator_RP2040">SenseCAP_Indicator_RP2040</a> </p>



![GitHub Last Commit](https://img.shields.io/github/last-commit/meshtastic/indicator_rp2040)
[![CLA Assistant](https://cla-assistant.io/readme/badge/meshtastic/indicator_rp2040)](https://cla-assistant.io/meshtastic/indicator_rp2040)
[![Fiscal Contributors](https://opencollective.com/meshtastic/tiers/badge.svg?label=Fiscal%20Contributors&color=deeppink)](https://opencollective.com/meshtastic/)

</div>

## Features
This firmware turns the RP2040 into a peripheral bridge for the main Meshtastic CPU, exchanging framed protobuf messages over the interboard UART link:
- **GPS**: forwards NMEA sentences from the onboard GNSS module to the main CPU, and relays configuration back down to the module
- **I2C bridge**: runs read/write transactions and bus scans on the secondary I2C bus on behalf of the main CPU
- **SD card**: chunked file read and write, directory creation, removal and listing, free-space statistics, plus mount, eject and format on request, with detection of a card inserted or removed while running
- **Buzzer**: tone control on request

## Installation

### Download a prebuilt binary
Tagged releases publish a ready-to-flash `firmware.uf2` on the [Releases page](https://github.com/meshtastic/indicator_rp2040/releases), alongside an archive that also bundles the `.elf` and `.bin`. To flash it:
1. Put the device in **BOOTSEL** mode so it mounts as a USB drive. See [Seeed's wiki](https://wiki.seeedstudio.com/SenseCAP_Indicator_How_To_Flash_The_Default_Firmware/#flash-the-uf2-file) for how.
2. Drag and drop the `.uf2` onto that drive. The device reboots into the new firmware.

### Build from source
To build a development version with [PlatformIO](https://platformio.org/install):
1. Clone this repository.
2. Open a terminal in the repository directory.
3. Build the firmware:
   ```sh
   pio run -e seeed_indicator_rp2040
   ```
4. Flash the result with either:
   - PlatformIO's **Upload** command, with the RP2040's serial port selected, or
   - the compiled `.uf2` under the build directory, dragged onto the RP2040 USB drive while in **BOOTSEL** mode.

## License
This project is licensed under the **GNU General Public License v3.0 (GPL-3.0)**. See the `LICENSE` file for details.

## Stats

![Alt](https://repobeats.axiom.co/api/embed/511382d749b5dff56df6c312f3b454a5150f67ef.svg "Repobeats analytics image")
