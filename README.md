# NotSoOTA

![NotSoOTA Logo](assets/NotSoOTA.png)

## Introduction
Welcome to `NotSoOTA`, a library uniquely designed to enable updates for multiple ESP32 devices simultaneously via a serial connection, such as RS-485.  
This approach deviates from traditional wireless Over-the-Air (OTA) methods, offering a reliable and efficient alternative for updating numerous devices in sync, especially in environments where wireless updates are impractical.

## Development Status
`NotSoOTA` is currently in the development phase. We have not reached a stable release yet, and the library is evolving with ongoing efforts to enhance features and ensure robust performance.

## Collaboration
Contributions from the community are highly encouraged and appreciated! Whether it's through code contributions, documentation improvements, or offering suggestions, your involvement is invaluable.   Feel free to fork the repository, submit pull requests, or open issues to discuss new ideas and report bugs.

## About ESP32 OTA and Partition Requirements
OTA (Over-the-Air) traditionally refers to the wireless update mechanism for devices, but `NotSoOTA` introduces a novel approach using serial connections for the ESP32.  

For OTA updates on the ESP32, specific partition configurations are necessary. The ESP32 should have at least two OTA app partitions (OTA_0 and OTA_1) and one for the OTA data.   
This setup allows one partition to run the current firmware and another to store the downloaded new firmware.  
After the download, the ESP32 can switch between these partitions to activate the new firmware, ensuring that there's a fallback option in case of any issues with the new update.

