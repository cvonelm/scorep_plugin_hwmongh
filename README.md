# Score-P hwmon GraceHopper Plugin

This code provides access to the Power Sensors found on the GraceHopper superchip

## Prerequisites

- Score-P 

## Installation


```
git clone --recursive-submodules git@gitlab.dkrz.de:eeclips/scorep_plugin_hwmongh.git
cd scorep_plugin_hwmongh
mkdir build & cd build
cmake ../
make

#copy the resulting libhwmongh_plugin.so into your LD_LIBRARY_PATH
```

## Usage

```
export SCOREP_METRIC_PLUGINS=hwmongh
export SCOREP_METRIC_HWMONGH_PLUGIN="hwmon1,hwmon2,hwmon3,hwmon4"
```

The plugin exports four metrics (See: [NVIDIA Grace Performance Tuning Guide -- Power Telemetry](https://docs.nvidia.com/grace-perf-tuning-guide/power-thermals.html#power-telemetry))

- hwmon1 ("Module Power Socket 0")
- hwmon2 ("Grace Power Socket 0")
- hwmon3 ("CPU Power Socet 0")
- hwmon4 ("SysIO Power Socket 0")
