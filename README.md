# SUM open-source code

This repository contains the SuM open-source code for testing and developing purposed.
The featueres it provides are summarized as follows:

* access DAC/ADC
* perform DSP (with Janus)
* transmit and receive
* JANUS medium access (ANEP-87 §2.2): carrier sensing with Global Awareness and
  binary exponential backoff before each transmission (`--csma 1`, the default)
* emergency messages: a payload starting with `SOS` skips the medium access;
  `SOS lat lon [depth speed heading nationality]` is sent as a class 0
  Emergency Position packet (`janus_executable --help` for the details)
* control the SuM's tx/rx switch via GPIO.


## Installer

An `installer.sh` script is provided with the following options:

```
OPTIONS:
   -h    Show this message
   -p    Prefix Path
   -c    clean repository

   e.g. $0 -p <the_current_path>
```
So, following our example and enabling tests you will have

```
./installer.sh -p <the_current_path>
```

If you want to clean repository

```
./installer.sh -c
```


## Dependencies
In addition to base-object repo, this module depends on:
* libasound2
```
sudo apt-get install libasound2
```
* libgpiod-dev
```
sudo apt install libgpiod-dev
```
* janus and janus plugins (download it from https://www.januswiki.com/ and install it to the system using the default path).
  Text messages (class 11) and emergency positions (class 0) need the plugins
  `plugin_011_02` and `plugin_000_01` of the janus-c fork (branch `sum-integration`).