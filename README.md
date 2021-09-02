# ESP32 state controller firmware for the Open eXtensible Rack System

A binary state controller for DIY home automation projects.

Listens for MQTT messages and activates/deactivates binary output signals using MCP23017 I2C I/O buffers. 

Typically the output signals would be used to drive relay coils in order to switch larger voltages - e.g. light circuits.


## Outputs
### Configuration
Each OUTPUT can be configured by publishing an MQTT message to one of these topics;
```
[BASETOPIC/]conf/CLIENTID/[LOCATION/]INDEX/type
[BASETOPIC/]conf/CLIENTID/[LOCATION/]INDEX/lock
[BASETOPIC/]conf/CLIENTID/[LOCATION/]INDEX/time
```    
where;
- `BASETOPIC`:   Optional base topic if your topic structure requires it
- `CLIENTID`:    Client id of device, defaults to `usc-<MACADDRESS>`
- `LOCATION`:    Optional location if your topic structure requires it
- `INDEX`:       Index of the output to configure (1-based)
    
The message should be;
- `/type`:       One of `motor`, `relay` or `timer`
- `/lock`:       Output index to interlock with (lock the opposite for interlocking both ways)
- `/time`:       Number of seconds an output stays `on` when type set to `timer`
    
A null or empty message will reset the output to;
- `/type`:       `relay`
- `/lock`:       Unlocked
- `/time`:       60 seconds

A retained message will ensure the USC auto-configures on startup.

The only difference between `motor` and `relay` outputs is the interlock delay (if an interlock is configured). 

|Output Type |Interlock delay|
|------------|---------------|
|`motor`     |2000ms         |
|`relay`     |500ms          |

### Commands
Each OUTPUT can be controlled by publishing an MQTT message to the topic;
```
[BASETOPIC/]cmnd/CLIENTID/[LOCATION/]INDEX
```
where;
- `BASETOPIC`:   Optional base topic if your topic structure requires it
- `CLIENTID`:    Client id of device, defaults to `usc-<MACADDRESS>`
- `LOCATION`:    Optional location if your topic structure requires it
- `INDEX`:       Index of the output to update (1-based)
    
The message should be;
- `0` or `off`:  Turn output OFF (deactivate the relay)
- `1` or `on`:   Turn output ON (activate the relay)
  
A null or empty message will cause the current output state to be published. I.e. can be used to query the current state.

### Events
An output STATE CHANGE is reported to a topic of the form;
```
[BASETOPIC/]stat/CLIENTID/[LOCATION/]INDEX
```
where; 
- `BASETOPIC`:   Optional base topic if your topic structure requires it
- `CLIENTID`:    Client id of device, defaults to `usc-<MACADDRESS>`
- `LOCATION`:    Optional location if your topic structure requires it
- `INDEX`:       Index of the output causing the event (1-based)

The message is a JSON payload of the form; 
```
{"index":6, "type":"relay", "event":"on"}
```
where;
- `type`:        One of `motor`, `relay` or `timer`
- `event`:       One of `on` or `off`

### Interlocking
Interlocking two outputs allows them to control equipment such as roller blinds, garage doors, louvre roofing etc.

However if you are planning to control a motor of any sort then it is important that the two outputs are configured as type `motor` and that both are interlocked with each other. This is to ensure that both outputs will not be commanded to operate at the same time and adds a 2 second delay between any changes of output.

Example configuration if using outputs 4 & 5 to control a set of roller blinds;
```
conf/usc-abcdef/4/type motor
conf/usc-abcdef/5/type motor
conf/usc-abcdef/4/lock 5
conf/usc-abcdef/5/lock 4
```

The operation of the interlocked outputs should be verified before connecting to any external equipment. External interlocking equipment may be required for some equipment. Most importantly, the manufacturers wiring and installation guides must be adhered to.


## Hardware
This firmware is compatible with;

- https://bmdesigns.com.au/shop/relay128-128-channel-relay-driver/
- https://bmdesigns.com.au/shop/relay16-16-channel-relay-driver/

And is designed to run on the [Universal Rack Controller](https://github.com/SuperHouse/URC) (URC).


## Credits
 * Jonathan Oxer <jon@oxer.com.au>
 * James Kennewell <James@bmdesigns.com.au>
 * Ben Jones <https://github.com/sumnerboy12>


## License
Copyright 2020-2021 SuperHouse Automation Pty Ltd  www.superhouse.tv  

The software portion of this project is licensed under the Simplified
BSD License. The "licence" folder within this project contains a
copy of this license in plain text format.
