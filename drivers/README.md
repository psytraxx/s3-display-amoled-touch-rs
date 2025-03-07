## Power Management Unit (BQ25896)

- High efficiency single-cell Li-Ion/Li-polymer battery charger
- Input voltage range: 3.9V to 14V
- Charging current up to 3A
- I2C programmable
- Battery temperature monitoring
- Multiple charging modes:
  - Pre-charge
  - Constant current
  - Constant voltage

## HLK-LD2410 24Ghz Human Presence Sensor

https://github.com/arendst/Tasmota/blob/development/tasmota/tasmota_xsns_sensor/xsns_102_ld2410.ino
https://www.hlktech.net/index.php?id=1094
https://github.com/iavorvel/MyLD2410/blob/master/src/MyLD2410.h

i have constant stream of bytes from serial interface - i am fetching them in 64byte chunks - i need to recognize a header and footer byte sequence and want the data inbetween - how do i do this with rust ?

### Generate documentation

    cargo doc -p bq25896x --no-deps  --open
