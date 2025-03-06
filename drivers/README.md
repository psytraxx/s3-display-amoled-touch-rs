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

https://www.hlktech.net/index.php?id=988
https://github.com/arendst/Tasmota/blob/development/tasmota/tasmota_xsns_sensor/xsns_102_ld2410.ino


### Generate documentation

    cargo doc -p bq25896x --no-deps  --open
