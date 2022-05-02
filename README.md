# ESP8266 based BME280 reader (atmospheric sensor)

This project is based on an ESP8266 module salvaged from a damaged WiFi outlet: WFG-1 (OBI Germany):

<img src=resources/WFG-1.jpg width=250>

It containes an OW8266 based MCU module which I have used to build a battery operated BME20 reader IoT device
that will push temperature, rel. humidity and atmospheric preasure vaules to a Blynk-Cloud:

<img src=resources/Ctrl-Module_Tags.jpg width=250>

The code users the ESP.deepSleep(MEASURE_INTERVAL , WAKE_NO_RFCAL) function and a static IP address to save as much power as pssible.

Since the MESURE_INTERVAL is not accurate due to the RTC I have setteled for a value of 460 sec. which will yield aprox. 5 min. 39 sec.
between measurements. Longer intervals will extend the battery running time before the protection circuit shuts down.

The device is beeing powered by a 18650 Li ion battery sitting on a Geekcreit baterry shield:

<img src=resources/18650-Battery-Shield.jpg width=280>

The atmospheric sensor used is the BME280:

<img src=resources/BME280.jpg width=280>

It is wired in 2-Wire mode:
|MCU       |  GPIO  |BME280|
|:---------|:------:|:----:|
|RELAY ON  |IO12    |PWR   |
|NETWORK   |IO4     |SCK   |
|RELAY OFF |IO5     |SDI   |


