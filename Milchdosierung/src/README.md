# Nucleo-F446RE Pinmapping-Test

Ein PlatformIO-Testprogramm fuer STM32Cube/HAL. In PlatformIO die Umgebung **`nucleo_f446re_pin_test`** verwenden; sie ist als Standardumgebung gesetzt. Die bestehende Mbed-Umgebung bleibt fuer das Hauptprojekt erhalten, ist fuer dieses HAL-Testprogramm aber nicht geeignet, da Mbed eigene HAL-I2C-Callbacks bereitstellt. Die serielle Kommandozeile laeuft ueber USART2 und den ST-LINK-Virtual-COM-Port.

## Start in VS Code / PlatformIO

1. Das Projekt `Milchdosierung` in VS Code oeffnen.
2. In PlatformIO die Umgebung **`nucleo_f446re_pin_test`** waehlen (sie ist als Standard gesetzt).
3. Nucleo-F446RE per USB am ST-LINK anschliessen.
4. **Build** und danach **Upload** ausfuehren.
5. Den PlatformIO Serial Monitor mit **115200 Baud, 8N1** oeffnen.
6. Nach Reset muss `=== PINTEST-HAL / NUCLEO-F446RE ===` erscheinen, gefolgt von `nucleo>`. `help` eingeben und die Antwort abwarten, bevor der naechste Befehl gesendet wird.
7. Vor jedem Verdrahtungsschritt fordert das Programm `go` an.

Die Zeilenendung im Monitor sollte LF sein. Der Monitor benutzt PA2/PA3 (USART2), die bei diesem Nucleo fuer den ST-LINK VCP vorgesehen sind.
Eingegebene Zeichen werden zurueckgespiegelt. Falls nach Reset nur eine andere oder unvollstaendige Startmeldung erscheint, ist wahrscheinlich eine andere PlatformIO-Umgebung oder ein anderer COM-Port aktiv.

## Befehle

| Befehl | Pruefung |
| --- | --- |
| `map` | CN-Pin-Uebersicht |
| `gpio` | DIR-, Enable- und optionale Display-Steuerpins gegen PA4/CN7-32 |
| `spi3` | SPI3 sendet Daten auf MOSI und vergleicht den Ruecklauf auf MISO |
| `uart4` | UART4 sendet Bytes von PA0/TX und vergleicht sie auf PA1/RX |
| `i2c1` | I2C1 als Master, I2C3 als Slave; schreibt und liest Testbytes |
| `i2c3` | I2C3 als Master, I2C1 als Slave; Gegenrichtung |
| `pwm` | TIM1-Ausgaenge und PC8/TIM3 einzeln als 1-kHz-/50-%-Signal |
| `flow` | TIM1-Test-PWM an PA15/TIM2_CH1; Frequenz wird per Input Capture gemessen |
| `inputs` | Eingangspegel live anzeigen; mit `q` beenden |
| `led` | User LED an PA5 blinken lassen |
| `all` | GPIO, SPI, UART4, I2C1/3, PWM und Flow nacheinander ausfuehren |

## Jumper und Messpunkte

| Test | Jumper / Messung |
| --- | --- |
| `gpio` | Jeweils den im Monitor genannten Ausgang mit **PA4/CN7-32** verbinden. Zwischen den Schritten den Jumper umstecken. |
| `spi3` | **PC12/MOSI/CN7-3** nach **PC11/MISO/CN7-2**; **PC10/SCK/CN7-1** bei Bedarf mit Oszi messen |
| `uart4` | **PA0/TX/CN7-28** nach **PA1/RX/CN7-30** |
| `i2c1` und `i2c3` | **PB7/SDA/CN7-21** nach **PC9/SDA/CN10-1**, sowie **PB6/SCL/CN10-17** nach **PA8/SCL/CN10-23**. Beide Jumper fuer beide Richtungen stecken lassen. |
| `pwm` | Jeden im Monitor genannten PWM-Ausgang nach **PA4/CN7-32** fuer die digitale Rueckmeldung; alternativ mit dem Oszilloskop am genannten Ausgang messen. |
| `flow` | **PA9/TIM1_CH2/CN10-21** nach **PA15/TIM2_CH1/CN10-11** |
| `inputs` | Eingangs-Pins einzeln auf GND und 3V3 legen oder Taster betaetigen. PC0-PC3 bekommen im Test Pull-downs; PA4/PA6/PA7/PB12 bekommen Pull-ups. Die User-Taste PC13 ist aktiv LOW. |

Bei `i2c1`/`i2c3` werden die beiden I2C-Controller des STM32 miteinander verbunden. Ein Controller sendet, der andere antwortet als Slave; anschliessend werden die Antwortdaten byteweise verglichen. Das Programm setzt fuer diesen Test interne Pull-ups und 10 kHz. Externe Module waehrenddessen abziehen.

USART2 ist die serielle Kommandozeile selbst: deren RX/TX-Pfad ueber PA2/PA3 und ST-LINK VCP wird durch die Bedienung laufend benutzt. Ein zusaetzlicher TX-RX-Jumper wuerde die Kommandos stoeren. Beim SPI-Test erzeugt der Controller SCK; fuer einen elektrischen SCK-Pin-Nachweis den genannten Pin mit dem Oszilloskop pruefen.

## Wichtige Hinweise

- Vor dem Test externe Motor-Treiber, Motoren, Display, Raspberry Pi und sonstige Module abziehen. Besonders der Motoren-Enable-Pin PB9 wird beim GPIO-Test geschaltet.
- Jumper nur zwischen den genannten MCU-Pins setzen. Keine Ausgaenge mit einer externen Versorgung verbinden; fuer Eingangstests nur GND oder **3V3**, niemals 5 V, verwenden.
- Das PWM-Testsignal ist 1 kHz mit ungefaehr 50 % Tastgrad. Der Test auf PC8 prueft TIM3_CH3 als Pin-/Timer-Ausgang, sendet aber kein WS2811-Protokoll.
- PA2/PA3 bleiben waehrend des Tests die serielle Kommandozeile. PA13/PA14 bleiben fuer SWD reserviert.
- In der Tabelle ist kein MCU-GPIO doppelt vergeben. PA4 wird waehrend GPIO-/PWM-Tests temporaer als Rueckmelde-Eingang verwendet. USART2 auf PA2/PA3 ist absichtlich gleichzeitig die serielle Konsole. Die vier Motor-PWM-Pins liegen auf TIM1; sie koennen unterschiedliche Tastgrade, aber nur dieselbe Timerfrequenz verwenden. PC8/TIM3 ist davon unabhaengig.
