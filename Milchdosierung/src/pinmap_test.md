# Pinmapping am NUCLEO-F446RE testen

Der Test verwendet deine neue Belegung aus `include/MilkMatePinMap.h`.
`src/main.cpp` und das ältere `PESBoardPinMap.h` gehören weiterhin zur normalen
Anwendung. Die Testumgebung kompiliert ausschliesslich `src/pin_test/main.cpp`.

Alle belegten Schnittstellen bleiben während der gesamten Prüfung initialisiert.
Die Busprüfungen laufen nacheinander; die fünf PWM-Signale, ihre Rückmessungen,
TIM2 und der I²C-Slave laufen auch während der Busprüfungen. Nach jeder Runde
werden GPIO-Modus, Alternate Function und Pull-Widerstände erneut gelesen.
Doppelte Pinzuweisungen werden bereits beim Kompilieren abgewiesen.

## 1. Aufbau

Benötigt: **19 Drahtbrücken und zwei 4,7-kΩ-Widerstände**, optional ein Oszi.

**Nur das nackte Nucleo mit den Testbrücken verwenden.** PES-Board, Motorendstufen,
Motoren, Raspberry Pi, Display, Flowmeter und LED-Streifen abziehen. Der Test
schaltet auch die Enable-Ausgänge HIGH. Versorgung über den ST-Link-USB-Anschluss.
Im ausgeschalteten Zustand verdrahten. Alle Signale arbeiten mit 3,3 V.

Die Positionen sind **ST-Morpho CN7/CN10**, keine Arduino-Pinnummern. Von oben auf
die Bauteilseite schauen; Pin 1 anhand der Boardmarkierung bzw. des Handbuchs
identifizieren. Die zwei Reihen zählen ungerade/gerade: 1/2, 3/4, …, 37/38.
Die folgenden Verbindungen bleiben für alle automatischen Tests gleichzeitig gesteckt.

### Busse und PWM

| Quelle | Ziel | Prüfung |
| --- | --- | --- |
| PA0 **CN7-28** | PA1 **CN7-30** | UART4 TX → RX |
| PC12 **CN7-3** | PC11 **CN7-2** | SPI3 MOSI → MISO |
| PC10 **CN7-1** | PB8 **CN10-3** | SPI3-SCK auf separatem Eingang mitzählen |
| PB7 **CN7-21** | PC9 **CN10-1** | I²C1 SDA ↔ I²C3 SDA |
| PB6 **CN10-17** | PA8 **CN10-23** | I²C1 SCL ↔ I²C3 SCL |
| PB15 **CN10-26** | PC0 **CN7-38** | Motor 1 PWM → Fehler-Eingang 1 |
| PA9 **CN10-21** | PC1 **CN7-36** | Motor 2 PWM → Fehler-Eingang 2 |
| PB13 **CN10-30** | PC2 **CN7-35** | Motor 3 PWM → Fehler-Eingang 3 |
| PA11 **CN10-14** | PC3 **CN7-37** | Motor 4 PWM → Fehler-Eingang 4 |
| PC8 **CN10-2** | PA15 **CN7-17** **und** PA10 **CN10-33** | PWM → TIM2-Pulszähler und separater PWM-Messeingang; eine Quelle auf zwei Eingänge |

An die zusammengeschaltete **SDA-Leitung einen 4,7-kΩ-Pull-up** nach **3V3
CN7-16**, ebenso an die **SCL-Leitung einen zweiten 4,7-kΩ-Pull-up** nach CN7-16.
Insgesamt zwei Widerstände; die internen Pull-ups werden für I²C nicht verwendet.
SDA/SCL nicht kreuzen. Auf einer gemeinsamen Platine ist keine zusätzliche
Massebrücke erforderlich. Oszi-Masse z. B. an **GND CN7-20** anschliessen.

### GPIOs

Die vier Taster-Eingänge werden von den vier Enable-Ausgängen angesteuert.
Vier weitere, im Produktmapping freie Pins dienen als Hilfseingänge.

| Bit in GPIO-Testmeldung | Ausgang | Eingang |
| --- | --- | --- |
| 0 | PB14 **CN10-28**, Enable 1 | PA4 **CN7-32**, Taster 1 |
| 1 | PB10 **CN10-25**, Enable 2 | PA6 **CN10-13**, Taster 2 |
| 2 | PC4 **CN10-34**, Enable 3 | PA7 **CN10-15**, Taster 3 |
| 3 | PC5 **CN10-6**, Enable 4 | PB12 **CN10-16**, Taster 4 |
| 4 | PB9 **CN10-5**, Motoren DIR | PB0 **CN7-34**, Hilfseingang |
| 5 | PC7 **CN10-19**, Display CS | PB1 **CN10-24**, Hilfseingang |
| 6 | PC6 **CN10-4**, Display D/C | PB4 **CN10-27**, Hilfseingang |
| 7 | PB2 **CN10-22**, Display RESET | PB5 **CN10-29**, Hilfseingang |

**Keine echten Taster parallel zu diesen Testbrücken anschliessen:** Ein Taster
nach GND könnte sonst einen HIGH geschalteten Enable-Ausgang kurzschliessen.
Optional je 1 kΩ in Serie in die Brücken von Push-pull-Ausgängen einfügen.
Die I²C-Verbindungen direkt stecken und die oben genannten Pull-ups verwenden.

**Board-Lötbrücken:** CN7-36 und CN7-38 müssen PC1 bzw. PC0 führen. Laut UM1724
sind dafür SB56/SB51 geschlossen und SB46/SB52 offen. Eine umgebaute
Arduino-I²C-Belegung kann hier stattdessen PB9/PB8 auflegen und mit diesem Aufbau
kollidieren. Vor dem Einschalten kontrollieren; bei einem unveränderten Board ist
normalerweise keine Lötarbeit nötig.

**PA2/PA3 nicht miteinander verbinden.** UART2 dient über ST-Link als Konsole;
eine direkte Schleife könnte mit dem ST-Link-TX konkurrieren. PA2 ist dabei
STM32-TX → ST-Link-RX, PA3 ist STM32-RX ← ST-Link-TX. SWD an PA13/PA14 bleibt
für Upload und Debugging reserviert; dort keine Testbrücken anbringen.

## 2. Starten in VS Code / PlatformIO

1. Im PlatformIO-Projekt die Umgebung **pinmap_test** auswählen.
2. Unter **Project Tasks → pinmap_test → General** zuerst **Build**, dann **Upload**.
3. **Monitor** für diese Umgebung öffnen: **115200 Baud, 8N1**, ST-Link Virtual COM Port.
4. Gegebenenfalls den schwarzen RESET-Taster drücken, um die Startmeldung zu sehen.

Alternativ im PlatformIO-Terminal:

```text
pio run -e pinmap_test
pio run -e pinmap_test -t upload
pio device monitor -e pinmap_test
```

Bei mehreren COM-Ports beim Monitor `--port COMx` ergänzen.
Zum Zurückwechseln zur Anwendung ausdrücklich **nucleo_f446re** auswählen bzw.
`pio run -e nucleo_f446re -t upload` ausführen. Vor deren Start die Testbrücken entfernen.

## 3. Erwartete Ergebnisse

Der Test wiederholt sich automatisch. Mindestens zehn Runden beobachten. Jede
automatische Gruppe soll `PASS` melden; `total failed groups=0` muss dabei bleiben.
`AUTO PASS` bezieht sich nur auf die automatischen Gruppen. Zusätzlich:

- Ein Zeichen im seriellen Monitor senden: `Console RX received ...` bestätigt
  auch die Richtung PC → ST-Link → PA3. Falls nötig im Monitor Enter drücken.
- Den blauen B1-Taster drücken und loslassen, jeweils ca. eine Sekunde halten:
  `B1 PC13=...` muss beide Zustände anzeigen und `press/release=seen` erscheinen.
- Die grüne User-LED **LD2 an PA5** muss blinken. In Mbed heisst ihr Alias `LED1`.
  Diesen optischen Test selbst protokollieren.

| Funktion | Automatische Prüfung / Sollwert |
| --- | --- |
| UART4 | 64 wechselnde Bytes pro Runde zurücklesen, 115200 8N1; Timeout bei fehlender Brücke |
| SPI3 | 64 wechselnde Bytes zurücklesen, Mode 0, 100 kHz angefordert; **512 externe SCK-Flanken** |
| I²C | I²C1 Master, I²C3 Slave, 100 kHz, Adresse **0x32 (7 Bit)**; vier Pakete à 16 Bytes schreiben und eine mit XOR 0xA5 veränderte Antwort zurücklesen |
| GPIOs | 20 Muster: alle LOW/HIGH, einzelne HIGH/LOW, 0x55/0xAA; prüft auch Vertauschungen der acht Brücken |
| Motor-PWM | Alle vier gleichzeitig **1 kHz**; ungerade Runde: HIGH-Anteile **20 / 35 / 60 / 80 %**, gerade Runde: **75 / 55 / 40 / 25 %** |
| PC8 | **2 kHz, 40 % HIGH**, Rückmessung über PA10 |
| PA15 | **TIM2_CH1 als externer Hardware-Zähler**; ca. 2000 steigende PC8-Flanken pro Sekunde |
| Pinmux | Modus, AF, Pull-up/Pull-down; bei AF-Pins auch Open-drain prüfen, vor und nach Busverkehr |

Die PWM-Rückmessung benutzt GPIO-Flankeninterrupts auf PC0…PC3 und PA10.
Sie toleriert 5 % Periodenabweichung und 8 Prozentpunkte Tastgradabweichung wegen
Interruptlatenzen unter Buslast. Das Oszi kann genauer messen. SPI-SCK wird real
auf PB8 mitgezählt: Eine reine MOSI/MISO-Schleife würde einen fehlenden externen
Taktpfad allein nicht nachweisen. Die tatsächlich erreichbare SPI-Taktrate kann
wegen des Hardware-Teilers vom angeforderten Wert abweichen.

## 4. Auffälligkeiten einordnen

- **Motor-PWM:** Alle vier Kanäle verwenden **TIM1**, mit gemeinsamem Prescaler
  und Periodenregister. Unterschiedliche Tastgrade sind möglich; unterschiedliche
  Frequenzen auf diesen vier Pins gleichzeitig nicht. `period_us()` später an
  einem Kanal zu ändern beeinflusst die anderen Kanäle.
- **PB15 und PB13:** TIM1_CH3N und TIM1_CH1N sind komplementäre Ausgänge. Der hier
  installierte Mbed-STM32-Treiber gibt bei `PwmOut::write(x)` dort den HIGH-Anteil
  `1-x` aus. Der Test kompensiert das absichtlich. Das muss auch der spätere
  Motortreiber berücksichtigen; `write(0)` bedeutet dort nicht LOW.
- **I²C:** Zwei Master, die nur Daten senden, ergeben keinen Loopback. Deshalb
  läuft I²C3 in einem separaten Mbed-Thread als Slave und liefert eine echte Antwort.
  Beide Datenrichtungen werden geprüft; der Rollenwechsel zu I²C3 als Master
  wird in diesem Test nicht geprüft. Bei FAIL zuerst Pull-ups, SDA/SCL und Brücken prüfen.
- **GPIO FAIL:** `xor` markiert die fehlerhaften Bits gemäss der GPIO-Tabelle.
  Beispielsweise bezeichnet Bit 2 die Schleife PC4 → PA7.
- **PIN FAIL:** Die Ausgabe enthält Ist/Soll für Modus, AF, Pull und Open-drain.
  Das deutet auf eine abweichende Initialisierung oder einen überschriebenen Pin hin.
- **PC8 / WS2811:** Hier wird die Timer-/Pin-Funktion mit normaler PWM geprüft.
  Ein WS2811-Datentelegramm mit passenden Bitzeiten, Resetpause und ggf. Pegelwandler
  ist eine spätere separate Treiberprüfung. Der Test weist keine LED-Protokollfunktion nach.

Ein bestandener Durchlauf bestätigt die geprüfte Konfiguration und Verdrahtung
bei diesen Testraten. Er ersetzt nicht den Integrationstest mit den späteren
Treibern, realen Geräten, deren Zielraten und Interrupt-/DMA-Last.

## Quellen

- [ST UM1724, Nucleo-64 MB1136](https://www.st.com/resource/en/user_manual/um1724-.pdf):
  Tabelle 29 (CN7/CN10), Abschnitte 7.6/7.7 (LED/Taster), 7.10/7.11 (UART/Lötbrücken).
- [STM32F446xC/E Datenblatt](https://www.st.com/resource/en/datasheet/stm32f446re.pdf):
  Alternate-Function-Belegung.
- Installiertes Mbed OS 6.17: `targets/TARGET_STM/pwmout_api.c`,
  `targets/TARGET_STM/i2c_api.c`, `targets/TARGET_STM/TARGET_STM32F4/us_ticker_data.h`
  (Mbed-Mikrosekundenticker verwendet TIM5) und
  `targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/TARGET_NUCLEO_F446RE/PeripheralPins.c`.
