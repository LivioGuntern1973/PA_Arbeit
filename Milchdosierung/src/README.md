# Automatischer Pin-Mapping-Test fuer NUCLEO-F446RE

Das Projekt initialisiert die Pins aus dem Pinmapping und laeuft danach automatisch. Es ist fuer das nackte Nucleo ohne angeschlossene Motoren, Flowmeter, Display oder sonstige Module gedacht.

## Ablauf

- Motor-PWM auf TIM1 wird mit 1 kHz und vier unterschiedlichen Tastverhaeltnissen ausgegeben. Die Tastverhaeltnisse wechseln etwa alle 5 Sekunden.
- Die vier DIR-Ausgaenge wechseln etwa einmal pro Sekunde. PB9 Motor Enable bleibt im inaktiven Pegel.
- PC8/TIM3_CH3 gibt ein 1-kHz-PWM-Signal aus. Das ist nur ein Pin-/AF-Test und kein WS2811-Datenprotokoll.
- UART4 und USART6 senden regelmaessig Testtexte. Auf den RX-Leitungen werden eingehende Bytes abgefragt.
- Beide I2C-Interfaces senden regelmaessig eine Adressphase fuer 7-Bit-Adresse 0x20. Ohne Slave ist NACK normal. Es wird kein Register beschrieben.
- PA5/TIM2_CH1 ist als Input Capture fuer den Flowmeter-Pulszaehler konfiguriert.
- PC0-PC3 werden als Fehler-Eingaenge gelesen. PA4, PA6, PA7 und PB12 werden mit Pull-up gelesen.
- Statusmeldungen erscheinen ueber Mbed-stdio auf dem ST-Link-Virtual-COM-Port mit 115200 Baud. PA13/PA14 bleiben fuer SWD unangetastet.

## PlatformIO starten

In VS Code kannst du **Build** und danach **Upload** in der PlatformIO-Leiste ausfuehren. Alternativ im Projektordner:

```text
pio run
pio run -t upload
```

Oeffne danach den ST-Link-Seriellport mit 115200 Baud, 8N1. Es gibt keine Befehle; der Test laeuft von selbst.

## Was ohne Zusatzhardware zu erwarten ist

- UART-TX wird gesendet, UART-RX bleibt normalerweise leer.
- I2C meldet NACK/keinen Slave. Das ist ohne Module kein Fehler.
- Der Flowmeter-Zaehler bleibt normalerweise bei 0. PA5 kann wegen der eingebauten Board-LED LD2 Pegel/Flanken zeigen.
- Fault-Eingaenge PC0-PC3 koennen ohne angeschlossene Treiberausgaenge undefinierte Pegel zeigen.
- PWM- und DIR-Pegel werden erzeugt, sind ohne Messgeraet aber nicht sichtbar. Das Programm bestaetigt hier nur, dass Mbed die Objekte und Timer initialisiert und der Task weiterlaeuft.

Die I2C-Leitungen haben auf einem nackten Board keine externen Pull-ups. Der Test setzt interne Pull-ups und verwendet 10 kHz, damit die unbelasteten Leitungen fuer diesen Initialisierungstest einen Ruhepegel bekommen. Diese Einstellung ersetzt keine korrekt dimensionierten externen Pull-ups fuer spaetere Hardware.

## Vor spaeterem Anschluss von Motoren

`MOTOR_ENABLE_ACTIVE_LEVEL` ist standardmaessig `1` (active-high), PB9 startet daher LOW. Falls dein Treiber active-low ist, ergaenze in `platformio.ini`:

```ini
    -DMOTOR_ENABLE_ACTIVE_LEVEL=0
```

Vor dem Anschluss der Motorversorgung die Polaritaet mit dem Datenblatt des Treibers abgleichen. PB13/TIM1_CH1N und PB15/TIM1_CH3N sind komplementaere PWM-Ausgaenge; die effektive Signalpolaritaet sollte spaeter am realen Aufbau geprueft werden.

## Korrekturen zum Pinout

- PA5 liegt auf **CN10-11**, nicht CN7-28. CN7-28 ist PA0. PA5 ist zudem mit der gruenen Board-LED LD2 verbunden.
- PA13 = **SWDIO**, PA14 = **SWCLK**. Der Test konfiguriert diese Pins nicht.
- Fuer UART4 auf PC10/PC11 muss Mbed die alternativen Pinnamen `PC_10_ALT0` und `PC_11_ALT0` verwenden. Ohne `ALT0` wird USART3 gewaehlt.
- Die VCP-Richtung aus Sicht des STM32: PA2 = TX zum ST-Link, PA3 = RX vom ST-Link.
