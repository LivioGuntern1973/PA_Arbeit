# Flowmeter-Simulator

Der Sketch emuliert den Allengra Micro Flow Sensor aus der I²C-Spezifikation v1.00. Er unterstützt den **Arduino Uno R3**, den **Arduino Nano Classic** mit ATmega328P und den **Joy-IT ARD-NanoV4-MC** mit ATmega328PB. Alle drei Boards werden mit **16 MHz** betrieben.

## Joy-IT Nano V4 mit MiniCore einrichten

In der Arduino IDE für den **ARD-NanoV4-MC** die folgenden Einstellungen wählen.

| Einstellung unter Werkzeuge | Wert |
| --- | --- |
| Board | MiniCore → ATmega328 |
| Variant | 328PB |
| Clock | External 16 MHz |
| Bootloader | Yes (UART0), falls diese Auswahl vorhanden ist |
| Port | USB-Anschluss des Nano |

Danach den Sketch über **Hochladen** übertragen. Die Anschlüsse sind **D2 für PWM**, **A4/SDA0 für SDA** und **A5/SCL0 für SCL**. Der Sketch verwendet die erste I²C-Schnittstelle `Wire`. Die zusätzliche Schnittstelle `Wire1` liegt auf anderen Pins. Die Boardauswahl und Pinbelegung sind in der [Joy-IT-Anleitung](https://joy-it.net/files/files/Produkte/ARD-NanoV4-MC/ARD-NanoV4-MC_Manual-EN_2026-05-12.pdf) beschrieben. Der ATmega328PB mit 16 MHz und 5-V-Logik ist in den [Herstellerangaben](https://joy-it.net/de/products/ARD-NanoV4-MC) aufgeführt.

Für das separate Modell **ARD-NanoV4-3V-MC** mit **8 MHz** ist eine weitere Anpassung des Timer-Takts nötig. Der vorliegende Sketch prüft ausdrücklich auf 16 MHz. [Herstellerangaben zum 3,3-V-Modell](https://joy-it.net/en/products/ARD-NanoV4-3V-MC)

## Anschliessen und starten

1. `FlowmeterSimulator.ino` in der Arduino IDE öffnen, das Board auswählen und den Sketch hochladen. Für einen anderen Maximaldurchfluss `#define MAX_FLOW_L_MIN 5.0` im Sketch ändern und erneut hochladen.
2. Arduino und Pumpensteuerung über **GND** verbinden. Den **Logik-PWM-Ausgang** der Steuerung mit **D2** verbinden. Bei der PES-Board-Software `Milch_made` ist dies `PB_PWM_M1`. Verwende bei abweichenden Logikpegeln einen geeigneten Pegelwandler. Ein 10-kΩ-Widerstand zwischen D2 und GND hält den Eingang bei abgezogenem PWM-Signal auf 0 %.
3. **SDA an A4**, **SCL an A5**. Für einen 3,3-V-I²C-Bus den 5-V-Arduino über einen **bidirektionalen I²C-Pegelwandler** anschliessen. SDA und SCL brauchen Pull-ups zur jeweiligen Versorgung auf beiden Seiten des Wandlers. [Arduino-Pinbelegung](https://docs.arduino.cc/learn/communication/wire/#arduino-i2c-pins)

Die Auswertung tastet D2 digital ab und mittelt den HIGH-Anteil über 50 ms. Dauerhaft LOW ergibt 0 l/min, 50 % ergeben etwa 2,5 l/min und dauerhaft HIGH ergibt 5 l/min. Die vorhandene `Milch_made`-Testsoftware verwendet bei Motorstillstand 50 % PWM. Der Simulator meldet dabei gemäss der gewünschten Kennlinie weiterhin etwa 2,5 l/min. Für 0 l/min muss das PWM-Signal auf 0 % stehen.

## Auslesen über I²C

Den Controller auf **100 kHz** einstellen. Die **7-Bit-Adresse ist `0x10`**. Pro Messwertabfrage `0x01 0x00` schreiben, mit STOP abschliessen und danach 24 Bytes lesen. Die drei Messwerte sind vorzeichenbehaftete 32-Bit-Zahlen in Little-Endian-Reihenfolge.

| Byte-Offset | Messwert | Umrechnung aus dem Rohwert |
| --- | --- | --- |
| 0 | Durchfluss | ÷ 100 für l/h, ÷ 6000 für l/min |
| 16 | Totalizer 1, Gesamtvolumen | ÷ 10 für ml |
| 20 | Totalizer 2, Volumen der aktuellen oder letzten Dosierung | ÷ 100 für ml |

Die übrigen Bytes stehen auf `0`, einschliesslich der Blasenerkennung. Die Spezifikation definiert für den 24-Byte-Datensatz kein CRC-Byte.

Beispiel für einen bereits eingerichteten Mbed-I²C-Controller `bus`. Mbed erwartet die um ein Bit verschobene Adresse. [Mbed-I²C-Referenz](https://os.mbed.com/docs/mbed-os/v6.16/apis/i2c.html)

```cpp
bus.frequency(100000);
constexpr int sensorAddress = 0x10 << 1;
const char command[] = {0x01, 0x00};
char data[24];
if (bus.write(sensorAddress, command, sizeof(command)) == 0 &&
    bus.read(sensorAddress, data, sizeof(data)) == 0) {
    // Durchfluss und Volumen aus den Offsets in der Tabelle dekodieren.
}
```

## Kurztest

- Mit 100 % PWM steigen beide Zähler um rund 83,3 ml pro Sekunde. Der Durchfluss-Rohwert ist `30000`.
- Bei 0 % bleiben beide Zähler auf ihrem letzten Stand. Für zwei getrennte Dosierungen mindestens 120 ms lang 0 % anlegen, damit ein ganzes Messfenster Stillstand erfasst. Bei der nächsten Dosierung startet Totalizer 2 wieder bei null und Totalizer 1 zählt weiter.
- Ein Neustart setzt beide Zähler zurück. Sie werden im RAM gehalten und bleiben beim maximal darstellbaren Registerwert stehen.
