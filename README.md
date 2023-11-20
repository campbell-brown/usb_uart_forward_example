# USB / UART Forward Example

A simple example project to forward data to/from the UART/USB on an esp32c3.

## Hardware

### ESP32-C3-DevKitM-1

| Pin | Function |
| --- | --- |
| 18 | USB D- |
| 19 | USB D+ |
| 5V | USB 5V |
| 10 | UART RX |
| 4 | UART TX |
| GND | USB GND |
| GND | UART GND |

## Usage

Establish a UART connection with PuTTY with the following settings:

- Baud: 9600
- Data bits: 8
- Stop bits: 1
- Parity: EVEN
- Flow control: OFF

Establish a USB connection with PuTTY with the same settings as above.

Typing text into the UART connection window should output to the USB connection, and vice versa.
