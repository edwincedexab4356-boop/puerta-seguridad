# Puerta de Seguridad (RFID + código + Bluetooth)

Arduino UNO R4 WiFi con lector RFID RC522, sensor ultrasónico HC-SR04, pantalla LCD 16x2 I2C, servomotor SG90, LED blanco, verde y rojo, buzzer y módulo Bluetooth HC-05.

- **Simulador y control:** https://edwincedexab4356-boop.github.io/puerta-seguridad/
- **Programa:** `puerta_seguridad/puerta_seguridad.ino`
- Se entra con una tarjeta registrada o con el código (de fábrica **1234**) desde el simulador.
- Con 3 intentos fallidos se bloquea 30 s y suena la alarma. La tarjeta de administrador abre siempre.
- Desde el simulador se controla todo (abrir, cerrar, bloquear, pánico, luz, sonido, tarjetas, tiempos y código) y el historial guarda todo lo que pasa.
