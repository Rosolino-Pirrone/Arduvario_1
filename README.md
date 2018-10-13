# Arduvario_1
Arduvario_1 è un strumento di volo basato su due schede Adafruit, Feather M0  Bleufruit LE, Feather Huzzah, e due sensori, MS5611 e Adafruit Ultimate GPS Breakout - 66 e un display 4DSystems uOLED-160-G2.
Le due schede comunicano tra loro tramite connessione i2c condivisa con il sensore MS5611. Il GPS è connesso tramite seriale. Le due schede si scambiano i dati per poi inviare le stringhe NMEA tramite serriale, Bluetooth e molto semplicemente si può implementare la trasmissione tramite WIFi del Adafruit Huzzah.
Come per Arduvario ESP32 bisogna aggiungere le due stringhe char alla libreria Adafruit GPS per recuperare in modo semplice le stringhe NMEA del GPS
La trasmissione Bluetooth della Feather M0 è di tipo LE
