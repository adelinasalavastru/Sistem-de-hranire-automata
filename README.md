#Sistem automat de hrănire (ESP32 + MQTT)

Descriere
Proiect IoT care simulează un sistem automat de hrănire folosind ESP32 și protocolul MQTT.
Dispozitivul primește comenzi printr-un broker MQTT public și confirmă execuția prin status și log.

## Tehnologii folosite
- ESP32 (simulat în Wokwi)
- MQTT Explorer (client PC)
- Mesaje log în format JSON

## Topic-uri MQTT
- Command: `adelina_salavastru/feed/command`
- Status: `adelina_salavastru/feed/status`
- Log: `adelina_salavastru/feed/log`

## Comenzi disponibile
- `FEED_NOW` – declanșează hrănire manuală (LED ON ~2 secunde)
- `SET_TIME:HH:MM` – setează ora hrănirii automate (o dată pe zi)

## Rulare (Wokwi)
1. Deschide proiectul în Wokwi.
2. Rulează simularea (Run).
3. Conectează-te cu MQTT Explorer la `broker.hivemq.com:1883`.
4. Trimite comenzi pe topic-ul de command.

## Rezultat
La execuție se aprinde LED-ul și se publică:
- `FEED_OK (MANUAL)` sau `FEED_OK (AUTO)` pe status
- mesaj JSON pe log
