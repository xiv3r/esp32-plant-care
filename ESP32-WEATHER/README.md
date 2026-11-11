## 1. Upload
- Open the file in Arduino IDE.
- Select **ESP32 Dev Module** (or your board).
- Click **Upload**.

## 2. First-time setup
1. On your phone/laptop, connect to WiFi **`ESP32_Live_Weather`**.
2. Password: **`12345678`**
3. A captive portal opens (or go to **`192.168.4.1`**).
4. Fill in:
   - **AP SSID / Password** — the ESP32's own hotspot name and password (8–63 chars or empty for open).
   - **SSID / Password** — your home WiFi.
   - **Latitude / Longitude** — your location (e.g. `14.5995`, `120.9842`).
   - **Timezone** — e.g. `Asia/Manila` or `UTC-8`.
5. Tap **Save & Reconnect**.
6. Reconnect your phone/laptop to your home WiFi.
7. Open **`http://<ESP32-IP>`** (shown on your router) to see the weather dashboard.

## 3. Daily use
- **Dashboard** — current weather, today, 7-day forecast, air quality.
- **Settings** — change WiFi, location, timezone.
- **WiFi Scan** — find nearby networks.
- **Factory Reset** — hold the **BOOT** button for **5 seconds** (or use the button in Settings).

## 4. If the timezone shows a warning
- Use a **POSIX string** like `UTC-8`, `UTC-5:30`, `CET-1CEST,M3.5.0,M10.5.0/3` — or one of the common IANA names already supported (Manila, Tokyo, London, New York, Sydney, etc.).

## 5. If something goes wrong
- Hold **BOOT** for 5 seconds → resets everything → reconnect to `ESP32_Live_Weather`.

That's it — the firmware is ready to run.
