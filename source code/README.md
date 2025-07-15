# How to set static com port for the low level microcontroller?


---

## 🔎 Step 1: Plug in your ESP32 and identify it

First, connect your ESP32 to your Raspberry Pi’s USB port.
Then, open a terminal and run:

```bash
lsusb
```

You’ll see a list of connected USB devices, like this:
![image](https://github.com/user-attachments/assets/d3ebd906-ed12-497b-8c47-4e51c6dc9645)




Take note of the **Vendor ID** (`1a86`) and **Product ID** (`7523`). These are your device’s fingerprints!

Now, let’s find its **serial number** (optional but recommended). Run:

```bash
udevadm info -a -n /dev/ttyUSB0
```

*(Replace `/dev/ttyUSB0` with the actual port assigned to your ESP32.Use the following command to find that)*
![image](https://github.com/user-attachments/assets/e19ba1fd-cdd1-463e-941e-fa568818ab8d)


Look for lines like:

```
ATTRS{idVendor}=="1a86"
ATTRS{idProduct}=="7523"
ATTRS{serial}=="0001"  # Example
```

If your device has a serial number, use it to **uniquely identify** it, especially if you have multiple ESP32s.

---

## ✍️ Step 2: Create the udev rule

Create a new rule file in `/etc/udev/rules.d/`.
You can call it something like:

```bash
sudo nano /etc/udev/rules.d/99-esp32.rules
```

Inside, add this line (replace with your actual values):

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="esp32_serial"
```

If you have a serial number too, add that:

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", ATTRS{serial}=="0001", SYMLINK+="esp32_serial"
```

This means:

* Whenever a device with **Vendor ID 1a86** and **Product ID 7523** is connected, udev will create a **symlink** (like an alias) at `/dev/esp32_serial`.

---

## 🔄 Step 3: Reload udev and test it

Reload udev rules:

```bash
sudo udevadm control --reload-rules
```

Then **unplug and plug back** your ESP32.

Check:

```bash
ls -l /dev/esp32_serial
```

You should see something like:

```
lrwxrwxrwx 1 root root 7 Jun 10 16:23 /dev/esp32_serial -> ttyUSB0
```

(Or whatever tty device it points to.)

---

## 🐍 Step 4: Use it in your Python script

Now, in your script:

```python
import serial

ser = serial.Serial('/dev/esp32_serial', 115200)
```

This way, no matter what port it lands on—`ttyUSB0`, `ttyUSB1`, `ttyACM0`—you’ll always connect using `/dev/esp32_serial`.

---

## 🚀 Pro Tips

✅ If your ESP32 shows up as `/dev/ttyACM0` instead of `/dev/ttyUSB0`, adjust your `udevadm info` command accordingly.
✅ Use `udevadm monitor` to see real-time device events:

```bash
udevadm monitor --udev
```

---

