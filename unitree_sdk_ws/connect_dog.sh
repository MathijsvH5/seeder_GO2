#!/bin/bash
# REPLACE WITH YOUR MAC
MAC="00:70:07:1D:0D:82"

echo "--- Resetting Bluetooth Link ---"

# 2. Force release the port in case it is 'stuck' from the last run
sudo rfcomm release 0 2>/dev/null

# 3. Re-bind the port to your ESP32
echo "Binding ESP32 ($MAC) to /dev/rfcomm0..."
sudo rfcomm bind 0 $MAC 1

# 4. Set permissions so your C++ code can access it
sudo chmod a+rw /dev/rfcomm0

echo "--- Done! Your ESP32 is ready for the C++ script. ---"