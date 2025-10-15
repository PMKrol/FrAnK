Toolset for acquiring data from ESP32: current weight, IR frame, motor position.

[todo] esp soft with explanation.

FrAnK reads data output from ESP ie.: sudo ./frank /dev/ttyUSB0 board_name
and saves it as txt file with zipped frames from camera.

FrAnCs translates txt file to three csv files and [todo] zip file (photos) to data.

fakeRS sends sample data (mimics serial port with ESP32 connected).
