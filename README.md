outliner
========

create an outline for you house easily.

Measures distance using a wheel a magnet and a hall effect sensor.
Mesures the heading using a magnometer(compass).
Combines the data to a SVG plot that outlines the shape that you have circled around.

See pictures here: https://plus.google.com/photos/115790465245856717320/albums/5888237192200439313?authkey=CNHJvdHGg-2EgQE
Inside the repository are code and schematics (in Fritzing format).

Instructions:
- Build the outliner device according to the photos above
- run server.py
- surf to /ui.html (tested on chrome and FF)
- Walk around with the device. make sure that the compass is balanced (not tilting, and points to the right direction)
- See the plot updated in real time in your browser!!

Parts:
- Electronics from Adafruit (Raspberry PI, LSM303DLHC, Hall effect sensor US5881LUA, rare earth magnet)
- caster wheel - from an Ikea furniture
- Stick, L shape brackets - hardware store.

Notes:
- The Perimeter of my caster wheel is 6.25"; if yours is different, update it in ui.html.
- When I use the device, sometimes the pi stops responding. probably because of the shakes. a reset resolves this.
- Used double sided tape to mount the magnet on the wheel
- Magnometer can't be near the magnet!! it will mess up the readings
