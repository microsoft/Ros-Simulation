# Creating a New World for the Robots

## Identify a Map

Create or locate a diagram to represent your map. The Arena will be the light color space. Dark color spaces will be walls, and grey areas will become boxes. For example (not to scale):

<img src="map-test.png" alt="Sample Map" width="600" />

The ability of the World Generator code to identify walls will be based on the color of your map. For best results, have very light open areas (white) and dark walls/obstacles (black). If you wish to modify this, adjust the OCCUPANCY_THRESHOLD in [worldGenerator.js](worldGenerator.js).

Note: keep track of the width and height of the map in pixels

## Generate XML from the Map

* Change the MAP_FILENAME value in [worldGenerator.js](worldGenerator.js) to map the file you wish to load
* Change the dimensions of the canvas in [worldGenerator.html](worldGenerator.html) to match the width and height of your map image in pixels
* Load [worldGenerator.html](worldGenerator.html) into a browser that will allow you to load your own image (we used Edge)
  * You should see your map in the browser. If you see a red square, try another browser
  * If you see red surrounding your image that does not represent walls, adjust the canvas size in the html file
* Enter the values for the x and y offset. These will help translate the dimensions from image units to ARGoS units. This will be half the width and half the height of your image
  * For example, if your image is 1278 x 1126 pixels, the x offset is 639 and the y offset is 563
* Click "Generate XML"
* Copy the XML from the box below the image to the construct.argos file. You may need to adjust the code to place the robots or OpenGL cameras.