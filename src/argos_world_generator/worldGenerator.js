/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

async function LoadImageToCanvas() {
    /* File in current directory displaying map*/
    const MAP_FILENAME = "map.png";

    var canvas = document.getElementById("canvas");
    if (canvas.getContext) {
        ctx = canvas.getContext("2d");
        var img = await loadImage(MAP_FILENAME);
        img.crossOrigin = "0.05";
        ctx.drawImage(img, 0, 0);
    }
}

function SetDefaultOffsets()
{
    /* Default offsets to half of the width & height */
    var canvas = document.getElementById("canvas");
    document.getElementById("xOffset").value = canvas.width / 2;
    document.getElementById("yOffset").value = canvas.height / 2;
}

function GenerateXML() {
    var canvas = document.getElementById("canvas");

    /* Offset ARGoS center origin of map vs image origin (upper left) */
    /* Defined as enter of map in image coordinates */
    var xOffset = document.getElementById("xOffset").value;
    var yOffset = document.getElementById("yOffset").value;

    var xml = "<?xml version=\"1.0\" ?>\n";
    xml += "<argos-configuration>\n";
    xml += generateFramework();
    xml += generateControllers();
    xml += generateArena(canvas.height, canvas.width, xOffset, yOffset);
    xml += generatePhysicsEngines();
    xml += generateMedia();
    xml += generateVisualization(canvas.height, canvas.width);
    xml += "</argos-configuration>\n";

    /* Write to screen */
    var outputDiv = document.getElementById("output");
    outputDiv.innerText = xml;
}

function generateFramework() {
    var xml = "  <framework>\n";
    xml += "    <system threads=\"0\" />\n";
    xml += "    <experiment length=\"0\"\n";
    xml += "                ticks_per_second=\"10\"\n";
    xml += "                random_seed=\"124\" />\n";
    xml += "  </framework>\n";
    return xml;
}

function generateControllers() {
    var xml = "  <controllers>\n";
    xml += "    <footbot_diffusion_controller id=\"fdc\" library=\"build/controllers/footbot_diffusion/libfootbot_diffusion\">\n";
    xml += "      <actuators>\n";
    xml += "        <differential_steering implementation=\"default\" />\n";
    xml += "      </actuators>\n";
    xml += "      <sensors>\n";
    xml += "        <footbot_proximity implementation=\"default\" show_rays=\"true\" />\n";
    xml += "      </sensors>\n";
    xml += "      <params alpha=\"7.5\" delta=\"0.1\" velocity=\"5\" />\n";
    xml += "    </footbot_diffusion_controller>\n";
    xml += "  </controllers>\n";
    return xml;
}

function generateArena(height, weight, xOffset, yOffset, botType) {
    // maybe move this to param in gui in order to approximate
    /* Width of images in pixels / width of place in meters
       will need to scale your image to the real size of the factory */
    const METERS_PER_PIXEL = 0.05; // For demo map

    /* May not reflect reality, but a short wall is easier to view in visualization
       robot will still see this and treat as barrier (height of laser scan + margin) */
    const WALL_HEIGHT_IN_METERS = 1;
    const ARENA_HEIGHT_IN_PIXELS = height;
    const ARENA_WIDTH_IN_PIXELS = weight;
    const OCCUPANCY_THRESHOLD = 0.65;
    const DECIMAL_PRECISION = 4;

    /* ARGoS center origin vs image origin */
    /* Center of map in image coordinates */
    const X_OFFSET_IN_METERS = xOffset * METERS_PER_PIXEL;
    const Y_OFFSET_IN_METERS = yOffset * METERS_PER_PIXEL;

    /* Build boxes for rows of adjacent pixels */
    var occupiedPixels = getOccupiedPixels(OCCUPANCY_THRESHOLD);
    var boxes = getBoxes(occupiedPixels);

    /* XML-serialize the boxes */
    var xml = "  <arena size=\"" +
        (ARENA_WIDTH_IN_PIXELS * METERS_PER_PIXEL).toFixed(DECIMAL_PRECISION) + ", " +
        (ARENA_HEIGHT_IN_PIXELS * METERS_PER_PIXEL).toFixed(DECIMAL_PRECISION) + ", " +
        WALL_HEIGHT_IN_METERS + "\" center=\"" +
        X_OFFSET_IN_METERS.toFixed(DECIMAL_PRECISION) + "," + 
        Y_OFFSET_IN_METERS.toFixed(DECIMAL_PRECISION) + ",0.5\">\n";

    for (var i = 0; i < boxes.length; i++) {
        var curBox = boxes[i];
        var midPointOfBar = getMidpointOfBar(curBox.x, curBox.y, curBox.height, curBox.width);

        /* Pixel coordinates are measured from the top-left corner. Find equivalent  */
        /* in ARGoS coordinates (origin in the center of the image)                  */
        var topLeftX = -ARENA_WIDTH_IN_PIXELS / 2.0;
        var topLeftY = ARENA_HEIGHT_IN_PIXELS / 2.0;
        var barMidXInArenaCoords = topLeftX + midPointOfBar.x;
        var barMidYInArenaCoords = topLeftY - midPointOfBar.y;

        xml += generateBox("wall" + i,
            (barMidXInArenaCoords * METERS_PER_PIXEL + X_OFFSET_IN_METERS).toFixed(DECIMAL_PRECISION),
            (barMidYInArenaCoords * METERS_PER_PIXEL + Y_OFFSET_IN_METERS).toFixed(DECIMAL_PRECISION),
            0,
            (curBox.width * METERS_PER_PIXEL).toFixed(DECIMAL_PRECISION),
            (curBox.height * METERS_PER_PIXEL).toFixed(DECIMAL_PRECISION),
            WALL_HEIGHT_IN_METERS);
    }

    /* Initialize the bot */
    /* Note: do not initialize bots in immoveable spaces (i.e. walls), otherwise you will see ARGoS errors */
    /*       see ARGoS documentation for more information on distributing robots within a bounding box    */
    xml += "    <foot-bot id=\"fb_0\">\n";
    xml += "      <body position=\"5,5,0\" orientation=\"0,0,0\" />\n";
    xml += "      <controller config=\"fdc\"/>\n";
    xml += "    </foot-bot>\n";

    xml += "  </arena>\n";

    return xml;
}

function generatePhysicsEngines() {
    var xml = "  <physics_engines>\n";
    xml += "    <dynamics2d id=\"dyn2d\" />\n";
    xml += "  </physics_engines>\n";
    return xml;
}

function generateMedia() {
    var xml = "  <media />\n";
    return xml;
}

function generateVisualization(height, width) {  
    /* factor to raise the camera above the map by, relative to the max dimension to show the whole map */
    const CAMERA_FACTOR = 16;

    var maxDimension = Math.max(width, height);

    xml = "  <visualization>\n";
    xml += "    <qt-opengl>\n";
    xml += "      <camera>\n";
    xml += `        <placement idx=\"0\" position=\"0,0,${maxDimension / CAMERA_FACTOR}\" look_at=\"0,0,0\" lens_focal_length=\"2\" />\n`;
    xml += "      </camera>\n";
    xml += "    </qt-opengl>\n";
    xml += "  </visualization>\n";
    return xml;
}

function getMidpointOfBar(x, y, height, width) {
    var midX = x + width / 2.0;
    var midY = y + height / 2.0;
    return {
        "x": midX,
        "y": midY
    };
}

function generateBox(id, xPos, yPos, zPos, xSize, ySize, zSize) {
    var xml = "    <box id=\"" + id + "\" size=\"" + xSize + "," + ySize + "," + zSize + "\" movable=\"false\">\n";
    xml += "      <body position=\"" + xPos + "," + yPos + "," + zPos + "\" orientation=\"0,0,0\" />\n";
    xml += "    </box>\n";
    return xml;
}

/* Create 1x1 walls for each pixel above threshold */
function getOccupiedPixels(threshold) {
    /* Each pixel shows as 4 values (RGBA) from 0-255. Average the color values and scale to range of 0..1 */
    /* Values above threshold (0.65) should be returned */
    /* Threshold needed for ROS SLAM mapping. and the probability that something is an obstacle */
    var occupiedPixels = [];
    var canvas = document.getElementById("canvas");
    if (canvas.getContext) {
        ctx = canvas.getContext("2d");
        var imgData = ctx.getImageData(0, 0, canvas.width, canvas.height);
        for (var y = 0; y < canvas.height; y++) {
            for (var x = 0; x < canvas.width; x++) {
                /* Since imgData is 1D array, calculate offset for this pixel */
                var offset = y * (canvas.width * 4) + x * 4;
                var avgColor = (imgData.data[offset] + imgData.data[offset + 1] + imgData.data[offset + 2]) / 3;
                var pctOccupied = (255 - avgColor) / 255.0;
                if (pctOccupied >= threshold) {
                    occupiedPixels.push({
                        "x": x,
                        "y": y
                    });
                    imgData.data[offset] = 255;
                    imgData.data[offset + 1] = 0;
                    imgData.data[offset + 2] = 0;
                    imgData.data[offset + 3] = 255;
                }
            }
        }
        ctx.putImageData(imgData, 0, 0);
    }
    return occupiedPixels;
}

function getBoxes(occupiedPixels) {
    const boxesAtY = new Map();

    /* Build up walls by combining two horizontally adjacent walls into bigger walls */
    for (var i = 0; i < occupiedPixels.length; i++) {
        var curPixel = occupiedPixels[i];
        if (!boxesAtY.has(curPixel.y)) {

            /* If no boxes exist for this row yet, create one */
            var boxesAtX = new Map();
            boxesAtX.set(curPixel.x, {
                "x": curPixel.x,
                "y": curPixel.y,
                height: 1,
                width: 1
            });
            boxesAtY.set(curPixel.y, boxesAtX);
        } else {
            var boxesAtX = boxesAtY.get(curPixel.y);
            var oneToLeft = curPixel.x - 1;
            if (boxesAtX.has(oneToLeft)) {
                var curBox = boxesAtX.get(curPixel.x - 1);
                curBox.width += 1;

                /* Index based on rightmost X-position occupied by the box */
                boxesAtX.delete(oneToLeft);
                boxesAtX.set(curPixel.x, curBox);
            } else {

                /* It's a new box */
                boxesAtX.set(curPixel.x, {
                    "x": curPixel.x,
                    "y": curPixel.y,
                    height: 1,
                    width: 1
                });
            }
        }
    }

    /*  Collapse rows of identical widths as follows:
        Loop over the boxes in a given row.
        If there is an adjacent box of the same width below it, update the adjacent box's height and y-position
        If not output the current box
    */
    var boxes = [];
    for (var [y, boxesInRow] of boxesAtY) {
        if (boxesAtY.has(y + 1)) {
            /* Boxes exist in next row, so check to see if we can collapse them */
            var boxesInNextRow = boxesAtY.get(y + 1);
            for (var [x, box] of boxesInRow) {
                if (boxesInNextRow.has(x)) {
                    var possibleMatch = boxesInNextRow.get(x);
                    if (box.width == possibleMatch.width) {
                        /* Box in next row exists at this xPos with same widths so collapse; do not emit current box */
                        possibleMatch.height += box.height;
                        possibleMatch.y -= box.height;
                    } else {
                        /* Box in next row exists at this xPos but widths vary so can't collapse; emit current box */
                        boxes.push(box);
                    }
                } else {
                    /* No box in the next row at same x pos, so cannot collapse; emit current box */
                    boxes.push(box);
                }
            }
        } else {
            /* No boxes in next row, so emit all from current row */
            for (var box of boxesInRow.values())
                boxes.push(box);
        }
    }
    return boxes;
}

function loadImage(url) {
    return new Promise(resolve => {
        var i = new Image();
        i.onload = () => {
            resolve(i)
        };
        i.src = url;
        i.crossOrigin = "anonymous";
    });
}