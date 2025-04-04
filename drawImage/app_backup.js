// app.js
const express = require('express');
const app = express();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const multer = require('multer');
const sharp = require('sharp');
const net = require('net');

const DEBUG = true;
const PORT = 3000;
const ROBOT_IP = '192.168.4.1';
const ROBOT_PORT = 8080;
const MAX_WIDTH = 360*4;
const MAX_HEIGHT = 90*4;

let robotClient = null;
let isConnected = false;
let currentGcode = [];
let isDrawing = false;
let currentCommandIndex = 0;

function debug(msg, data = null) {
    if (DEBUG) {
        if (data) {
            console.log(`[DEBUG] ${msg}:`, data);
        } else {
            console.log(`[DEBUG] ${msg}`);
        }
    }
}

// Set up static file serving and file upload
app.use(express.static('public'));
const upload = multer({ storage: multer.memoryStorage() });

// Handle image upload
app.post('/upload', upload.single('image'), async (req, res) => {
    try {
        const image = sharp(req.file.buffer);
        const metadata = await image.metadata();
        
        // Calculate new dimensions
        let newWidth = MAX_WIDTH;
        let newHeight = Math.round((MAX_WIDTH * metadata.height) / metadata.width);
        if (newHeight > MAX_HEIGHT) {
            newHeight = MAX_HEIGHT;
            newWidth = Math.round((MAX_HEIGHT * metadata.width) / metadata.height);
        }

        // Process image
        const processedImage = await image
            .resize(newWidth, newHeight)
            .greyscale()
            .normalize()
            .threshold(200)
            .raw()
            .toBuffer();

        // Generate G-code
        const gcode = await imageToGcode(processedImage, newWidth, newHeight);
        debug('G-code generated', { commands: gcode.length });
        
        res.json({
            success: true,
            gcode: gcode,
            dimensions: { width: newWidth, height: newHeight }
        });
    } catch (error) {
        console.error('Error:', error);
        res.json({ success: false, error: error.message });
    }
});

async function imageToGcode(buffer, width, height) {
  let gcode = [];
  gcode.push('G1 Z1'); // Pen up
  gcode.push('G1 X0 Y0'); // Home position
  
  // Calculate scaling factors to map to the required output ranges
  const xScale = 360 / width;  // Scale to 0-360 range
  const yScale = 90 / height;  // Scale to 0-90 range
  
  for (let y = 0; y < height; y++) {
      let lineStart = null;
      for (let x = 0; x < width; x++) {
          const isBlack = buffer[y * width + x] < 128;
          if (isBlack && lineStart === null) {
              lineStart = x;
          } else if (!isBlack && lineStart !== null) {
              const scaledY = (y * yScale).toFixed(3);
              const scaledStartX = (lineStart * xScale).toFixed(3);
              const scaledEndX = ((x-1) * xScale).toFixed(3);
              
              gcode.push(`G1 Z1`); // Pen up
              gcode.push(`G1 X${scaledStartX} Y${scaledY}`); // Move to start
              gcode.push(`G1 Z0`); // Pen down
              gcode.push(`G1 X${scaledEndX} Y${scaledY}`); // Draw to end
              lineStart = null;
          }
      }
      if (lineStart !== null) {
          const scaledY = (y * yScale).toFixed(3);
          const scaledStartX = (lineStart * xScale).toFixed(3);
          const scaledEndX = ((width-1) * xScale).toFixed(3);
          
          gcode.push(`G1 Z1`);
          gcode.push(`G1 X${scaledStartX} Y${scaledY}`);
          gcode.push(`G1 Z0`);
          gcode.push(`G1 X${scaledEndX} Y${scaledY}`);
      }
  }
  
  gcode.push('G1 Z1'); // Pen up
  gcode.push('G1 X0 Y0'); // Return home
  return gcode;
}

function sendNextCommand() {
    if (!isDrawing || currentCommandIndex >= currentGcode.length) {
        if (isDrawing) {
            debug('Drawing complete');
            isDrawing = false;
            io.emit('drawing-complete');
        }
        return;
    }

    const command = currentGcode[currentCommandIndex];
    debug(`Sending command ${currentCommandIndex + 1}/${currentGcode.length}`, command);
    robotClient.write(command + '\n');
}

io.on('connection', (socket) => {
    debug('Web client connected');
    socket.emit('connectionStatus', isConnected);

    socket.on('connect-to-robot', () => {
        if (!robotClient) {
            robotClient = new net.Socket();
            robotClient.connect(ROBOT_PORT, ROBOT_IP, () => {
                debug('Connected to robot');
                isConnected = true;
                io.emit('connectionStatus', true);
            });

            robotClient.on('data', (data) => {
                const response = data.toString().trim();
                debug('Robot response', response);
                
                if (response.includes('OK')) {
                    // Send progress update
                    const progress = ((currentCommandIndex + 1) / currentGcode.length) * 100;
                    io.emit('progress', {
                        current: currentCommandIndex + 1,
                        total: currentGcode.length,
                        percent: progress
                    });
                    
                    // Move to next command
                    currentCommandIndex++;
                    sendNextCommand();
                }
                
                io.emit('robot-response', response);
            });

            robotClient.on('error', (err) => {
                debug('Connection error', err.message);
                isConnected = false;
                robotClient = null;
                io.emit('connectionStatus', false);
            });

            robotClient.on('close', () => {
                debug('Connection closed');
                isConnected = false;
                robotClient = null;
                io.emit('connectionStatus', false);
            });
        }
    });

    socket.on('disconnect-from-robot', () => {
        if (robotClient) {
            robotClient.destroy();
            robotClient = null;
            isConnected = false;
            io.emit('connectionStatus', false);
        }
    });

    socket.on('start-drawing', (gcode) => {
        if (isConnected && robotClient && !isDrawing) {
            debug('Starting new drawing', { commands: gcode.length });
            currentGcode = gcode;
            currentCommandIndex = 0;
            isDrawing = true;
            sendNextCommand();
        }
    });

    socket.on('stop-drawing', () => {
        debug('Stopping drawing');
        isDrawing = false;
        currentGcode = [];
        currentCommandIndex = 0;
        io.emit('drawing-stopped');
    });
});

http.listen(PORT, () => {
    debug('Server started', { port: PORT });
    console.log(`Server running at http://localhost:${PORT}`);
});