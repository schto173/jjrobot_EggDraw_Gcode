// app.js
const express = require('express');
const app = express();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const multer = require('multer');
const sharp = require('sharp');
const net = require('net');
const path = require('path');
const fs = require('fs');
const cv = require('opencv4nodejs');

const DEBUG = true;
const PORT = 3000;
const ROBOT_IP = '192.168.4.1';
const ROBOT_PORT = 8080;
const MAX_WIDTH = 360;
const MAX_HEIGHT = 90;

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

// Create necessary directories
const dirs = ['public', 'public/uploads'];
dirs.forEach(dir => {
    if (!fs.existsSync(dir)) {
        fs.mkdirSync(dir, { recursive: true });
    }
});

// Multer setup for file uploads
const storage = multer.diskStorage({
    destination: (req, file, cb) => {
        cb(null, 'public/uploads/');
    },
    filename: (req, file, cb) => {
        cb(null, 'input-' + Date.now() + path.extname(file.originalname));
    }
});

const upload = multer({
    storage: storage,
    fileFilter: (req, file, cb) => {
        const allowedTypes = /jpeg|jpg|png/;
        const extname = allowedTypes.test(path.extname(file.originalname).toLowerCase());
        const mimetype = allowedTypes.test(file.mimetype);
        
        if (extname && mimetype) {
            cb(null, true);
        } else {
            cb(new Error('Only image files (JPEG, JPG, PNG) are allowed!'));
        }
    }
});

// Function to trace edges from the image
function traceEdges(edges) {
    const height = edges.rows;
    const width = edges.cols;
    const visited = new Set();
    const paths = [];
    
    // Helper to get pixel value (0 or 255)
    function getPixel(x, y) {
        if (x < 0 || x >= width || y < 0 || y >= height) return 0;
        return edges.at(y, x);
    }
    
    // Helper to check if a point is visited
    function isVisited(x, y) {
        return visited.has(`${x},${y}`);
    }
    
    // Helper to mark a point as visited
    function markVisited(x, y) {
        visited.add(`${x},${y}`);
    }
    
    // Helper to find next edge pixel
    function findNextPixel(x, y) {
        const directions = [
            [-1,-1], [0,-1], [1,-1],
            [-1, 0],         [1, 0],
            [-1, 1], [0, 1], [1, 1]
        ];
        
        for (let [dx, dy] of directions) {
            const nx = x + dx;
            const ny = y + dy;
            if (getPixel(nx, ny) === 255 && !isVisited(nx, ny)) {
                return [nx, ny];
            }
        }
        return null;
    }
    
    // Trace edges
    for (let y = 0; y < height; y++) {
        for (let x = 0; x < width; x++) {
            if (getPixel(x, y) === 255 && !isVisited(x, y)) {
                const path = [[x, y]];
                markVisited(x, y);
                
                let current = [x, y];
                let next;
                
                while ((next = findNextPixel(current[0], current[1]))) {
                    path.push(next);
                    markVisited(next[0], next[1]);
                    current = next;
                }
                
                if (path.length > 5) { // Filter out very short paths
                    paths.push(path);
                }
            }
        }
    }
    
    return paths;
}

// Function to convert image to vector paths and then to G-code
async function imageToVectorGcode(inputPath, scale = 100) {
    try {
        // 1. Read image
        debug('Reading image...');
        const img = cv.imread(inputPath);
        
        // 2. Convert to grayscale
        debug('Converting to grayscale...');
        const gray = img.cvtColor(cv.COLOR_BGR2GRAY);
        
        // 3. Apply Gaussian blur
        debug('Applying Gaussian blur...');
        const blurred = gray.gaussianBlur(new cv.Size(5, 5), 1.5);
        
        // 4. Apply Canny edge detection
        debug('Detecting edges...');
        const edges = blurred.canny(50, 150);
        
        // 5. Trace the edges
        debug('Tracing edges...');
        const paths = traceEdges(edges);
        debug(`Found ${paths.length} paths`);
        
        // Calculate scaling factors
        const scaleFactor = scale / 100;
        const originalWidth = img.cols;
        const originalHeight = img.rows;
        
        // Determine aspect ratio and fit within MAX_WIDTH and MAX_HEIGHT
        let xScale, yScale;
        if (originalWidth / originalHeight > MAX_WIDTH / MAX_HEIGHT) {
            // Width is the limiting factor
            xScale = (MAX_WIDTH * scaleFactor) / originalWidth;
            yScale = xScale; // Keep aspect ratio
        } else {
            // Height is the limiting factor
            yScale = (MAX_HEIGHT * scaleFactor) / originalHeight;
            xScale = yScale; // Keep aspect ratio
        }
        
        // Generate G-code from paths
        let gcode = [];
        gcode.push('G1 Z1'); // Pen up
        gcode.push('G1 X0 Y0'); // Home position
        
        paths.forEach(path => {
            if (path.length > 0) {
                // Move to the start of the path with pen up
                const startX = (path[0][0] * xScale).toFixed(3);
                const startY = (path[0][1] * yScale).toFixed(3);
                gcode.push(`G1 Z1`); // Pen up
                gcode.push(`G1 X${startX} Y${startY}`); // Move to start
                gcode.push(`G1 Z0`); // Pen down
                
                // Draw the path
                for (let i = 1; i < path.length; i++) {
                    const x = (path[i][0] * xScale).toFixed(3);
                    const y = (path[i][1] * yScale).toFixed(3);
                    gcode.push(`G1 X${x} Y${y}`);
                }
            }
        });
        
        gcode.push('G1 Z1'); // Pen up
        gcode.push('G1 X0 Y0'); // Return home
        
        debug('G-code generated', { commands: gcode.length });
        return gcode;
        
    } catch (error) {
        console.error('Error in imageToVectorGcode:', error);
        throw error;
    }
}

// Legacy function for backward compatibility
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

// Handle image upload - support both vector and legacy modes
app.post('/upload', upload.single('image'), async (req, res) => {
    try {
        if (!req.file) {
            throw new Error('Please select an image to upload');
        }

        const mode = req.body.mode || 'vector'; // Default to vector mode
        const scale = parseInt(req.body.scale || 100);
        
        if (scale < 20 || scale > 100) {
            throw new Error('Scale must be between 20% and 100%');
        }

        let gcode;
        
        if (mode === 'vector') {
            // Vector mode - use the new tracing algorithm
            gcode = await imageToVectorGcode(req.file.path, scale);
        } else {
            // Legacy mode - use the old algorithm
            const image = sharp(req.file.buffer);
            const metadata = await image.metadata();
            
            // Calculate new dimensions
            let newWidth = MAX_WIDTH;
            let newHeight = Math.round((MAX_WIDTH * metadata.height) / metadata.width);
            if (newHeight > MAX_HEIGHT) {
                newHeight = MAX_HEIGHT;
                newWidth = Math.round((MAX_HEIGHT * metadata.width) / metadata.height);
            }

            // Scale dimensions based on scale parameter
            newWidth = Math.round(newWidth * scale / 100);
            newHeight = Math.round(newHeight * scale / 100);

            // Process image
            const processedImage = await image
                .resize(newWidth, newHeight)
                .greyscale()
                .normalize()
                .threshold(200)
                .raw()
                .toBuffer();

            // Generate G-code
            gcode = await imageToGcode(processedImage, newWidth, newHeight);
        }
        
        debug('G-code generated', { commands: gcode.length });
        
        res.json({
            success: true,
            gcode: gcode,
            dimensions: { 
                width: MAX_WIDTH * scale / 100, 
                height: MAX_HEIGHT * scale / 100 
            }
        });
    } catch (error) {
        console.error('Error:', error);
        res.json({ success: false, error: error.message });
    }
});

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