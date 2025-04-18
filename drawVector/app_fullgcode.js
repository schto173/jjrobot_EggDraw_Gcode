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

// Helper: Calculate the square of the distance between two points
function getSqDist(p1, p2) {
    const dx = p1[0] - p2[0];
    const dy = p1[1] - p2[1];
    return dx * dx + dy * dy;
}

// Helper: Calculate the square of the perpendicular distance from point p to line segment [p1, p2]
function getSqSegDist(p, p1, p2) {
    let x = p1[0];
    let y = p1[1];
    let dx = p2[0] - x;
    let dy = p2[1] - y;

    if (dx !== 0 || dy !== 0) {
        const t = ((p[0] - x) * dx + (p[1] - y) * dy) / (dx * dx + dy * dy);

        if (t > 1) {
            x = p2[0];
            y = p2[1];
        } else if (t > 0) {
            x += dx * t;
            y += dy * t;
        }
    }

    dx = p[0] - x;
    dy = p[1] - y;

    return dx * dx + dy * dy;
}

// Ramer-Douglas-Peucker algorithm
function simplifyRDP(points, epsilonSq) { // Use squared epsilon for efficiency
    if (points.length < 3) {
        return points; // Cannot simplify further
    }

    let dmax = 0;
    let index = 0;
    const end = points.length - 1;

    for (let i = 1; i < end; i++) {
        const d = getSqSegDist(points[i], points[0], points[end]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }

    // If max distance is greater than epsilon, recursively simplify
    if (dmax > epsilonSq) {
        const recResults1 = simplifyRDP(points.slice(0, index + 1), epsilonSq);
        const recResults2 = simplifyRDP(points.slice(index, points.length), epsilonSq);

        // Build the result list
        return recResults1.slice(0, recResults1.length - 1).concat(recResults2);
    } else {
        // Max distance is within tolerance, simplify to start and end points
        return [points[0], points[end]];
    }
}

// Wrapper function to handle the epsilon squaring
function simplifyPath(path, epsilon) {
    if (epsilon <= 0) return path; // No simplification
    return simplifyRDP(path, epsilon * epsilon);
}



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
            [-1, -1], [0, -1], [1, -1],
            [-1, 0], [1, 0],
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

// Chaikin's Algorithm for path smoothing
function smoothPathChaikin(path, iterations = 1) {
    if (iterations <= 0 || path.length < 3) {
        return path; // No smoothing needed or possible
    }

    let currentPath = path;

    for (let iter = 0; iter < iterations; iter++) {
        const smoothedPath = [];
        if (currentPath.length > 0) {
            // Keep the very first point
            smoothedPath.push(currentPath[0]);
        }

        for (let i = 0; i < currentPath.length - 1; i++) {
            const p0 = currentPath[i];
            const p1 = currentPath[i + 1];

            // Calculate the two new points for the segment p0-p1
            // Point 1 (q_i): 1/4 from p0 towards p1
            const q_i = [
                p0[0] * 0.75 + p1[0] * 0.25,
                p0[1] * 0.75 + p1[1] * 0.25
            ];
            // Point 2 (r_i): 1/4 from p1 towards p0 (or 3/4 from p0 towards p1)
             const r_i = [
                p0[0] * 0.25 + p1[0] * 0.75,
                p0[1] * 0.25 + p1[1] * 0.75
            ];

            smoothedPath.push(q_i);
            smoothedPath.push(r_i);
        }

         if (currentPath.length > 1) {
             // Keep the very last point
             smoothedPath.push(currentPath[currentPath.length - 1]);
         }
        currentPath = smoothedPath; // Use the smoothed path for the next iteration
    }

    return currentPath;
}



// Function to convert image to vector paths and then to G-code
async function imageToVectorGcode(inputPath, scale = 100) {
    try {
        // --- Parameters ---
        const paddingSize = 2;
        const cannyLowerThreshold = 50;
        const cannyUpperThreshold = 180;
        const gaussianKernelSize = new cv.Size(3, 3);
        const gaussianSigma = 2.2;
        const minPathLengthBeforeSimp = 5;
        const simplificationTolerance = 0.7; // RDP epsilon (pixels). Tune this! Maybe higher now?
        const smoothingIterations = 1;      // <<< NEW PARAMETER: Chaikin iterations (0=off, 1-4 typical)

        // ... (Keep steps 1-5: Read, Pad, Gray, Blur, Canny) ...
        // 1. Read image
        debug('Reading image...');
        const originalImg = cv.imread(inputPath);
        const originalHeight = originalImg.rows;
        const originalWidth = originalImg.cols;

        // 2. Add padding
        debug(`Adding ${paddingSize}px padding...`);
        const paddedImg = originalImg.copyMakeBorder(paddingSize, paddingSize, paddingSize, paddingSize, cv.BORDER_CONSTANT, new cv.Vec3(0, 0, 0));

        // 3. Convert padded image to grayscale
        debug('Converting padded image to grayscale...');
        const gray = paddedImg.cvtColor(cv.COLOR_BGR2GRAY);

        // 4. Apply Gaussian blur
        debug('Applying Gaussian blur...');
        const blurred = gray.gaussianBlur(gaussianKernelSize, gaussianSigma);

        // 5. Apply Canny edge detection on the padded image
        debug('Detecting edges on padded image...');
        const edges = blurred.canny(cannyLowerThreshold, cannyUpperThreshold);


        // 6. Thinning (Optional but recommended)
        let processedEdges = edges;
        try {
            if (cv.ximgproc && cv.ximgproc.thinning) {
                 debug('Applying morphological thinning...');
                 const thinnedEdges = cv.ximgproc.thinning(edges, cv.ximgproc.THINNING_ZHANGSUEN);
                 processedEdges = thinnedEdges;
                 debug('Thinning complete.');
                 // cv.imwrite('public/uploads/thinned_edges_debug.png', processedEdges);
            } else {
                 debug('cv.ximgproc.thinning not available. Skipping thinning step.');
            }
        } catch (thinningError) {
             console.error('Error during thinning:', thinningError);
             debug('Proceeding without thinning due to error.');
        }

        // 7. Trace the edges
        debug('Tracing edges...');
        const rawPaths = traceEdges(processedEdges);
        debug(`Found ${rawPaths.length} raw paths`);

        // 8. Simplify the traced paths (RDP)
        debug(`Simplifying paths with RDP tolerance ${simplificationTolerance}...`);
        const simplifiedPaths = rawPaths
            .filter(path => path.length >= minPathLengthBeforeSimp)
            .map(path => simplifyPath(path, simplificationTolerance)) // Apply RDP
            .filter(path => path.length >= 2);
        debug(`Reduced to ${simplifiedPaths.length} simplified paths after RDP`);

        // 9. Smooth the simplified paths (Chaikin)
        let finalPaths = simplifiedPaths;
        if (smoothingIterations > 0) {
            debug(`Smoothing paths with Chaikin (${smoothingIterations} iterations)...`);
            finalPaths = simplifiedPaths.map(path => smoothPathChaikin(path, smoothingIterations));
             debug(`Path count after smoothing: ${finalPaths.length}`); // Should be same as after RDP
        }


        // 10. Calculate scaling factors
        const scaleFactor = scale / 100;
        let xScale, yScale;
        // ... (rest of scaling logic) ...
        if (originalWidth / originalHeight > MAX_WIDTH / MAX_HEIGHT) {
            xScale = (MAX_WIDTH * scaleFactor) / originalWidth;
            yScale = xScale;
        } else {
            yScale = (MAX_HEIGHT * scaleFactor) / originalHeight;
            xScale = yScale;
        }
        debug('Calculated scaling factors', { xScale, yScale });


        // 11. Generate G-code from FINAL (smoothed) paths
        let gcode = [];
        gcode.push('G1 Z1'); // Pen up
        gcode.push('G1 X0 Y0'); // Home position

        let totalPoints = 0;

        finalPaths.forEach(path => { // Use finalPaths here
            if (path.length < 2) return; // Skip paths with less than 2 points after smoothing

            totalPoints += path.length;
            const startX = ((path[0][0] - paddingSize) * xScale).toFixed(3);
            const startY = ((path[0][1] - paddingSize) * yScale).toFixed(3);

            if (startX < 0 || startY < 0) return;

            gcode.push(`G1 Z1`);
            gcode.push(`G1 X${startX} Y${startY}`);
            gcode.push(`G1 Z0`);

            // Draw the path using smoothed points
            for (let i = 1; i < path.length; i++) {
                const x = ((path[i][0] - paddingSize) * xScale).toFixed(3);
                const y = ((path[i][1] - paddingSize) * yScale).toFixed(3);

                // ... (rest of G-code generation logic with boundary checks) ...
                 if (x < 0 || y < 0 || x > MAX_WIDTH || y > MAX_HEIGHT) {
                     gcode.push(`G1 Z1`);
                } else {
                    if (gcode[gcode.length - 1] === 'G1 Z1') {
                        gcode.push(`G1 X${x} Y${y}`);
                        gcode.push(`G1 Z0`);
                    } else {
                        gcode.push(`G1 X${x} Y${y}`);
                    }
                }
            }
            gcode.push(`G1 Z1`);
        });

        // ... (rest of G-code cleanup and return) ...
        if (gcode.length > 0 && gcode[gcode.length - 1] === 'G1 Z1') {
            gcode.pop();
        }

        gcode.push('G1 Z1');
        gcode.push('G1 X0 Y0');

        debug('Final G-code generated', { commands: gcode.length, totalPoints: totalPoints });
        return gcode;

    } catch (error) {
        console.error('Error in imageToVectorGcode:', error);
        throw error;
    }
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
async function sendGcodeBatch(gcode, socket) {
  try {
    if (!gcode || gcode.length === 0) {
      socket.emit('error', 'No G-code commands to send');
      return;
    }
    
    // Split commands into chunks of 20 (leaving room for batch overhead)
    const chunks = [];
    for (let i = 0; i < gcode.length; i += 20) {
      chunks.push(gcode.slice(i, i + 20));
    }
    
    socket.emit('status', `Processing ${gcode.length} commands in ${chunks.length} batches...`);
    let totalProcessed = 0;
    
    // Process each chunk
    for (let i = 0; i < chunks.length; i++) {
      const chunk = chunks[i];
      socket.emit('status', `Processing batch ${i + 1}/${chunks.length}...`);
      console.log(chunk);
      // Start batch mode
      const startResponse = await sendCommand('BATCH_START');
      if (!startResponse.includes('BATCH_MODE_READY')) {
        throw new Error('Failed to start batch mode');
      }
      
      // Send commands in current chunk
      for (const cmd of chunk) {
        const response = await sendCommand(`BATCH_COMMAND:${cmd}`);
        if (response.includes('CMD_ADDED')) {
          totalProcessed++;
          socket.emit('progress', {
            current: totalProcessed,
            total: gcode.length,
            percent: (totalProcessed / gcode.length) * 100
          });
        }
      }
      
      // End current batch and wait for completion
      const endResponse = await sendCommand('BATCH_END');
      if (!endResponse.includes('BATCH_PROCESSING')) {
        throw new Error('Failed to start batch processing');
      }
      
      // Wait for batch completion
      await waitForBatchCompletion(socket);
    }
    
    socket.emit('drawing-complete');
    return true;
  } catch (error) {
    console.error('Error in batch mode:', error);
    socket.emit('error', error.message);
    return false;
  }
}

// Helper function to wait for batch completion
function waitForBatchCompletion(socket) {
  return new Promise((resolve, reject) => {
    const checkInterval = setInterval(async () => {
      try {
        const response = await sendCommand('BATCH_STATUS');
        if (response.includes('BATCH_COMPLETE')) {
          clearInterval(checkInterval);
          resolve();
        }
      } catch (error) {
        clearInterval(checkInterval);
        reject(error);
      }
    }, 1000);
    
    // Add timeout
    setTimeout(() => {
      clearInterval(checkInterval);
      reject(new Error('Batch completion timeout'));
    }, 60000); // 1 minute timeout
  });
}


function sendCommand(command) {
    return new Promise((resolve, reject) => {
        if (!robotClient || !isConnected) {
            reject(new Error('Not connected to robot'));
            return;
        }

        let responseTimeout;
        let responseData = '';

        const responseHandler = (data) => {
            const response = data.toString().trim();
            responseData += response;

            if (response.includes('OK') ||
                response.includes('BATCH_MODE_READY') ||
                response.includes('CMD_ADDED') ||
                response.includes('BATCH_PROCESSING') ||
                response.includes('BATCH_COMPLETE') ||
                response.includes('STATUS:')) {

                clearTimeout(responseTimeout);
                robotClient.removeListener('data', responseHandler);
                resolve(responseData);
            }
        };

        responseTimeout = setTimeout(() => {
            robotClient.removeListener('data', responseHandler);
            reject(new Error('Command timeout'));
        }, 5000);

        robotClient.on('data', responseHandler);
        robotClient.write(command + '\n');
    });
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

            // Use batch mode for drawings with more than 10 commands
            if (gcode.length > 10) {
                isDrawing = true;
                sendGcodeBatch(gcode, socket)
                    .then(success => {
                        if (!success) {
                            isDrawing = false;
                            io.emit('drawing-stopped');
                        }
                    })
                    .catch(err => {
                        console.error('Batch drawing error:', err);
                        isDrawing = false;
                        io.emit('drawing-stopped');
                    });
            } else {
                // Use original sequential mode for small drawings
                currentGcode = gcode;
                currentCommandIndex = 0;
                isDrawing = true;
                sendNextCommand();
            }
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