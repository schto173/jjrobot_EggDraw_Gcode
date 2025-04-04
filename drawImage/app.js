const express = require('express');
const app = express();
const http = require('http').Server(app);
const io = require('socket.io')(http);
const multer = require('multer');
const sharp = require('sharp');
const net = require('net');
const potrace = require('potrace');
const { createCanvas, loadImage } = require('canvas');
const svgpath = require('svgpath');

const DEBUG = true;
const PORT = 3000;
const ROBOT_IP = '192.168.4.1';
const ROBOT_PORT = 8080;
const ROBOT_MAX_X = 360;
const ROBOT_MAX_Y = 90;

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

app.use(express.static('public'));
const upload = multer({ storage: multer.memoryStorage() });

function calculateScaledDimensions(originalWidth, originalHeight) {
    const imageAspectRatio = originalWidth / originalHeight;
    const robotAspectRatio = ROBOT_MAX_X / ROBOT_MAX_Y;
    
    if (imageAspectRatio > robotAspectRatio) {
        const scaledWidth = ROBOT_MAX_X;
        const scaledHeight = ROBOT_MAX_X / imageAspectRatio;
        const yOffset = (ROBOT_MAX_Y - scaledHeight) / 2;
        const scale = ROBOT_MAX_X / originalWidth;
        return { width: originalWidth, height: originalHeight, scale, yOffset };
    } else {
        const scaledHeight = ROBOT_MAX_Y;
        const scaledWidth = ROBOT_MAX_Y * imageAspectRatio;
        const xOffset = (ROBOT_MAX_X - scaledWidth) / 2;
        const scale = ROBOT_MAX_Y / originalHeight;
        return { width: originalWidth, height: originalHeight, scale, xOffset };
    }
}

function optimizePath(path, tolerance = 0.5) {
    // Douglas-Peucker algorithm for path simplification
    if (path.length <= 2) return path;
    
    function findFurthestPoint(start, end) {
        let maxDist = 0;
        let index = 0;
        
        const line = {
            x1: start.x,
            y1: start.y,
            x2: end.x,
            y2: end.y
        };
        
        for (let i = 1; i < path.length - 1; i++) {
            const point = path[i];
            const dist = pointToLineDistance(point, line);
            
            if (dist > maxDist) {
                maxDist = dist;
                index = i;
            }
        }
        
        return { maxDist, index };
    }
    
    function pointToLineDistance(point, line) {
        const numerator = Math.abs(
            (line.y2 - line.y1) * point.x -
            (line.x2 - line.x1) * point.y +
            line.x2 * line.y1 -
            line.y2 * line.x1
        );
        
        const denominator = Math.sqrt(
            Math.pow(line.y2 - line.y1, 2) +
            Math.pow(line.x2 - line.x1, 2)
        );
        
        return numerator / denominator;
    }
    
    const result = [path[0]];
    const stack = [[0, path.length - 1]];
    
    while (stack.length > 0) {
        const [start, end] = stack.pop();
        const { maxDist, index } = findFurthestPoint(path[start], path[end]);
        
        if (maxDist > tolerance) {
            stack.push([start, index]);
            stack.push([index, end]);
        } else {
            if (end - start > 1) {
                result.push(path[end]);
            }
        }
    }
    
    return result;
}

function optimizePathOrder(paths) {
    // Nearest neighbor algorithm for path ordering
    const optimizedPaths = [];
    const unvisited = [...paths];
    let currentPoint = { x: 0, y: 0 }; // Start from home position
    
    while (unvisited.length > 0) {
        let nearestDist = Infinity;
        let nearestPath = null;
        let nearestIndex = -1;
        let shouldReverse = false;
        
        // Find nearest path
        unvisited.forEach((path, index) => {
            // Check distance to path start
            const distToStart = distance(currentPoint, path[0]);
            const distToEnd = distance(currentPoint, path[path.length - 1]);
            
            if (distToStart < nearestDist) {
                nearestDist = distToStart;
                nearestPath = path;
                nearestIndex = index;
                shouldReverse = false;
            }
            
            if (distToEnd < nearestDist) {
                nearestDist = distToEnd;
                nearestPath = path;
                nearestIndex = index;
                shouldReverse = true;
            }
        });
        
        if (shouldReverse) {
            nearestPath.reverse();
        }
        
        optimizedPaths.push(nearestPath);
        currentPoint = nearestPath[nearestPath.length - 1];
        unvisited.splice(nearestIndex, 1);
    }
    
    return optimizedPaths;
}

function distance(p1, p2) {
    return Math.sqrt(Math.pow(p2.x - p1.x, 2) + Math.pow(p2.y - p1.y, 2));
}


function approximateBezier(p0, p1, p2, p3, segments) {
    const points = [];
    for (let t = 0; t <= 1; t += 1/segments) {
        const mt = 1 - t;
        const x = mt*mt*mt*p0.x + 3*mt*mt*t*p1.x + 3*mt*t*t*p2.x + t*t*t*p3.x;
        const y = mt*mt*mt*p0.y + 3*mt*mt*t*p1.y + 3*mt*t*t*p2.y + t*t*t*p3.y;
        points.push({ x, y });
    }
    return points;
}

function parseSVGPath(pathData) {
    const path = svgpath(pathData).abs(); // Convert to absolute coordinates
    const segments = path.segments;
    const points = [];
    let currentPoint = { x: 0, y: 0 };

    segments.forEach(segment => {
        const [command, ...params] = segment;

        switch (command.toLowerCase()) {
            case 'm': // Move to
                currentPoint = { x: params[0], y: params[1] };
                points.push({ ...currentPoint });
                break;

            case 'l': // Line to
                currentPoint = { x: params[0], y: params[1] };
                points.push({ ...currentPoint });
                break;

            case 'h': // Horizontal line
                currentPoint.x = params[0];
                points.push({ ...currentPoint });
                break;

            case 'v': // Vertical line
                currentPoint.y = params[0];
                points.push({ ...currentPoint });
                break;

            case 'c': // Cubic bezier
                // Add start and end points, approximate curve with line segments
                const bezierPoints = approximateBezier(
                    currentPoint,
                    { x: params[0], y: params[1] },
                    { x: params[2], y: params[3] },
                    { x: params[4], y: params[5] },
                    10 // number of segments
                );
                points.push(...bezierPoints);
                currentPoint = { x: params[4], y: params[5] };
                break;

            case 'z': // Close path
                if (points.length > 0) {
                    points.push({ ...points[0] }); // Close to first point
                }
                break;
        }
    });

    return points;
}

async function imageToVectors(buffer, scaleInfo) {
    return new Promise((resolve, reject) => {
        const params = {
            turdSize: 2,
            alphaMax: 1,
            optCurve: true,
            optTolerance: 0.2,
            threshold: 128
        };

        // Preprocess image with sharp
        sharp(buffer)
            .resize(scaleInfo.width, scaleInfo.height, {
                fit: 'contain',
                background: { r: 255, g: 255, b: 255, alpha: 1 }
            })
            .greyscale()
            .normalize()
            .toBuffer()
            .then(processedBuffer => {
                potrace.trace(processedBuffer, params, (err, svg) => {
                    if (err) reject(err);
                    
                    // Parse SVG paths
                    const paths = [];
                    const pathRegex = /d="([^"]+)"/g;
                    let match;
                    
                    while ((match = pathRegex.exec(svg)) !== null) {
                        const pathData = match[1];
                        const points = parseSVGPath(pathData);
                        
                        // Scale points
                        const scaledPoints = points.map(point => ({
                            x: point.x * scaleInfo.scale + (scaleInfo.xOffset || 0),
                            y: point.y * scaleInfo.scale + (scaleInfo.yOffset || 0)
                        }));
                        
                        paths.push(scaledPoints);
                    }
                    
                    // Optimize paths
                    const simplifiedPaths = paths.map(path => optimizePath(path));
                    const optimizedPaths = optimizePathOrder(simplifiedPaths);
                    
                    resolve(optimizedPaths);
                });
            })
            .catch(err => reject(err));
    });
}

function pathsToGcode(paths) {
    let gcode = [];
    gcode.push('G1 Z1'); // Pen up
    gcode.push('G1 X0 Y0'); // Home position
    
    paths.forEach(path => {
        if (path.length < 2) return;
        
        // Move to start of path
        gcode.push('G1 Z1'); // Pen up
        gcode.push(`G1 X${path[0].x.toFixed(3)} Y${path[0].y.toFixed(3)}`);
        gcode.push('G1 Z0'); // Pen down
        
        // Draw path
        path.slice(1).forEach(point => {
            gcode.push(`G1 X${point.x.toFixed(3)} Y${point.y.toFixed(3)}`);
        });
    });
    
    gcode.push('G1 Z1'); // Pen up
    gcode.push('G1 X0 Y0'); // Return home
    
    return gcode;
}

app.post('/upload', upload.single('image'), async (req, res) => {
    try {
        const image = sharp(req.file.buffer);
        const metadata = await image.metadata();
        
        const scaleInfo = calculateScaledDimensions(metadata.width, metadata.height);
        
        // Process image with Potrace
        const vectors = await imageToVectors(req.file.buffer, scaleInfo);
        const gcode = pathsToGcode(vectors);
        
        debug('G-code generated', { 
            commands: gcode.length,
            scaling: scaleInfo,
            paths: vectors.length
        });
        
        res.json({
            success: true,
            gcode: gcode,
            dimensions: {
                width: scaleInfo.width,
                height: scaleInfo.height,
                scale: scaleInfo.scale
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
                    const progress = ((currentCommandIndex + 1) / currentGcode.length) * 100;
                    io.emit('progress', {
                        current: currentCommandIndex + 1,
                        total: currentGcode.length,
                        percent: progress
                    });
                    
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