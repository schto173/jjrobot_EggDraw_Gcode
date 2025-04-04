// app.js
const express = require('express');
const app = express();
const http = require('http').createServer(app);
const io = require('socket.io')(http);
const net = require('net');

const PORT = 3000;
const ROBOT_IP = '192.168.4.1';
const ROBOT_PORT = 8080;

let robotClient = null;
let isConnected = false;

// Serve static files from 'public' directory
app.use(express.static('public'));

// Socket.IO connection handling
io.on('connection', (socket) => {
    console.log('Web client connected');
    
    // Send current connection status
    socket.emit('connectionStatus', isConnected);

    // Handle connect request
    socket.on('connect-to-robot', () => {
        connectToRobot();
    });

    // Handle disconnect request
    socket.on('disconnect-from-robot', () => {
        disconnectFromRobot();
    });

    // Handle G-code commands
    socket.on('send-command', (command) => {
        if (isConnected && robotClient) {
            console.log('Sending command:', command);
            robotClient.write(command + '\n');
        }
    });
});

function connectToRobot() {
    if (robotClient) {
        return;
    }

    robotClient = new net.Socket();

    robotClient.connect(ROBOT_PORT, ROBOT_IP, () => {
        console.log('Connected to robot');
        isConnected = true;
        io.emit('connectionStatus', true);
    });

    robotClient.on('data', (data) => {
        console.log('Received:', data.toString());
        io.emit('robot-response', data.toString());
    });

    robotClient.on('close', () => {
        console.log('Connection closed');
        isConnected = false;
        robotClient = null;
        io.emit('connectionStatus', false);
    });

    robotClient.on('error', (err) => {
        console.log('Connection error:', err);
        isConnected = false;
        robotClient = null;
        io.emit('connectionStatus', false);
    });
}

function disconnectFromRobot() {
    if (robotClient) {
        robotClient.destroy();
        robotClient = null;
        isConnected = false;
        io.emit('connectionStatus', false);
    }
}

http.listen(PORT, () => {
    console.log(`Server running at http://localhost:${PORT}`);
});