let workspace;
let ws = null;
let stopRequested = false;
let currentDistance = 0;
let currentLeftEncoder = 0;
let currentRightEncoder = 0;
let currentServo = 90;
let debugMode = false;

let reconnectAttempts = 0;
let maxReconnectAttempts = 20;
let reconnectDelay = 1000;
let maxReconnectDelay = 30000;
let reconnectTimer = null;
let isReconnecting = false;
let lastConnectedIP = null;
let heartbeatInterval = null;
let lastHeartbeat = Date.now();
let connectionLost = false;

let commandQueue = [];
let pendingCommands = new Map();
let commandId = 0;

function initBlockly() {
    const toolbox = {
        "kind": "categoryToolbox",
        "contents": [
            {
                "kind": "category",
                "name": "🚗 Bewegung",
                "colour": "120",
                "contents": [
                    {"kind": "block", "type": "robot_move_forward"},
                    {"kind": "block", "type": "robot_move_backward"},
                    {"kind": "block", "type": "robot_turn_degrees"},
                    {"kind": "block", "type": "robot_stop"},
                    {"kind": "block", "type": "robot_reset_encoders"},
                    {"kind": "block", "type": "robot_get_encoder"}
                ]
            },
            {
                "kind": "category",
                "name": "📏 Sensoren",
                "colour": "230",
                "contents": [
                    {"kind": "block", "type": "robot_servo"},
                    {"kind": "block", "type": "robot_get_distance"},
                    {"kind": "block", "type": "robot_if_obstacle"},
                    {"kind": "block", "type": "robot_servo_scan"},
                    {"kind": "block", "type": "robot_find_best_direction"}
                ]
            },
            {
                "kind": "category",
                "name": "🔁 Steuerung",
                "colour": "120",
                "contents": [
                    {"kind": "block", "type": "robot_repeat"},
                    {"kind": "block", "type": "robot_forever"},
                    {"kind": "block", "type": "robot_wait"}
                ]
            },
            {
                "kind": "category",
                "name": "📝 Ausgabe",
                "colour": "160",
                "contents": [
                    {"kind": "block", "type": "robot_print"},
                    {"kind": "block", "type": "robot_print_distance"}
                ]
            }
        ]
    };

    workspace = Blockly.inject('blocklyDiv', {
        toolbox: toolbox,
        grid: {
            spacing: 20,
            length: 3,
            colour: '#ccc',
            snap: true
        },
        zoom: {
            controls: true,
            wheel: true,
            startScale: 1.0,
            maxScale: 3,
            minScale: 0.3,
            scaleSpeed: 1.2
        },
        trashcan: true
    });

    loadWorkspace();
    workspace.addChangeListener(() => {
        saveWorkspace();
    });
}

function connectWebSocket(autoReconnect = false) {
    const ip = document.getElementById('espIP').value || lastConnectedIP;
    if (!ip) {
        addToConsole('Bitte ESP32 IP-Adresse eingeben', 'error');
        return;
    }

    // force close existing connection if any
    if (ws) {
        if (ws.readyState === WebSocket.CONNECTING || ws.readyState === WebSocket.OPEN) {
            if (!autoReconnect) {
                addToConsole('Schließe bestehende Verbindung...', 'info');
            }
            try {
                // prevent triggering onclose handlers
                ws.onclose = null;
                ws.onerror = null;
                ws.onmessage = null;
                ws.close();
            } catch (e) {
                console.error('Error closing WebSocket:', e);
            }
        }
        ws = null;
    }

    lastConnectedIP = ip;

    if (autoReconnect) {
        isReconnecting = true;
        reconnectAttempts++;
        updateConnectionStatus(false, `Wiederverbindung... Versuch ${reconnectAttempts}/${maxReconnectAttempts}`);
    } else {
        reconnectAttempts = 0;
        reconnectDelay = 1000;
    }

    const wsUrl = `ws://${ip}:81`;
    addToConsole(`${autoReconnect ? 'Wiederverbindung' : 'Verbindung'} zu ${wsUrl}...`);

    try {
        ws = new WebSocket(wsUrl);
        setupWebSocketHandlers();
    } catch (error) {
        addToConsole(`Verbindungsfehler: ${error.message}`, 'error');
        if (autoReconnect) {
            scheduleReconnect();
        }
    }
}

function setupWebSocketHandlers() {
    ws.onopen = function() {
        addToConsole('✅ Mit Roboter verbunden!', 'success');
        updateConnectionStatus(true);

        isReconnecting = false;
        reconnectAttempts = 0;
        reconnectDelay = 1000;
        connectionLost = false;

        startHeartbeat();

        processCommandQueue();
    };

    ws.onmessage = function(event) {
        try {
            const data = JSON.parse(event.data);
            handleWebSocketMessage(data);
        } catch (e) {
            addToConsole(`Empfangen: ${event.data}`);
        }
    };

    ws.onerror = function(error) {
        console.error('WebSocket Fehler:', error);
        addToConsole(`Verbindungsfehler aufgetreten`, 'error');

        if (!connectionLost) {
            connectionLost = true;
            updateConnectionStatus(false, 'Verbindungsfehler');
        }
    };

    ws.onclose = function(event) {
        ws = null;
        stopHeartbeat();

        if (event.wasClean) {
            addToConsole('Verbindung geschlossen', 'warning');
            updateConnectionStatus(false, 'Getrennt');
        } else {
            addToConsole(`⚠️ Verbindung unterbrochen (Code: ${event.code})`, 'error');
            updateConnectionStatus(false, 'Verbindung unterbrochen');

            if (!stopRequested && reconnectAttempts < maxReconnectAttempts) {
                scheduleReconnect();
            } else if (reconnectAttempts >= maxReconnectAttempts) {
                addToConsole(`❌ Max. Wiederverbindungsversuche (${maxReconnectAttempts}) erreicht`, 'error');
                updateConnectionStatus(false, 'Wiederverbindung fehlgeschlagen');
            }
        }
    };
}

function scheduleReconnect() {
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
    }

    const delay = Math.min(reconnectDelay * Math.pow(1.5, reconnectAttempts - 1), maxReconnectDelay);
    const seconds = Math.round(delay / 1000);

    addToConsole(`⏳ Nächster Verbindungsversuch in ${seconds} Sekunden...`, 'info');
    updateConnectionStatus(false, `Wartet ${seconds}s auf Wiederverbindung`);

    reconnectTimer = setTimeout(() => {
        // Reset stopRequested for auto-reconnect
        stopRequested = false;
        connectWebSocket(true);
    }, delay);
}

let isExecutingCode = false;

function startHeartbeat() {
    stopHeartbeat();
    lastHeartbeat = Date.now();

    heartbeatInterval = setInterval(() => {
        if (ws && ws.readyState === WebSocket.OPEN) {
            // more aggressive timeout for better dead connection detection
            const heartbeatTimeout = isExecutingCode ? 15000 : 10000;

            if (Date.now() - lastHeartbeat > heartbeatTimeout) {
                addToConsole('⚠️ Keine Antwort vom Roboter - Verbindung unterbrochen', 'warning');
                // force close to trigger reconnect
                if (ws) {
                    ws.close();
                }
                return;
            }

            // send ping more frequently
            if (!isExecutingCode || Date.now() - lastHeartbeat > 2000) {
                try {
                    sendCommand({ type: 'ping' }, false);
                } catch (e) {
                    console.error('Error sending ping:', e);
                }
            }
        }
    }, 2000); // check more frequently (every 2 seconds)
}

function stopHeartbeat() {
    if (heartbeatInterval) {
        clearInterval(heartbeatInterval);
        heartbeatInterval = null;
    }
}

function sendCommand(command, queue = true) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        if (queue && command.type !== 'ping') {
            commandQueue.push(command);
            addToConsole(`⏳ Befehl in Warteschlange (${commandQueue.length} wartend)`, 'info');
        }
        return false;
    }

    try {
        if (command.type === 'command') {
            command.id = ++commandId;
            pendingCommands.set(command.id, {
                command: command,
                timestamp: Date.now(),
                retries: 0
            });

            setTimeout(() => {
                if (pendingCommands.has(command.id)) {
                    pendingCommands.delete(command.id);
                    if (command.action !== 'differential' || commandId < 100) {
                        addToConsole(`⚠️ Befehl ${command.id} Timeout`, 'warning');
                    }
                }
            }, 8000);
        }

        ws.send(JSON.stringify(command));
        return command.id || true;
    } catch (error) {
        addToConsole(`Fehler beim Senden: ${error.message}`, 'error');
        if (queue) {
            commandQueue.push(command);
        }
        return false;
    }
}

function processCommandQueue() {
    if (commandQueue.length > 0) {
        addToConsole(`📤 Sende ${commandQueue.length} wartende Befehle...`, 'info');
        while (commandQueue.length > 0) {
            const command = commandQueue.shift();
            sendCommand(command, false);
        }
    }
}

function handleWebSocketMessage(data) {
    if (data.type === 'pong' || data.type === 'sensor_data') {
        lastHeartbeat = Date.now();
    }

    if (data.type === 'sensor_data') {
        currentDistance = data.distance || 0;
        currentLeftEncoder = data.left_encoder || 0;
        currentRightEncoder = data.right_encoder || 0;
        currentServo = data.servo_position || 90;

        updateEncoderDisplay(currentLeftEncoder, currentRightEncoder);

        if (data.distance > 0 && Math.random() > 0.95) {
            if (debugMode) {
                addToConsole(`📏 Abstand: ${data.distance}cm | 🎯 Encoder L:${currentLeftEncoder} R:${currentRightEncoder}`);
            } else {
                addToConsole(`📏 Abstand: ${data.distance}cm`);
            }
        }
    } else if (data.type === 'command_ack') {
        if (data.id) {
            if (pendingCommands.has(data.id)) {
                pendingCommands.delete(data.id);
            }

            const ackResolver = commandAckPromises.get(data.id);
            if (ackResolver) {
                ackResolver(data.success !== false);
            }
        }

        if (!data.success) {
            addToConsole('❌ Befehl fehlgeschlagen!', 'error');
        }
    } else if (data.type === 'status') {
        if (!data.arduino_connected) {
            addToConsole('⚠️ Arduino nicht verbunden! I2C-Verkabelung prüfen.', 'error');
        }
    }
}

function updateEncoderDisplay(leftCount, rightCount) {
    const status = document.getElementById('encoderStatus');
    if (!status) return;

    status.style.display = 'inline';

    const leftMM = Math.round(leftCount * 9.72);
    const rightMM = Math.round(rightCount * 9.72);

    status.innerHTML = `<span style="color: #2196F3; font-weight: bold;">🎯 L:${leftCount} (${leftMM}mm) R:${rightCount} (${rightMM}mm)</span>`;
}

function toggleDebug() {
    debugMode = !debugMode;

    if (debugMode) {
        addToConsole('🐛 Debug-Modus aktiviert', 'info');
    } else {
        addToConsole('🐛 Debug-Modus deaktiviert', 'info');
    }
}

function updateConnectionStatus(connected, customMessage = null) {
    const status = document.getElementById('connectionStatus');
    const connectBtn = document.getElementById('connectBtn');
    const disconnectBtn = document.getElementById('disconnectBtn');

    if (customMessage) {
        status.textContent = customMessage;
        status.className = 'connection-status ' + (connected ? 'connected' : 'disconnected');
    } else if (connected) {
        status.className = 'connection-status connected';
        status.textContent = 'Verbunden';
    } else {
        status.className = 'connection-status disconnected';
        status.textContent = 'Getrennt';
    }

    // Toggle button visibility based on connection state
    if (connectBtn && disconnectBtn) {
        if (connected) {
            connectBtn.style.display = 'none';
            disconnectBtn.style.display = 'inline-block';
        } else {
            connectBtn.style.display = 'inline-block';
            disconnectBtn.style.display = 'none';
        }
    }
}

// rate limiting - reduce to minimum
let lastCommandTime = 0;
const MIN_COMMAND_INTERVAL = 10;  // Reduced from 50ms to 10ms
let commandAckPromises = new Map();

async function waitForCommandAck(commandId, timeout = 2000) {
    return new Promise((resolve) => {
        const timer = setTimeout(() => {
            commandAckPromises.delete(commandId);
            resolve(false);
        }, timeout);

        commandAckPromises.set(commandId, (success) => {
            clearTimeout(timer);
            commandAckPromises.delete(commandId);
            resolve(success);
        });
    });
}

async function sendCommandWithRateLimit(command) {
    const now = Date.now();
    const timeSinceLastCommand = now - lastCommandTime;
    if (timeSinceLastCommand < MIN_COMMAND_INTERVAL) {
        await wait(MIN_COMMAND_INTERVAL - timeSinceLastCommand);
    }
    lastCommandTime = Date.now();

    const result = sendCommand(command, false);
    if (result) {
        // Don't wait for ACK on movement commands - Arduino handles them autonomously
        // Only wait for ACK on critical commands if needed
        return true;
    }
    return false;
}

// motor calibration
const MOTOR_CALIBRATION = {
    leftMultiplier: 1.0,
    rightMultiplier: 1.0
};

async function moveRobot(leftSpeed, rightSpeed, duration) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        throw new Error('Nicht verbunden');
    }

    const calibratedLeft = Math.round(leftSpeed * MOTOR_CALIBRATION.leftMultiplier);
    const calibratedRight = Math.round(rightSpeed * MOTOR_CALIBRATION.rightMultiplier);

    // collision protection
    const isMovingForward = calibratedLeft > 0 && calibratedRight > 0;
    if (isMovingForward) {
        const distance = await getDistance();
        if (distance < 5) {
            addToConsole('⚠️ Hindernis zu nah! Bewegung gestoppt.', 'warning');
            await stopRobot();
            return;
        }
    }

    const command = {
        type: 'command',
        action: 'differential',
        leftSpeed: Math.max(-255, Math.min(255, calibratedLeft)),
        rightSpeed: Math.max(-255, Math.min(255, calibratedRight))
    };

    const commandSentTime = Date.now();
    await sendCommandWithRateLimit(command);

    const commandDelay = Date.now() - commandSentTime;
    const actualDuration = Math.max(0, duration - commandDelay);

    addToConsole(`🚗 Bewege: L=${calibratedLeft}, R=${calibratedRight} für exakt ${duration}ms`);

    const startTime = Date.now();
    let lastUpdate = startTime;
    let lastCollisionCheck = startTime;

    while (Date.now() - startTime < actualDuration) {
        if (window.stopRequested) {
            await stopRobot();
            return;
        }

        if (isMovingForward && Date.now() - lastCollisionCheck > 100) {
            const distance = await getDistance();
            if (distance < 5) {
                addToConsole('⚠️ Kollision erkannt! Notbremsung!', 'warning');
                await stopRobot();
                await moveRobot(-100, -100, 200);
                return;
            }
            lastCollisionCheck = Date.now();
        }

        if (Date.now() - lastUpdate > 2000) {
            lastHeartbeat = Date.now();
            lastUpdate = Date.now();
        }

        await wait(5);
    }

    if (!window.stopRequested) {
        await stopRobot();
    }

}

async function stopRobot() {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;

    const command = {
        type: 'command',
        action: 'differential',
        leftSpeed: 0,
        rightSpeed: 0
    };

    await sendCommandWithRateLimit(command);
    addToConsole('🛑 Roboter gestoppt');

}

async function turnDegrees(direction, degrees, speed) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        throw new Error('Nicht verbunden');
    }

    // simplified calibration - direct mapping without complex physics
    // adjust these values based on your robot's actual behavior
    const TURN_CALIBRATION = {
        // speed -> milliseconds per degree (simplified approach)
        100: 3.5,  // ms per degree at speed 100
        150: 2.3,  // ms per degree at speed 150
        200: 1.7,  // ms per degree at speed 200
        255: 1.4   // ms per degree at speed 255
    };

    // get calibration for closest speed
    const speeds = Object.keys(TURN_CALIBRATION).map(Number).sort((a, b) => a - b);
    const closestSpeed = speeds.reduce((prev, curr) =>
        Math.abs(curr - speed) < Math.abs(prev - speed) ? curr : prev
    );
    const msPerDegree = TURN_CALIBRATION[closestSpeed];

    // simple duration calculation
    const duration = Math.round(degrees * msPerDegree);

    // debug output
    console.log(`Turn Debug: ${degrees}° turn at speed ${speed}`);
    console.log(`  - Using speed ${closestSpeed} calibration`);
    console.log(`  - Ms per degree: ${msPerDegree}`);
    console.log(`  - Total duration: ${duration}ms`);

    // set motor speeds
    let leftSpeed, rightSpeed;
    if (direction === 'left') {
        leftSpeed = Math.round(-speed * MOTOR_CALIBRATION.leftMultiplier);
        rightSpeed = Math.round(speed * MOTOR_CALIBRATION.rightMultiplier);
    } else {
        leftSpeed = Math.round(speed * MOTOR_CALIBRATION.leftMultiplier);
        rightSpeed = Math.round(-speed * MOTOR_CALIBRATION.rightMultiplier);
    }

    const command = {
        type: 'command',
        action: 'differential',
        leftSpeed: leftSpeed,
        rightSpeed: rightSpeed
    };

    const commandSentTime = Date.now();
    await sendCommandWithRateLimit(command);

    const commandDelay = Date.now() - commandSentTime;
    const actualDuration = Math.max(0, duration - commandDelay);

    addToConsole(`🔄 Drehe ${direction === 'left' ? 'links' : 'rechts'} ${degrees}° für ${duration}ms`);

    const startTime = Date.now();

    while (Date.now() - startTime < actualDuration) {
        if (window.stopRequested) {
            await stopRobot();
            return;
        }
        await wait(5);
    }

    if (!window.stopRequested) {
        await stopRobot();
    }
}

async function setServo(angle) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        throw new Error('Nicht verbunden');
    }

    angle = Math.max(0, Math.min(180, angle));
    const command = {
        type: 'command',
        action: 'servo',
        angle: angle
    };

    sendCommand(command, false);  // Don't wait, just send
    addToConsole(`📷 Servo auf ${angle}° gestellt`);
    // No wait - servo moves on its own
}

// encoder-based movement with explicit directions
async function moveDistance(distance, direction, speed) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        throw new Error('Nicht verbunden');
    }

    const command = {
        type: 'command',
        action: 'moveDistance',
        distance: Math.abs(distance),
        direction: direction,
        speed: speed
    };

    sendCommand(command, false);
    const directionText = direction === 'forward' ? 'vorwärts' : 'rückwärts';
    addToConsole(`🎯 Fahre ${Math.abs(distance)}mm ${directionText} mit Geschw. ${speed}`);
}

async function turnPrecise(degrees, direction, speed) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        throw new Error('Nicht verbunden');
    }

    const command = {
        type: 'command',
        action: 'rotateDegrees',
        degrees: Math.abs(degrees),
        direction: direction,
        speed: speed
    };

    sendCommand(command, false);
    const directionText = direction === 'left' ? 'links' : 'rechts';
    addToConsole(`🎯 Drehe ${Math.abs(degrees)}° ${directionText} mit Geschw. ${speed}`);
}

async function resetEncoders() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        throw new Error('Nicht verbunden');
    }

    const command = {
        type: 'command',
        action: 'resetEncoders'
    };

    await sendCommandWithRateLimit(command);
    currentLeftEncoder = 0;
    currentRightEncoder = 0;
    updateEncoderDisplay(0, 0);
    addToConsole('🔄 Encoder zurückgesetzt');
}

let lastDistanceCheck = 0;

async function getDistance() {
    // Request distance reading from Arduino only when needed
    if (ws && ws.readyState === WebSocket.OPEN) {
        const command = {
            type: 'command',
            action: 'read_distance'
        };
        sendCommand(command, false);
    }

    // Small delay to allow reading
    await wait(50);

    return currentDistance || 0;
}

async function wait(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// servo scan functions
async function performServoScan(thresholdDistance = 30) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        return [];
    }

    addToConsole(`🔍 Schnellscan...`);

    const scanResults = [];
    const angles = [0, 45, 90, 135, 180];

    try {
        for (let angle of angles) {
            const servoCommand = {
                type: 'command',
                action: 'servo',
                angle: angle
            };

            sendCommand(servoCommand, false);

            await wait(200);

            const distance = currentDistance || 0;
            scanResults.push({ angle, distance });
            addToConsole(`  ${angle}°: ${distance}cm`);
        }

        const centerCommand = {
            type: 'command',
            action: 'servo',
            angle: 90
        };
        sendCommand(centerCommand, false);
        await wait(200);

        return scanResults;

    } catch (error) {
        addToConsole(`❌ Scan-Fehler: ${error.message}`, 'error');
        return [];
    }
}

async function findBestDirection() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        return 'geradeaus';
    }

    addToConsole('🧭 Suche beste Richtung...');

    try {
        const scanResults = [];
        const directions = {
            0: 'scharf_links',
            45: 'links',
            90: 'geradeaus',
            135: 'rechts',
            180: 'scharf_rechts'
        };

        for (let angle of Object.keys(directions).map(Number)) {
            const servoCommand = {
                type: 'command',
                action: 'servo',
                angle: angle
            };
            sendCommand(servoCommand, false);

            await wait(200);

            const distance = currentDistance || 0;
            scanResults.push({ angle, distance, direction: directions[angle] });
            addToConsole(`  ${directions[angle]}: ${distance}cm`);
        }

        const centerCommand = {
            type: 'command',
            action: 'servo',
            angle: 90
        };
        sendCommand(centerCommand, false);

        if (scanResults.length > 0) {
            let bestResult = scanResults[0];
            for (let result of scanResults) {
                if (result.distance > bestResult.distance) {
                    bestResult = result;
                }
            }

            addToConsole(`➡️ Beste Richtung: ${bestResult.direction} (${bestResult.distance}cm frei)`);
            return bestResult.direction;
        }
        return 'geradeaus';

    } catch (error) {
        addToConsole(`❌ Fehler bei Richtungssuche: ${error.message}`, 'error');
        return 'geradeaus';
    }
}

// calibration routine for precise turning
async function calibrateTurning(testSpeed = 150) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht mit Roboter verbunden!', 'error');
        return;
    }

    addToConsole('=== 🎯 DREHKALIBRIERUNG START ===', 'success');
    addToConsole('1. Platziere den Roboter auf einer ebenen Fläche');
    addToConsole('2. Markiere die Startposition (z.B. mit Klebeband)');
    addToConsole('3. Beobachte die tatsächlichen Drehwinkel');
    await wait(3000);

    const testAngles = [90, 180, 360];

    for (let targetAngle of testAngles) {
        addToConsole(`\n📐 Test: ${targetAngle}° Drehung bei Geschwindigkeit ${testSpeed}`);
        addToConsole('Beobachte die tatsächliche Drehung...');
        await wait(2000);

        // perform test turn
        await turnDegrees('right', targetAngle, testSpeed);
        await wait(1000);

        addToConsole(`Ziel: ${targetAngle}° - Notiere die tatsächliche Drehung`);
        addToConsole('Der Roboter kehrt zur Startposition zurück in 5 Sekunden...');
        await wait(5000);

        // return to start position for next test
        await turnDegrees('left', targetAngle, testSpeed);
        await wait(2000);
    }

    addToConsole('\n=== ✅ KALIBRIERUNG ABGESCHLOSSEN ===', 'success');
    addToConsole('📝 Kalibrierungshinweise:');
    addToConsole('- Dreht zu weit: Verringere ms/degree Wert');
    addToConsole('- Dreht zu wenig: Erhöhe ms/degree Wert');
    addToConsole('- Passe die Werte in TURN_CALIBRATION an (Zeile 494-499)');
    addToConsole('- Aktuelle Werte: 100->3.5ms, 150->2.3ms, 200->1.7ms, 255->1.4ms');
}

function print(text) {
    addToConsole(`💬 ${text}`, 'info');
}

async function runCode() {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Bitte zuerst mit Roboter verbinden!', 'error');
        return;
    }

    window.stopRequested = false;
    isExecutingCode = true;

    commandId = 0;
    pendingCommands.clear();
    commandAckPromises.clear();

    addToConsole('▶️ Programm wird ausgeführt...', 'success');

    try {
        let code;

        if (window.generateRobotCode) {
            code = window.generateRobotCode(workspace);
            console.log('Verwende benutzerdefinierten Code-Generator');
        } else {
            try {
                code = Blockly.JavaScript.workspaceToCode(workspace);
                console.log('Verwende Blockly.JavaScript Generator');
            } catch (e) {
                console.error('Blockly.JavaScript fehlgeschlagen:', e);
                addToConsole('Code-Generierung fehlgeschlagen!', 'error');
                return;
            }
        }

        if (!code || code.trim() === '') {
            addToConsole('Keine Blöcke zum Ausführen!', 'warning');
            return;
        }

        addToConsole('Generierter Code:', 'info');
        console.log(code);

        sendCommand({ type: 'ping' }, false);

        const asyncWrapper = `
            (async () => {
                try {
                    ${code}
                } catch (innerError) {
                    if (innerError.message !== 'Nicht verbunden') {
                        console.error('Laufzeitfehler:', innerError);
                        throw innerError;
                    }
                }
            })()
        `;

        await eval(asyncWrapper);

        await wait(100);

        if (!window.stopRequested && ws && ws.readyState === WebSocket.OPEN) {
            addToConsole('✅ Programm abgeschlossen!', 'success');
        }
    } catch (error) {
        addToConsole(`❌ Fehler: ${error.message}`, 'error');
        console.error('Ausführungsfehler:', error);
    } finally {
        if (ws && ws.readyState === WebSocket.OPEN) {
            await stopRobot();
        }

        window.stopRequested = false;
        isExecutingCode = false;

        if (ws && ws.readyState === WebSocket.OPEN) {
            sendCommand({ type: 'ping' }, false);
        }
    }
}

async function stopCode() {
    window.stopRequested = true;
    isExecutingCode = false;

    // immediate stop
    const emergencyStop = {
        type: 'command',
        action: 'differential',
        leftSpeed: 0,
        rightSpeed: 0
    };

    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(emergencyStop));
        addToConsole('🛑 NOTFALL-STOPP ausgeführt!', 'error');
    }

    await wait(10);

    addToConsole('⏹️ Programm gestoppt!', 'warning');
}

function addToConsole(message, type = 'normal') {
    const consoleDiv = document.getElementById('console');
    const timestamp = new Date().toLocaleTimeString();
    const entry = document.createElement('div');

    const colors = {
        'normal': '#00ff00',
        'error': '#ff4444',
        'warning': '#ffaa00',
        'success': '#44ff44',
        'info': '#44aaff'
    };

    entry.style.color = colors[type] || colors['normal'];
    entry.textContent = `[${timestamp}] ${message}`;
    consoleDiv.appendChild(entry);
    consoleDiv.scrollTop = consoleDiv.scrollHeight;

    // keep last 100 messages
    while (consoleDiv.children.length > 100) {
        consoleDiv.removeChild(consoleDiv.children[0]);
    }
}

function sendManualCommand() {
    const input = document.getElementById('commandInput');
    const command = input.value.trim();

    if (!command) return;

    if (!ws || ws.readyState !== WebSocket.OPEN) {
        addToConsole('Nicht verbunden!', 'error');
        return;
    }

    if (command.startsWith('M ')) {
        const parts = command.split(' ');
        if (parts.length === 3) {
            const cmd = {
                type: 'command',
                action: 'differential',
                leftSpeed: parseInt(parts[1]),
                rightSpeed: parseInt(parts[2])
            };
            sendCommand(cmd);
            addToConsole(`Gesendet: ${command}`);
        }
    } else if (command.startsWith('S ')) {
        const angle = parseInt(command.substring(2));
        const cmd = {
            type: 'command',
            action: 'servo',
            angle: angle
        };
        sendCommand(cmd);
        addToConsole(`Gesendet: Servo ${angle}°`);
    } else {
        addToConsole(`Unbekannter Befehl: ${command}`, 'error');
    }

    input.value = '';
}

function saveWorkspace() {
    const xml = Blockly.Xml.workspaceToDom(workspace);
    const xmlText = Blockly.Xml.domToText(xml);
    localStorage.setItem('robotCarWorkspace', xmlText);
}

function loadWorkspace() {
    const saved = localStorage.getItem('robotCarWorkspace');
    if (saved) {
        const xml = Blockly.Xml.textToDom(saved);
        Blockly.Xml.clearWorkspaceAndLoadFromXml(xml, workspace);
    } else {
        loadDefaultProgram();
    }
}

function loadDefaultProgram() {
    const xml = `
    <xml>
        <block type="robot_print" x="20" y="20">
            <field name="TEXT">Hindernisvermeidung startet!</field>
            <next>
                <block type="robot_servo">
                    <field name="ANGLE">90</field>
                    <next>
                        <block type="robot_forever">
                            <statement name="DO">
                                <block type="robot_if_obstacle">
                                    <field name="DISTANCE">15</field>
                                    <statement name="DO">
                                        <block type="robot_print">
                                            <field name="TEXT">Hindernis erkannt!</field>
                                            <next>
                                                <block type="robot_stop">
                                                    <next>
                                                        <block type="robot_move_backward">
                                                            <field name="SPEED">150</field>
                                                            <field name="DURATION">0.5</field>
                                                            <next>
                                                                <block type="robot_turn_degrees">
                                                                    <field name="DEGREES">90</field>
                                                                    <field name="DIRECTION">RIGHT</field>
                                                                </block>
                                                            </next>
                                                        </block>
                                                    </next>
                                                </block>
                                            </next>
                                        </block>
                                    </statement>
                                    <statement name="ELSE">
                                        <block type="robot_move_forward">
                                            <field name="SPEED">150</field>
                                            <field name="DURATION">0.5</field>
                                        </block>
                                    </statement>
                                </block>
                            </statement>
                        </block>
                    </next>
                </block>
            </next>
        </block>
    </xml>`;

    const dom = Blockly.Xml.textToDom(xml);
    Blockly.Xml.clearWorkspaceAndLoadFromXml(dom, workspace);
}

window.addEventListener('load', function() {
    function startApp() {
        initBlockly();
        addToConsole('🤖 BKTM Auto - Block-Programmierung bereit!', 'success');
        addToConsole('📡 ESP32 IP eingeben und auf Verbinden klicken', 'info');
        updateBatteryDisplay(0);

        if (lastConnectedIP) {
            setTimeout(() => {
                addToConsole('🔄 Versuche automatische Wiederverbindung...', 'info');
                connectWebSocket(true);
            }, 1000);
        }
    }

    if (window.robotBlocksInitialized) {
        startApp();
    } else {
        window.addEventListener('robotBlocksReady', startApp);
        setTimeout(function() {
            if (!window.robotBlocksInitialized) {
                console.log('Erzwinge Initialisierung nach Timeout');
                startApp();
            }
        }, 1000);
    }
});

// Enter-Taste in Eingabefeldern
document.addEventListener('DOMContentLoaded', function() {
    const commandInput = document.getElementById('commandInput');
    if (commandInput) {
        commandInput.addEventListener('keypress', function(e) {
            if (e.key === 'Enter') {
                sendManualCommand();
            }
        });
    }

    const espIP = document.getElementById('espIP');
    if (espIP) {
        espIP.addEventListener('keypress', function(e) {
            if (e.key === 'Enter') {
                connectWebSocket();
            }
        });
        // Standard-IP setzen
        espIP.value = '192.168.4.1';

        // Lade gespeicherte IP falls vorhanden
        const savedIP = localStorage.getItem('lastConnectedIP');
        if (savedIP) {
            espIP.value = savedIP;
            lastConnectedIP = savedIP;
        }
    }
});

// Speichere IP bei erfolgreicher Verbindung
window.addEventListener('beforeunload', function() {
    if (lastConnectedIP) {
        localStorage.setItem('lastConnectedIP', lastConnectedIP);
    }
});

// Disconnect function to properly clean up connection
function disconnectWebSocket() {
    stopRequested = true; // prevent auto-reconnect

    // Clear any pending reconnect timers
    if (reconnectTimer) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
    }

    // Stop heartbeat
    stopHeartbeat();

    // Close WebSocket if open
    if (ws) {
        try {
            // Remove handlers to prevent triggering reconnect
            ws.onclose = null;
            ws.onerror = null;
            ws.onmessage = null;
            ws.onopen = null;

            if (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING) {
                ws.close(1000, 'User requested disconnect');
            }
        } catch (e) {
            console.error('Error closing WebSocket:', e);
        }
        ws = null;
    }

    // Clear command queue
    commandQueue = [];
    pendingCommands.clear();

    // Update UI
    updateConnectionStatus(false, 'Getrennt');
    addToConsole('📴 Verbindung getrennt', 'info');

    // Reset connection state
    isReconnecting = false;
    connectionLost = false;
    reconnectAttempts = 0;
}

// Manual reconnect button
function manualReconnect() {
    // First ensure clean disconnection
    disconnectWebSocket();

    // Reset reconnect state
    stopRequested = false;
    reconnectAttempts = 0;
    reconnectDelay = 1000;

    // Small delay to ensure clean state
    setTimeout(() => {
        connectWebSocket(false);
    }, 100);
}

// Page Visibility Handler - detect when user leaves/returns to page
document.addEventListener('visibilitychange', () => {
    if (document.hidden) {
        // Page is hidden (user switched tabs or minimized browser)
        console.log('Page hidden - may affect WebSocket connection');
    } else {
        // Page is visible again
        console.log('Page visible again - checking connection');

        // If we had an active connection, check if it's still alive
        if (ws && ws.readyState === WebSocket.OPEN && !stopRequested) {
            // Send a ping to check if connection is still alive
            try {
                sendCommand({ type: 'ping' }, false);
            } catch (e) {
                console.log('Connection check failed, may need reconnection');
            }
        } else if (!ws && lastConnectedIP && !stopRequested && !isReconnecting) {
            // If we lost connection while page was hidden, try to reconnect
            addToConsole('🔄 Seite wieder aktiv - versuche Wiederverbindung...', 'info');
            connectWebSocket(true);
        }
    }
});

// Handle browser refresh/close - try to cleanly close WebSocket
window.addEventListener('beforeunload', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
        try {
            ws.close(1000, 'Page unloading');
        } catch (e) {
            // Ignore errors during page unload
        }
    }
});