/**
 * ROS Bridge Connection Manager
 * 
 * This handles talking to ROS over WebSocket. It connects to rosbridge,
 * publishes commands to motors and relays, and listens for status updates.
 */
const ROSBridge = {
    ros: null,
    motor1Pub: null,
    motor2Pub: null,
    motor1SpeedPub: null,
    motor2SpeedPub: null,
    relayPub: null,
    sequencePub: null,
    estopPub: null,
    executionStatePub: null,
    executionStateSub: null,
    executionStopRequestPub: null,
    executionStopRequestSub: null,
    executionStateInterval: null,
    activeBlocksPub: null,
    activeBlocksSub: null,
    motor1StatusSub: null,
    motor2StatusSub: null,
    arduinoStatusSub: null,
    relayStatusSubs: [],
    potentiometerSub: null,
    potDebugSub: null,
    motorSpeedSetpointSub: null,
    motor1SpeedEchoSub: null,
    motor2SpeedEchoSub: null,
    serialLogSub: null,
    motorStatus: {}, // Keep track of each motor's status
    /** Last parsed Arduino connection status { connected, port, baud } */
    arduinoStatus: null,
    /** relayId -> { relay_id, state, lastChangeMs, lastPayload } */
    relayStates: { 1: null, 2: null, 3: null, 4: null },
    lastPotRaw: null,
    lastPotAtMs: 0,
    lastPotDebug: null,
    lastPotDebugAtMs: 0,
    /** Latest Float32 from /motor_speed/setpoint when present */
    motorSpeedSetpoint: null,
    /** Last speed msgs observed on /motorN/speed (bus echo / commanded) */
    topicMotorSpeed: { 1: null, 2: null },
    /** topicName -> { lastT, hz, count, windowStart, windowCount } */
    telemetryMeta: {},
    /** Last raw message string per topic (for Control Center copy) */
    telemetryLastRaw: {},
    isConnected: false,
    wsSessionStartMs: 0,
    wsReconnectCount: 0,
    /** Shared execution state across all clients: { running, projectId, clientId } */
    executionSyncState: { running: false, projectId: null, clientId: null },
    dynamicSubscriptions: new Map(), // For subscribing to topics on the fly
    /** motorId (1|2) -> { topicName, topic, handler } for continuous motor speed from topic */
    motorSpeedTopicSubscriptions: new Map(),
    messageThrottle: new Map(), // Slow down updates that come too fast
    connectionQuality: { latency: 0, messageRate: 0, lastMessageTime: null }, // How's the connection doing?
    parseCache: new Map(), // Cache parsed JSON so we don't parse the same thing twice

    recordTelemetry(topicName, rawPayload = null) {
        const now = performance.now();
        let m = this.telemetryMeta[topicName];
        if (!m) {
            m = this.telemetryMeta[topicName] = { lastT: 0, hz: 0, count: 0, windowStart: now, windowCount: 0 };
        }
        m.count++;
        m.lastT = now;
        m.windowCount++;
        const dt = now - m.windowStart;
        if (dt >= 1000) {
            m.hz = m.windowCount / (dt / 1000);
            m.windowCount = 0;
            m.windowStart = now;
        }
        if (rawPayload !== null && rawPayload !== undefined) {
            this.telemetryLastRaw[topicName] = typeof rawPayload === 'string' ? rawPayload : JSON.stringify(rawPayload);
        }
    },

    getTelemetryAgeSec(topicName) {
        const m = this.telemetryMeta[topicName];
        if (!m || !m.lastT) return null;
        return (performance.now() - m.lastT) / 1000;
    },

    /**
     * Parse JSON safely, with caching to avoid parsing the same thing twice
     * @param {string} jsonString - The JSON string to parse
     * @param {string} cacheKey - Optional key to cache the result
     * @returns {Object|null} - The parsed object, or null if something went wrong
     */
    safeJsonParse(jsonString, cacheKey = null) {
        if (!jsonString || typeof jsonString !== 'string') {
            return null;
        }
        
        // Check if we've already parsed this exact string
        if (cacheKey && this.parseCache.has(cacheKey)) {
            const cached = this.parseCache.get(cacheKey);
            if (cached.jsonString === jsonString) {
                return cached.parsed;
            }
        }
        
        try {
            const parsed = JSON.parse(jsonString);
            
            // Save it in the cache so we don't have to parse it again
            if (cacheKey) {
                // Don't let the cache get too big - remove old entries if needed
                if (this.parseCache.size > 100) {
                    const firstKey = this.parseCache.keys().next().value;
                    this.parseCache.delete(firstKey);
                }
                this.parseCache.set(cacheKey, { jsonString, parsed });
            }
            
            return parsed;
        } catch (e) {
            console.error('[ROS] JSON parse error:', e, 'String:', jsonString.substring(0, 100));
            return null;
        }
    },
    
    /**
     * Connect to ROS Bridge over WebSocket
     */
    init() {
        this.ros = new ROSLIB.Ros({
            url: Config.ROS_BRIDGE_URL
        });

        this.ros.on('connection', () => {
            UIUtils.log('[ROS] Connected to ROS Bridge', 'success');
            this.isConnected = true;
            this.wsSessionStartMs = Date.now();
            UIUtils.updateRosStatus(true);
            this.initializePublishers();
            this.initializeSubscribers();
            if (typeof ControlCenter !== 'undefined') ControlCenter.onRosConnected();
        });

        this.ros.on('error', (error) => {
            UIUtils.log('[ROS] Connection error: ' + error, 'error');
            this.isConnected = false;
            UIUtils.updateRosStatus(false);
        });

        this.ros.on('close', () => {
            UIUtils.log('[ROS] Connection closed', 'error');
            this.isConnected = false;
            this.wsReconnectCount = (this.wsReconnectCount || 0) + 1;
            UIUtils.updateRosStatus(false);
            if (typeof ControlCenter !== 'undefined') ControlCenter.onRosDisconnected();

            // Clean up any subscriptions we made
            this.cleanupDynamicSubscriptions();

            // Try to reconnect after a short delay
            setTimeout(() => {
                this.init();
            }, Config.ROS_RECONNECT_DELAY);
        });
        
        // When this tab closes, if we were the executor, try to publish stopped so other clients update
        if (typeof window !== 'undefined') {
            window.addEventListener('beforeunload', () => {
                if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.isExecuting) {
                    this.stopExecutionStateHeartbeat();
                    const projectId = (typeof StorageManager !== 'undefined' && StorageManager.getCurrentProject()) ? StorageManager.getCurrentProject().id : null;
                    try { this.publishExecutionState(false, projectId); } catch (e) {}
                }
            });
        }
    },
    
    /**
     * Set up publishers so we can send commands to motors and relays
     */
    initializePublishers() {
        this.motor1Pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor1/command',
            messageType: 'std_msgs/Int32'
        });
        
        this.motor2Pub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor2/command',
            messageType: 'std_msgs/Int32'
        });
        
        this.motor1SpeedPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor1/speed',
            messageType: 'std_msgs/Float32'
        });
        
        this.motor2SpeedPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor2/speed',
            messageType: 'std_msgs/Float32'
        });
        
        this.relayPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/relay/command',
            messageType: 'std_msgs/String'
        });
        
        this.sequencePub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sequence/execute',
            messageType: 'std_msgs/String'
        });
        
        this.estopPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/estop',
            messageType: 'std_msgs/String'
        });
        
        this.executionStatePub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/assembly_line/execution_state',
            messageType: 'std_msgs/String'
        });
        
        this.executionStopRequestPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/assembly_line/execution_stop_request',
            messageType: 'std_msgs/String'
        });
        
        this.activeBlocksPub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/assembly_line/active_blocks',
            messageType: 'std_msgs/String'
        });
        
        UIUtils.log('[ROS] Publishers initialized', 'success');
    },
    
    /**
     * Publish active blocks state so desktop (non-executor) can show playback panel
     * @param {number[]} blockIds - Currently executing block IDs
     * @param {number} totalElapsed - Total execution elapsed seconds
     * @param {Object} blockElapsed - Map of blockId -> elapsed seconds
     */
    publishActiveBlocksState(blockIds, totalElapsed, blockElapsed) {
        if (!this.isConnected || !this.activeBlocksPub) return;
        try {
            const payload = {
                blockIds: blockIds || [],
                totalElapsed: totalElapsed || 0,
                blockElapsed: blockElapsed || {}
            };
            this.activeBlocksPub.publish(new ROSLIB.Message({ data: JSON.stringify(payload) }));
        } catch (e) {
            console.error('[ROS] publishActiveBlocksState error:', e);
        }
    },
    
    /**
     * Get or create a stable client ID for this tab (used for execution sync across instances)
     */
    getClientId() {
        try {
            let id = typeof sessionStorage !== 'undefined' && sessionStorage.getItem('assembly_line_client_id');
            if (!id) {
                id = 'xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx'.replace(/[xy]/g, (c) => {
                    const r = Math.random() * 16 | 0;
                    const v = c === 'y' ? (Math.random() * 4 | 8) : (Math.random() * 16 | 0);
                    return v.toString(16);
                });
                if (typeof sessionStorage !== 'undefined') sessionStorage.setItem('assembly_line_client_id', id);
            }
            return id;
        } catch (e) {
            return 'client-' + Math.random().toString(36).slice(2, 11);
        }
    },
    
    /**
     * Publish execution state so all clients (main + remote) stay in sync
     * @param {boolean} running - Whether playback is running
     * @param {string|null} projectId - Current project id when running
     */
    publishExecutionState(running, projectId) {
        if (!this.isConnected || !this.executionStatePub) return;
        try {
            this.executionSyncState = {
                running: !!running,
                projectId: running ? (projectId || null) : null,
                clientId: this.getClientId()
            };
            const msg = new ROSLIB.Message({
                data: JSON.stringify({
                    running: this.executionSyncState.running,
                    projectId: this.executionSyncState.projectId,
                    clientId: this.executionSyncState.clientId
                })
            });
            this.executionStatePub.publish(msg);
            if (typeof window.onExecutionStateUpdate === 'function') {
                window.onExecutionStateUpdate(this.executionSyncState);
            }
        } catch (e) {
            console.error('[ROS] publishExecutionState error:', e);
        }
    },
    
    /**
     * Request that the current executor stop playback (used when another client presses Stop)
     */
    publishStopRequest() {
        if (!this.isConnected || !this.executionStopRequestPub) return;
        try {
            this.executionStopRequestPub.publish(new ROSLIB.Message({ data: '{}' }));
        } catch (e) {
            console.error('[ROS] publishStopRequest error:', e);
        }
    },
    
    /**
     * Start periodic re-publish of execution state so late-joining clients see current state
     */
    startExecutionStateHeartbeat(projectId) {
        this.stopExecutionStateHeartbeat();
        const self = this;
        this.executionStateInterval = setInterval(() => {
            if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.isExecuting) {
                self.publishExecutionState(true, projectId);
            } else {
                self.stopExecutionStateHeartbeat();
            }
        }, 1500);
    },
    
    stopExecutionStateHeartbeat() {
        if (this.executionStateInterval) {
            clearInterval(this.executionStateInterval);
            this.executionStateInterval = null;
        }
    },
    
    /**
     * Set up subscribers so we can listen for status updates from motors
     */
    initializeSubscribers() {
        const dropSub = (sub) => {
            if (sub) try { sub.unsubscribe(); } catch (e) { /* ignore */ }
        };
        dropSub(this.motor1StatusSub);
        dropSub(this.motor2StatusSub);
        dropSub(this.arduinoStatusSub);
        this.relayStatusSubs.forEach(dropSub);
        this.relayStatusSubs = [];
        dropSub(this.potentiometerSub);
        dropSub(this.potDebugSub);
        dropSub(this.motorSpeedSetpointSub);
        dropSub(this.motor1SpeedEchoSub);
        dropSub(this.motor2SpeedEchoSub);
        dropSub(this.serialLogSub);
        dropSub(this.sensorStatusSub);
        this.sensorStatusSub = null;
        dropSub(this.executionStateSub);
        dropSub(this.executionStopRequestSub);
        dropSub(this.activeBlocksSub);

        // Listen for motor status updates
        // Throttle to 10Hz (100ms) to match the ROS publisher rate and reduce jitter
        const motorStatusThrottle = 100;

        this.motor1StatusSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor1/status',
            messageType: 'std_msgs/String'
        });
        this.motor1StatusSub.subscribe((msg) => {
            try {
                this.recordTelemetry('/motor1/status', msg.data);
                const now = performance.now();
                const lastUpdate = this.messageThrottle.get('/motor1/status')?.lastUpdate || 0;

                if (now - lastUpdate >= motorStatusThrottle) {
                    const status = this.safeJsonParse(msg.data, '/motor1/status');
                    if (status) {
                        this.motorStatus[1] = status;
                        this.messageThrottle.set('/motor1/status', { lastUpdate: now, throttleMs: motorStatusThrottle });
                        this.updateConnectionQuality();
                    }
                }
            } catch (e) {
                console.error('Failed to parse motor1 status:', e);
            }
        });
        
        this.motor2StatusSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor2/status',
            messageType: 'std_msgs/String'
        });
        this.motor2StatusSub.subscribe((msg) => {
            try {
                this.recordTelemetry('/motor2/status', msg.data);
                const now = performance.now();
                const lastUpdate = this.messageThrottle.get('/motor2/status')?.lastUpdate || 0;

                if (now - lastUpdate >= motorStatusThrottle) {
                    const status = this.safeJsonParse(msg.data, '/motor2/status');
                    if (status) {
                        this.motorStatus[2] = status;
                        this.messageThrottle.set('/motor2/status', { lastUpdate: now, throttleMs: motorStatusThrottle });
                        this.updateConnectionQuality();
                    }
                }
            } catch (e) {
                console.error('Failed to parse motor2 status:', e);
            }
        });

        this.arduinoStatusSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/arduino/status',
            messageType: 'std_msgs/String'
        });
        this.arduinoStatusSub.subscribe((msg) => {
            try {
                this.recordTelemetry('/arduino/status', msg.data);
                const data = this.safeJsonParse(msg.data, '/arduino/status');
                if (data && typeof data.connected === 'boolean') {
                    this.arduinoStatus = data;
                }
            } catch (e) {
                console.error('[ROS] arduino/status parse error:', e);
            }
        });

        for (let rid = 1; rid <= 4; rid++) {
            const topicName = `/relay${rid}/status`;
            const sub = new ROSLIB.Topic({
                ros: this.ros,
                name: topicName,
                messageType: 'std_msgs/String'
            });
            sub.subscribe((msg) => {
                try {
                    this.recordTelemetry(topicName, msg.data);
                    const data = this.safeJsonParse(msg.data, topicName);
                    if (data && data.relay_id != null) {
                        const prev = this.relayStates[rid];
                        const state = (data.state || 'off').toLowerCase();
                        const lastChangeMs = (!prev || prev.state !== state) ? Date.now() : prev.lastChangeMs;
                        this.relayStates[rid] = {
                            relay_id: data.relay_id,
                            state,
                            lastChangeMs,
                            lastPayload: msg.data
                        };
                    }
                } catch (e) {
                    console.error(`[ROS] ${topicName} parse error:`, e);
                }
            });
            this.relayStatusSubs.push(sub);
        }

        this.potentiometerSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/potentiometer/raw',
            messageType: 'std_msgs/Float32'
        });
        this.potentiometerSub.subscribe((msg) => {
            const v = msg && msg.data !== undefined && msg.data !== null ? Number(msg.data) : NaN;
            this.recordTelemetry('/potentiometer/raw', Number.isFinite(v) ? String(v) : String(msg && msg.data));
            if (Number.isFinite(v)) {
                this.lastPotRaw = v;
                this.lastPotAtMs = Date.now();
            }
        });

        this.potDebugSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/potentiometer/debug',
            messageType: 'std_msgs/String'
        });
        this.potDebugSub.subscribe((msg) => {
            this.recordTelemetry('/potentiometer/debug', msg.data);
            const parsed = this.safeJsonParse(msg.data, '/potentiometer/debug');
            if (parsed) {
                this.lastPotDebug = parsed;
                this.lastPotDebugAtMs = Date.now();
            }
        });

        this.motorSpeedSetpointSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor_speed/setpoint',
            messageType: 'std_msgs/Float32'
        });
        this.motorSpeedSetpointSub.subscribe((msg) => {
            const v = msg && msg.data !== undefined && msg.data !== null ? Number(msg.data) : NaN;
            this.recordTelemetry('/motor_speed/setpoint', Number.isFinite(v) ? String(v) : String(msg && msg.data));
            if (Number.isFinite(v)) {
                this.motorSpeedSetpoint = v;
            }
        });

        this.motor1SpeedEchoSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor1/speed',
            messageType: 'std_msgs/Float32'
        });
        this.motor1SpeedEchoSub.subscribe((msg) => {
            this.recordTelemetry('/motor1/speed_bus', String(msg.data));
            this.topicMotorSpeed[1] = msg.data;
        });

        this.motor2SpeedEchoSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor2/speed',
            messageType: 'std_msgs/Float32'
        });
        this.motor2SpeedEchoSub.subscribe((msg) => {
            this.recordTelemetry('/motor2/speed_bus', String(msg.data));
            this.topicMotorSpeed[2] = msg.data;
        });

        this.serialLogSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/arduino/serial_log',
            messageType: 'std_msgs/String'
        });
        this.serialLogSub.subscribe((msg) => {
            this.recordTelemetry('/arduino/serial_log', msg.data);
            if (typeof window.onArduinoSerialLog === 'function') {
                window.onArduinoSerialLog(msg.data);
            }
        });

        this.ensureSensorStatusSubscription();
        
        // Execution sync: so all instances (main + remote) see same playback state
        this.executionStateSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/assembly_line/execution_state',
            messageType: 'std_msgs/String'
        });
        this.executionStateSub.subscribe((msg) => {
            try {
                this.recordTelemetry('/assembly_line/execution_state', msg.data);
                const data = this.safeJsonParse(msg.data, '/assembly_line/execution_state');
                if (data && typeof data.running === 'boolean') {
                    this.executionSyncState = {
                        running: data.running,
                        projectId: data.projectId || null,
                        clientId: data.clientId || null
                    };
                    if (typeof window.onExecutionStateUpdate === 'function') {
                        window.onExecutionStateUpdate(this.executionSyncState);
                    }
                }
            } catch (e) {
                console.error('[ROS] execution_state parse error:', e);
            }
        });
        
        this.executionStopRequestSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/assembly_line/execution_stop_request',
            messageType: 'std_msgs/String'
        });
        this.executionStopRequestSub.subscribe(() => {
            if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.isExecuting) {
                ExecutionEngine.stop();
                this.stopExecutionStateHeartbeat();
                const projectId = (typeof StorageManager !== 'undefined' && StorageManager.getCurrentProject())
                    ? StorageManager.getCurrentProject().id
                    : null;
                this.publishExecutionState(false, projectId);
                if (typeof UIUtils !== 'undefined') {
                    UIUtils.log('[SYNC] Playback stopped by another device', 'warning');
                }
            }
        });
        
        this.activeBlocksSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/assembly_line/active_blocks',
            messageType: 'std_msgs/String'
        });
        this.activeBlocksSub.subscribe((msg) => {
            try {
                const data = this.safeJsonParse(msg.data, '/assembly_line/active_blocks');
                if (!data) return;
                if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.isExecuting && this.executionSyncState.clientId === this.getClientId()) {
                    return;
                }
                if (typeof window.onActiveBlocksUpdate === 'function') {
                    window.onActiveBlocksUpdate(data);
                }
            } catch (e) {
                console.error('[ROS] active_blocks parse error:', e);
            }
        });
        
        UIUtils.log('[ROS] Subscribers initialized', 'success');
    },

    ensureSensorStatusSubscription() {
        if (this.sensorStatusSub && this.sensorStatusSub.ros === this.ros) return;
        if (this.sensorStatusSub) try { this.sensorStatusSub.unsubscribe(); } catch (e) { /* ignore */ }
        this.sensorStatusSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/sensor/status',
            messageType: 'std_msgs/String'
        });
        this.sensorStatusSub.subscribe((msg) => {
            try {
                this.recordTelemetry('/sensor/status', msg.data);
                const status = this.safeJsonParse(msg.data, '/sensor/status');
                if (status && status.sensor_id) {
                    this.sensorStates.set(status.sensor_id, status);
                }
            } catch (e) {
                console.error('Failed to parse sensor status:', e);
            }
        });
    },

    /**
     * Get motor status
     * @param {number} motorId - Motor ID
     * @returns {Object|null} - Motor status or null
     */
    getMotorStatus(motorId) {
        // Check if simulation mode is active
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            return SimulationEngine.getMotorStatus(motorId);
        }
        
        return this.motorStatus[motorId] || null;
    },
    
    /**
     * Publish a motor command
     * @param {number} motorId - Motor ID (1 or 2)
     * @param {number} steps - Number of steps
     * @param {number} speed - Optional speed in steps per second (if not provided, uses global speed)
     * @returns {boolean} - Success status
     */
    publishMotorCommand(motorId, steps, speed = null) {
        // Check if simulation mode is active
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            return SimulationEngine.simulateMotorCommand(motorId, steps, speed);
        }
        
        if (!this.isConnected) {
            UIUtils.log('[ROS] Not connected, cannot publish motor command', 'error');
            return false;
        }
        
        let motorPub = null;
        let speedPub = null;
        if (motorId === 1) {
            motorPub = this.motor1Pub;
            speedPub = this.motor1SpeedPub;
        } else if (motorId === 2) {
            motorPub = this.motor2Pub;
            speedPub = this.motor2SpeedPub;
        }
        
        if (!motorPub) {
            UIUtils.log(`[ROS] Motor ${motorId} publisher not initialized`, 'error');
            return false;
        }
        
        try {
            // Always publish Float32 to motor{N}/speed when possible so ROS nodes (e.g. potentiometer_speed_node)
            // can learn baseline from the same message as the move block, even if caller passed speed=null.
            let effSpeed = speed;
            if ((effSpeed === null || effSpeed === undefined) && typeof MotorSpeedManager !== 'undefined') {
                effSpeed = MotorSpeedManager.getSpeed(motorId);
            }
            if (effSpeed !== null && effSpeed !== undefined && speedPub) {
                const speedMsg = new ROSLIB.Message({ data: parseFloat(effSpeed) });
                speedPub.publish(speedMsg);
            }
            
            // Then send the steps command
            const msg = new ROSLIB.Message({ data: parseInt(steps) });
            motorPub.publish(msg);
            return true;
        } catch (error) {
            UIUtils.log(`[ROS] Error publishing motor command: ${error}`, 'error');
            return false;
        }
    },

    /**
     * Publish motor speed only (no steps). Use this to change speed while a motor is moving
     * so that steps_remaining is preserved. Does not publish to /motorN/command.
     * @param {number} motorId - Motor ID (1 or 2)
     * @param {number} speed - Speed in steps per second
     * @returns {boolean} - Success status
     */
    publishMotorSpeedOnly(motorId, speed) {
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            SimulationEngine.setMotorSpeed(motorId, speed);
            return true;
        }
        if (!this.isConnected) return false;
        let speedPub = null;
        if (motorId === 1) speedPub = this.motor1SpeedPub;
        else if (motorId === 2) speedPub = this.motor2SpeedPub;
        if (!speedPub) return false;
        try {
            const speedVal = parseFloat(speed);
            const speedMsg = new ROSLIB.Message({ data: speedVal });
            speedPub.publish(speedMsg);
            // Update local status so Live display updates immediately
            this.motorStatus[motorId] = { ...(this.motorStatus[motorId] || {}), speed: speedVal };
            return true;
        } catch (error) {
            return false;
        }
    },

    /**
     * Publish a manual motor command: stop the motor first, then send steps.
     * This ignores any previous steps_remaining so the motor runs exactly the requested steps.
     * Use this for all manual (UI) motor controls.
     * @param {number} motorId - Motor ID (1 or 2)
     * @param {number} steps - Number of steps (use 0 only to stop; for stop use publishMotorCommand(motorId, 0))
     * @param {number} speed - Optional speed in steps per second
     * @returns {boolean} - Success status
     */
    publishManualMotorCommand(motorId, steps, speed = null) {
        if (steps === 0) {
            return this.publishMotorCommand(motorId, 0, speed);
        }
        this.publishMotorCommand(motorId, 0, null);
        return this.publishMotorCommand(motorId, steps, speed);
    },
    
    /**
     * Stop all motors by sending them 0 steps
     */
    stopAllMotors() {
        // Check if simulation mode is active
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            for (let motorId = 1; motorId <= 2; motorId++) {
                SimulationEngine.simulateMotorCommand(motorId, 0);
            }
            UIUtils.log('[STOP] All motors stopped (simulation)', 'warning');
            return;
        }
        
        if (!this.isConnected) {
            UIUtils.log('[ROS] Not connected, cannot stop motors', 'error');
            return;
        }
        
        // Tell both motors to stop (0 steps = stop command)
        for (let motorId = 1; motorId <= 2; motorId++) {
            this.publishMotorCommand(motorId, 0);
        }
        UIUtils.log('[STOP] All motors stopped', 'warning');
    },
    
    /**
     * Emergency stop - stops all motors and turns off all relays via dedicated E-Stop command
     */
    emergencyStop() {
        // Check if simulation mode is active
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            SimulationEngine.reset();
            UIUtils.log('[E-STOP] Emergency stop (simulation)', 'error');
            return true;
        }
        
        if (!this.isConnected) {
            UIUtils.log('[ROS] Not connected, cannot send E-Stop', 'error');
            return false;
        }
        
        // Send E-Stop command via dedicated topic
        try {
            if (this.estopPub) {
                const msg = new ROSLIB.Message({ data: 'ESTOP' });
                this.estopPub.publish(msg);
                UIUtils.log('[E-STOP] Emergency stop command sent', 'error');
            }
            
            // Also stop motors individually as backup
            this.stopAllMotors();
            
            // Turn off all relays
            for (let relayId = 1; relayId <= 4; relayId++) {
                this.publishRelayCommand(relayId, 'off');
            }
            
            return true;
        } catch (error) {
            UIUtils.log(`[E-STOP] Error sending E-Stop: ${error}`, 'error');
            return false;
        }
    },
    
    /**
     * Publish a relay command
     * @param {number} relayId - Relay ID
     * @param {string} state - State: 'on' or 'off'
     * @returns {boolean} - Success status
     */
    publishRelayCommand(relayId, state) {
        // Check if simulation mode is active
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            return SimulationEngine.simulateRelayCommand(relayId, state);
        }
        
        if (!this.isConnected) {
            UIUtils.log('[ROS] Not connected, cannot publish relay command', 'error');
            return false;
        }
        
        if (!this.relayPub) {
            UIUtils.log('[ROS] Relay publisher not initialized', 'error');
            return false;
        }
        
        try {
            const relayData = JSON.stringify({
                relay_id: parseInt(relayId),
                state: state
            });
            const msg = new ROSLIB.Message({ data: relayData });
            this.relayPub.publish(msg);
            return true;
        } catch (error) {
            UIUtils.log(`[ROS] Error publishing relay command: ${error}`, 'error');
            return false;
        }
    },
    
    /**
     * Get the correct message type for a topic (auto-detect system topics)
     * @param {string} topicName - ROS topic name
     * @param {string} defaultType - Default message type
     * @returns {string} - Correct message type for the topic
     */
    getTopicMessageType(topicName, defaultType = 'std_msgs/String') {
        // Auto-detect message types for known system topics
        // Note: ROSBridge for ROS 2 uses the /msg/ format for ROS 2 message types
        const systemTopicTypes = {
            '/rosout': 'rcl_interfaces/msg/Log',
            '/clock': 'builtin_interfaces/msg/Time',
            '/tf': 'tf2_msgs/msg/TFMessage',
            '/tf_static': 'tf2_msgs/msg/TFMessage'
        };
        
        // Store systemTopicTypes for use in waitForTopicString
        this.systemTopicTypes = systemTopicTypes;
        
        const detectedType = systemTopicTypes[topicName] || defaultType;
        UIUtils.log(`[ROS] Detected message type for ${topicName}: ${detectedType}`, 'success');
        return detectedType;
    },
    
    /**
     * Extract string from a ROS message (handles different message types)
     * @param {Object} msg - ROS message
     * @param {string} messageType - Message type
     * @returns {string} - Extracted string
     */
    extractStringFromMessage(msg, messageType) {
        // Handle rcl_interfaces/msg/Log (rosout)
        if (messageType === 'rcl_interfaces/msg/Log' || messageType.includes('Log')) {
            // Log messages have a 'msg' field containing the actual log message
            if (msg.msg !== undefined) {
                return String(msg.msg);
            }
            // Fallback: try to construct a readable string from log fields
            if (msg.name !== undefined || msg.msg !== undefined) {
                const parts = [];
                if (msg.name) parts.push(`[${msg.name}]`);
                if (msg.msg) parts.push(msg.msg);
                return parts.join(' ');
            }
        }
        
        // Handle std_msgs/String
        if (messageType === 'std_msgs/String' || messageType.includes('String')) {
            if (msg.data !== undefined) {
                return String(msg.data);
            }
        }
        
        // Generic fallback
        if (msg.data !== undefined) {
            return String(msg.data);
        } else if (typeof msg === 'string') {
            return msg;
        } else if (msg.msg !== undefined) {
            return String(msg.msg);
        } else {
            return JSON.stringify(msg);
        }
    },
    
    /**
     * Subscribe to a Float32 topic and continuously update motor speed (for "subscribe to motor speed topic" block).
     * Only one topic per motor; subscribing again for the same motor replaces the previous subscription.
     * @param {number} motorId - Motor ID (1 or 2)
     * @param {string} topicName - ROS topic name (std_msgs/Float32)
     * @returns {boolean} - Success
     */
    subscribeToMotorSpeedTopic(motorId, topicName) {
        if (motorId !== 1 && motorId !== 2) return false;
        const minSpeed = (typeof Config !== 'undefined' && Config.MIN_MOTOR_SPEED) ? Config.MIN_MOTOR_SPEED : 1;
        const maxSpeed = (typeof Config !== 'undefined' && Config.MAX_MOTOR_SPEED) ? Config.MAX_MOTOR_SPEED : 6500;
        this.unsubscribeFromMotorSpeedTopic(motorId);
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            this.motorSpeedTopicSubscriptions.set(motorId, { topicName, topic: null, handler: null, simulation: true });
            if (typeof UIUtils !== 'undefined') UIUtils.log(`[ROS] Motor ${motorId} speed following topic ${topicName} (simulation)`, 'success');
            return true;
        }
        if (!this.ros || !this.isConnected) return false;
        topicName = (topicName || '').trim() || '/motor_speed/setpoint';
        const topic = new ROSLIB.Topic({
            ros: this.ros,
            name: topicName,
            messageType: 'std_msgs/Float32'
        });
        const handler = (msg) => {
            const raw = (msg && msg.data !== undefined) ? parseFloat(msg.data) : NaN;
            const speed = Math.max(minSpeed, Math.min(maxSpeed, isNaN(raw) ? minSpeed : Math.round(raw)));
            if (typeof MotorSpeedManager !== 'undefined') MotorSpeedManager.setSpeed(motorId, speed);
            this.publishMotorSpeedOnly(motorId, speed);
            // Update local status so Live display updates immediately without waiting for /motorN/status
            this.motorStatus[motorId] = { ...(this.motorStatus[motorId] || {}), speed };
        };
        topic.subscribe(handler);
        this.motorSpeedTopicSubscriptions.set(motorId, { topicName, topic, handler });
        if (typeof UIUtils !== 'undefined') UIUtils.log(`[ROS] Motor ${motorId} speed following topic ${topicName}`, 'success');
        return true;
    },

    /**
     * Unsubscribe from continuous motor speed updates for a motor.
     * @param {number} motorId - Motor ID (1 or 2)
     * @returns {boolean} - Success
     */
    unsubscribeFromMotorSpeedTopic(motorId) {
        if (motorId !== 1 && motorId !== 2) return false;
        const sub = this.motorSpeedTopicSubscriptions.get(motorId);
        if (!sub) return true;
        this.motorSpeedTopicSubscriptions.delete(motorId);
        if (sub.topic) {
            try { sub.topic.unsubscribe(); } catch (e) {}
        }
        if (typeof UIUtils !== 'undefined') UIUtils.log(`[ROS] Motor ${motorId} no longer following speed topic`, 'success');
        return true;
    },

    /**
     * Wait for one Float32 message from a topic (e.g. motor speed setpoint).
     * @param {string} topicName - ROS topic name (std_msgs/Float32)
     * @param {number} timeoutMs - Timeout in ms (default 5000)
     * @returns {Promise<number>} - Resolves with msg.data, rejects on timeout or disconnect
     */
    waitForTopicFloat32(topicName, timeoutMs = 5000) {
        return new Promise((resolve, reject) => {
            if (!this.ros || !this.isConnected) {
                reject(new Error('Not connected to ROS Bridge'));
                return;
            }
            const topic = new ROSLIB.Topic({
                ros: this.ros,
                name: topicName,
                messageType: 'std_msgs/Float32'
            });
            const timeout = setTimeout(() => {
                try { topic.unsubscribe(); } catch (e) {}
                reject(new Error(`Timeout waiting for Float32 on ${topicName}`));
            }, timeoutMs);
            topic.subscribe((msg) => {
                clearTimeout(timeout);
                try { topic.unsubscribe(); } catch (e) {}
                const value = (msg && msg.data !== undefined) ? parseFloat(msg.data) : NaN;
                resolve(value);
            });
        });
    },
    
    /**
     * Subscribe to a ROS topic and wait for a specific string message
     * @param {string} topicName - ROS topic name
     * @param {string} expectedString - Expected string value to wait for
     * @param {string} messageType - ROS message type (default: auto-detect)
     * @param {number} startTime - Timestamp when the wait started (only process messages after this time)
     * @returns {Promise<string>} - Promise that resolves when the expected string is received
     */
    waitForTopicString(topicName, expectedString, messageType = null, startTime = null) {
        return new Promise((resolve, reject) => {
            console.log(`[ROS] waitForTopicString called: topic=${topicName}, expected="${expectedString}", messageType=${messageType}, startTime=${startTime}`);
            
            if (!this.isConnected) {
                const error = new Error('Not connected to ROS Bridge');
                console.error('[ROS] Not connected, rejecting promise');
                reject(error);
                return;
            }
            
            // Use current time as start time if not provided (for backward compatibility)
            const waitStartTime = startTime || Date.now();
            
            // Auto-detect message type if not provided
            if (!messageType) {
                messageType = this.getTopicMessageType(topicName);
            } else {
                // Log when message type is explicitly provided
                UIUtils.log(`[ROS] Using provided message type for ${topicName}: ${messageType}`, 'success');
            }
            
            console.log(`[ROS] Using messageType: ${messageType} for topic: ${topicName}`);
            
            // Ensure we're using the correct message type (re-detect for system topics to be sure)
            const systemTopicTypes = this.systemTopicTypes || {
                '/rosout': 'rcl_interfaces/msg/Log',
                '/clock': 'builtin_interfaces/msg/Time',
                '/tf': 'tf2_msgs/msg/TFMessage',
                '/tf_static': 'tf2_msgs/msg/TFMessage'
            };
            const correctMessageType = this.getTopicMessageType(topicName, messageType);
            if (correctMessageType !== messageType && systemTopicTypes.hasOwnProperty(topicName)) {
                UIUtils.log(`[ROS] Overriding message type for ${topicName} from ${messageType} to ${correctMessageType}`, 'warning');
                messageType = correctMessageType;
            }
            
            // Check if we already have a subscription for this topic in our dynamic subscriptions
            let subscription = this.dynamicSubscriptions.get(topicName);
            
            if (!subscription) {
                // Create new subscription - ROSLIB allows multiple subscriptions to the same topic
                // Even if the topic is already subscribed elsewhere, this will work fine
                UIUtils.log(`[ROS] Creating new subscription to ${topicName} with messageType: ${messageType}`, 'success');
                const topic = new ROSLIB.Topic({
                    ros: this.ros,
                    name: topicName,
                    messageType: messageType
                });
                
                subscription = {
                    topic: topic,
                    messageType: messageType,
                    listeners: new Map(), // Map of expectedString -> listener function
                    masterHandler: null // Will be set below
                };
                
                // Create a master message handler that distributes to all listeners
                const masterHandler = (msg) => {
                    // Extract string from message (handle different message formats)
                    const receivedString = this.extractStringFromMessage(msg, messageType);
                    
                    // Get message timestamp (use current time if not available in message)
                    // ROS messages may have a header with timestamp, but for simplicity we use receive time
                    const messageTimestamp = Date.now();
                    
                    // Log received messages for debugging (only if there are active listeners)
                    if (subscription.listeners.size > 0) {
                        UIUtils.log(`[ROS] Received on ${topicName}: "${receivedString}"`, 'info');
                    }
                    
                    // Notify all listeners (create a copy of the map to avoid modification during iteration)
                    const listenersCopy = new Map(subscription.listeners);
                    listenersCopy.forEach((listener) => {
                        try {
                            listener(receivedString, messageTimestamp);
                        } catch (error) {
                            console.error(`[ROS] Error in listener for topic ${topicName}:`, error);
                        }
                    });
                };
                
                // Store the master handler in the subscription object
                subscription.masterHandler = masterHandler;
                
                // Subscribe the master handler with error handling
                // Note: ROSLIB.Topic.subscribe() can be called multiple times on the same topic
                // Each subscription will receive all messages, so this works even if the topic
                // is already subscribed to elsewhere (e.g., by motor status subscriptions)
                try {
                    topic.subscribe(masterHandler);
                    this.dynamicSubscriptions.set(topicName, subscription);
                    UIUtils.log(`[ROS] Subscribed to topic: ${topicName} (messageType: ${messageType})`, 'success');
                } catch (error) {
                    // Handle subscription errors (e.g., message type mismatch)
                    const errorMsg = error.message || String(error);
                    if (errorMsg.includes('already established') || errorMsg.includes('message type')) {
                        UIUtils.log(`[ROS] Error: Topic ${topicName} is already subscribed with a different message type. Please check the topic's actual message type.`, 'error');
                        reject(new Error(`Topic ${topicName} message type mismatch. Expected ${messageType} but topic uses a different type.`));
                        return;
                    } else {
                        UIUtils.log(`[ROS] Error subscribing to topic ${topicName}: ${errorMsg}`, 'error');
                        reject(error);
                        return;
                    }
                }
            } else {
                // Topic already subscribed - verify message type matches
                if (subscription.messageType !== messageType) {
                    UIUtils.log(`[ROS] Warning: Topic ${topicName} already subscribed with messageType ${subscription.messageType}, but requested ${messageType}. Using existing subscription.`, 'warning');
                }
                
                // Ensure the subscription has a master handler that passes timestamps
                // If it doesn't, we need to resubscribe with a new handler
                if (!subscription.masterHandler) {
                    UIUtils.log(`[ROS] Existing subscription missing master handler, resubscribing...`, 'warning');
                    // Unsubscribe the old subscription
                    subscription.topic.unsubscribe();
                    
                    // Create a new master handler that passes timestamps
                    const masterHandler = (msg) => {
                        const receivedString = this.extractStringFromMessage(msg, messageType);
                        const messageTimestamp = Date.now();
                        
                        if (subscription.listeners.size > 0) {
                            UIUtils.log(`[ROS] Received on ${topicName}: "${receivedString}"`, 'info');
                        }
                        
                        const listenersCopy = new Map(subscription.listeners);
                        listenersCopy.forEach((listener) => {
                            try {
                                listener(receivedString, messageTimestamp);
                            } catch (error) {
                                console.error(`[ROS] Error in listener for topic ${topicName}:`, error);
                            }
                        });
                    };
                    
                    subscription.masterHandler = masterHandler;
                    subscription.topic.subscribe(masterHandler);
                    UIUtils.log(`[ROS] Resubscribed to topic: ${topicName} with timestamp support`, 'success');
                }
            }
            
            // Create a listener for this specific wait
            // Use a unique key that includes both expectedString and a timestamp to handle
            // multiple blocks waiting for the same string
            const listenerKey = `${expectedString}_${Date.now()}_${Math.random()}`;
            let isResolved = false; // Prevent multiple resolutions
            
            const listener = (receivedString, messageTimestamp = null) => {
                // Only process messages received after the wait started
                // This ensures we don't process messages from before the workflow started
                // If messageTimestamp is not provided (e.g., from an old subscription handler),
                // use current time. Since this listener was just added, current time will be
                // >= waitStartTime, so the message will be processed correctly.
                const messageTime = messageTimestamp !== null && messageTimestamp !== undefined ? messageTimestamp : Date.now();
                if (messageTime < waitStartTime) {
                    console.log(`[ROS] Ignoring message received before wait started (message: ${new Date(messageTime).toISOString()}, wait start: ${new Date(waitStartTime).toISOString()})`);
                    return;
                }
                
                console.log(`[ROS] Listener called for "${expectedString}": received="${receivedString}"`);
                
                // Skip if already resolved
                if (isResolved) {
                    console.log(`[ROS] Listener already resolved, skipping`);
                    return;
                }
                
                // Normalize strings for comparison (trim and case-insensitive)
                const normalizedReceived = receivedString.trim().toLowerCase();
                const normalizedExpected = expectedString.trim().toLowerCase();
                
                console.log(`[ROS] Comparing: "${normalizedReceived}" contains "${normalizedExpected}"?`);
                
                // Check if this matches the expected string
                // Support both exact match and substring match (if expected string is found in received string)
                const exactMatch = normalizedReceived === normalizedExpected;
                const substringMatch = normalizedReceived.includes(normalizedExpected);
                
                console.log(`[ROS] Match results: exact=${exactMatch}, substring=${substringMatch}`);
                
                if (exactMatch || substringMatch) {
                    console.log(`[ROS] MATCH FOUND! Resolving promise with: "${receivedString}"`);
                    isResolved = true;
                    
                    // Remove this listener
                    subscription.listeners.delete(listenerKey);
                    
                    // Keep the subscription active even when there are no listeners
                    // This allows it to be reused in subsequent workflow runs without
                    // needing to unsubscribe and resubscribe, which can cause issues
                    // The master handler will simply not call any listeners when the map is empty
                    if (subscription.listeners.size === 0) {
                        UIUtils.log(`[ROS] All listeners removed from ${topicName}, keeping subscription active for reuse`, 'info');
                    }
                    
                    UIUtils.log(`[ROS] Matched "${expectedString}" in message: "${receivedString}"`, 'success');
                    resolve(receivedString);
                } else {
                    console.log(`[ROS] No match, continuing to wait...`);
                }
            };
            
            // Add listener to the map BEFORE subscribing (if new subscription)
            // This ensures we don't miss any messages
            subscription.listeners.set(listenerKey, listener);
            
            // Ensure the subscription is active (for both new and existing subscriptions)
            // ROSLIB subscriptions should remain active, but we verify
            if (subscription.topic) {
                UIUtils.log(`[ROS] Listener added to subscription for ${topicName}, waiting for messages...`, 'success');
            }
            
            // Set a timeout to log a warning (but don't reject)
            setTimeout(() => {
                if (subscription.listeners.has(listenerKey) && !isResolved) {
                    UIUtils.log(`[ROS] Still waiting for "${expectedString}" on topic ${topicName}...`, 'warning');
                }
            }, 5000);
        });
    },
    
    /**
     * Unsubscribe from a dynamic topic
     * @param {string} topicName - ROS topic name
     */
    unsubscribeFromTopic(topicName) {
        const subscription = this.dynamicSubscriptions.get(topicName);
        if (subscription) {
            subscription.topic.unsubscribe();
            this.dynamicSubscriptions.delete(topicName);
            UIUtils.log(`[ROS] Unsubscribed from topic: ${topicName}`, 'success');
        }
    },
    
    /**
     * Clean up all dynamic subscriptions (called on disconnect)
     */
    cleanupDynamicSubscriptions() {
        this.dynamicSubscriptions.forEach((subscription, topicName) => {
            subscription.topic.unsubscribe();
        });
        this.dynamicSubscriptions.clear();
        this.motorSpeedTopicSubscriptions.forEach((sub) => {
            if (sub.topic) try { sub.topic.unsubscribe(); } catch (e) {}
        });
        this.motorSpeedTopicSubscriptions.clear();
        this.messageThrottle.clear();
        this.parseCache.clear();
    },
    
    /**
     * Update connection quality metrics
     */
    updateConnectionQuality() {
        const now = performance.now();
        const lastMessageTime = this.connectionQuality.lastMessageTime;
        
        if (lastMessageTime) {
            const latency = now - lastMessageTime;
            // Exponential moving average for latency
            this.connectionQuality.latency = this.connectionQuality.latency * 0.9 + latency * 0.1;
            
            // Calculate message rate (messages per second)
            const timeDelta = (now - lastMessageTime) / 1000;
            if (timeDelta > 0) {
                const currentRate = 1 / timeDelta;
                this.connectionQuality.messageRate = this.connectionQuality.messageRate * 0.9 + currentRate * 0.1;
            }
        }
        
        this.connectionQuality.lastMessageTime = now;
    },
    
    /**
     * Get connection quality metrics
     * @returns {Object} Connection quality metrics
     */
    getConnectionQuality() {
        return {
            latency: this.connectionQuality.latency,
            messageRate: this.connectionQuality.messageRate,
            isHealthy: this.connectionQuality.latency < 100 && this.connectionQuality.messageRate > 5
        };
    },
    
    /**
     * Sensor states cache
     */
    sensorStates: new Map(),
    
    /**
     * Wait for sensor condition
     * @param {string} sensorId - Sensor ID
     * @param {string} condition - Condition: 'HIGH', 'LOW', '>', '<', '==', '!='
     * @param {number} threshold - Threshold value (for analog sensors)
     * @param {number} timeout - Timeout in milliseconds (0 = no timeout)
     * @returns {Promise<boolean>} - Resolves when condition is met or timeout
     */
    waitForSensor(sensorId, condition, threshold = 0, timeout = 0) {
        return new Promise((resolve, reject) => {
            if (!this.isConnected) {
                reject(new Error('Not connected to ROS Bridge'));
                return;
            }
            
            const startTime = performance.now();
            const checkInterval = 100; // Check every 100ms
            
            const checkCondition = () => {
                const sensor = this.sensorStates.get(sensorId);
                
                if (!sensor) {
                    // Sensor not found - check timeout
                    if (timeout > 0 && (performance.now() - startTime) >= timeout) {
                        clearInterval(intervalId);
                        reject(new Error(`Sensor ${sensorId} timeout after ${timeout}ms`));
                        return;
                    }
                    return; // Continue waiting
                }
                
                const value = sensor.value;
                let conditionMet = false;
                
                if (sensor.type === 'digital') {
                    // Digital sensor conditions
                    if (condition === 'HIGH' && value === true) {
                        conditionMet = true;
                    } else if (condition === 'LOW' && value === false) {
                        conditionMet = true;
                    }
                } else {
                    // Analog sensor conditions
                    switch (condition) {
                        case '>':
                            conditionMet = value > threshold;
                            break;
                        case '<':
                            conditionMet = value < threshold;
                            break;
                        case '==':
                            conditionMet = Math.abs(value - threshold) < 0.01;
                            break;
                        case '!=':
                            conditionMet = Math.abs(value - threshold) >= 0.01;
                            break;
                    }
                }
                
                if (conditionMet) {
                    clearInterval(intervalId);
                    if (timeoutId) clearTimeout(timeoutId);
                    resolve(true);
                    return;
                }
                
                // Check timeout
                if (timeout > 0 && (performance.now() - startTime) >= timeout) {
                    clearInterval(intervalId);
                    if (timeoutId) clearTimeout(timeoutId);
                    reject(new Error(`Sensor ${sensorId} timeout after ${timeout}ms`));
                    return;
                }
            };
            
            this.ensureSensorStatusSubscription();
            
            // Check immediately
            checkCondition();
            
            // Set up interval
            const intervalId = setInterval(checkCondition, checkInterval);
            
            // Set up timeout if specified
            let timeoutId = null;
            if (timeout > 0) {
                timeoutId = setTimeout(() => {
                    clearInterval(intervalId);
                    reject(new Error(`Sensor ${sensorId} timeout after ${timeout}ms`));
                }, timeout);
            }
        });
    },
    
    /**
     * Read sensor value
     * @param {string} sensorId - Sensor ID
     * @returns {boolean|number|null} - Sensor value or null if not found
     */
    readSensor(sensorId) {
        const sensor = this.sensorStates.get(sensorId);
        return sensor ? sensor.value : null;
    }
};

