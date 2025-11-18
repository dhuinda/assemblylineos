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
    relayPub: null,
    sequencePub: null,
    motor1StatusSub: null,
    motor2StatusSub: null,
    motorStatus: {}, // Keep track of each motor's status
    isConnected: false,
    dynamicSubscriptions: new Map(), // For subscribing to topics on the fly
    messageThrottle: new Map(), // Slow down updates that come too fast
    connectionQuality: { latency: 0, messageRate: 0, lastMessageTime: null }, // How's the connection doing?
    parseCache: new Map(), // Cache parsed JSON so we don't parse the same thing twice
    
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
            UIUtils.updateRosStatus(true);
            this.initializePublishers();
            this.initializeSubscribers();
        });

        this.ros.on('error', (error) => {
            UIUtils.log('[ROS] Connection error: ' + error, 'error');
            this.isConnected = false;
            UIUtils.updateRosStatus(false);
        });

        this.ros.on('close', () => {
            UIUtils.log('[ROS] Connection closed', 'error');
            this.isConnected = false;
            UIUtils.updateRosStatus(false);
            
            // Clean up any subscriptions we made
            this.cleanupDynamicSubscriptions();
            
            // Try to reconnect after a short delay
            setTimeout(() => {
                this.init();
            }, Config.ROS_RECONNECT_DELAY);
        });
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
        
        UIUtils.log('[ROS] Publishers initialized', 'success');
    },
    
    /**
     * Set up subscribers so we can listen for status updates from motors
     */
    initializeSubscribers() {
        // Listen for motor status updates
        // Slow them down a bit so we don't get overwhelmed (max 20 times per second)
        const motorStatusThrottle = 50;
        
        this.motor1StatusSub = new ROSLIB.Topic({
            ros: this.ros,
            name: '/motor1/status',
            messageType: 'std_msgs/String'
        });
        this.motor1StatusSub.subscribe((msg) => {
            try {
                const now = performance.now();
                const lastUpdate = this.messageThrottle.get('/motor1/status')?.lastUpdate || 0;
                
                // Only process if enough time has passed (throttling)
                if (now - lastUpdate >= motorStatusThrottle) {
                    const status = this.safeJsonParse(msg.data, '/motor1/status');
                    if (status) {
                        this.motorStatus[1] = status;
                        this.messageThrottle.set('/motor1/status', { lastUpdate: now, throttleMs: motorStatusThrottle });
                        
                        // Update our connection quality stats
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
                const now = performance.now();
                const lastUpdate = this.messageThrottle.get('/motor2/status')?.lastUpdate || 0;
                
                // Throttle updates to reduce parsing overhead
                if (now - lastUpdate >= motorStatusThrottle) {
                    const status = this.safeJsonParse(msg.data, '/motor2/status');
                    if (status) {
                        this.motorStatus[2] = status;
                        this.messageThrottle.set('/motor2/status', { lastUpdate: now, throttleMs: motorStatusThrottle });
                        
                        // Update connection quality metrics
                        this.updateConnectionQuality();
                    }
                }
            } catch (e) {
                console.error('Failed to parse motor2 status:', e);
            }
        });
        
        UIUtils.log('[ROS] Subscribers initialized', 'success');
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
     * @returns {boolean} - Success status
     */
    publishMotorCommand(motorId, steps) {
        // Check if simulation mode is active
        if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
            return SimulationEngine.simulateMotorCommand(motorId, steps);
        }
        
        if (!this.isConnected) {
            UIUtils.log('[ROS] Not connected, cannot publish motor command', 'error');
            return false;
        }
        
        let motorPub = null;
        if (motorId === 1) motorPub = this.motor1Pub;
        else if (motorId === 2) motorPub = this.motor2Pub;
        
        if (!motorPub) {
            UIUtils.log(`[ROS] Motor ${motorId} publisher not initialized`, 'error');
            return false;
        }
        
        try {
            const msg = new ROSLIB.Message({ data: parseInt(steps) });
            motorPub.publish(msg);
            return true;
        } catch (error) {
            UIUtils.log(`[ROS] Error publishing motor command: ${error}`, 'error');
            return false;
        }
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
            UIUtils.log('[PAUSE] All motors stopped (simulation)', 'warning');
            return;
        }
        
        if (!this.isConnected) {
            UIUtils.log('[ROS] Not connected, cannot stop motors', 'error');
            return;
        }
        
        // Tell both motors to stop
        for (let motorId = 1; motorId <= 2; motorId++) {
            this.publishMotorCommand(motorId, 0);
        }
        UIUtils.log('[PAUSE] All motors stopped', 'warning');
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
            
            // Subscribe to sensor status if not already subscribed
            if (!this.sensorStatusSub) {
                this.sensorStatusSub = new ROSLIB.Topic({
                    ros: this.ros,
                    name: '/sensor/status',
                    messageType: 'std_msgs/String'
                });
                
                this.sensorStatusSub.subscribe((msg) => {
                    try {
                        const status = this.safeJsonParse(msg.data, '/sensor/status');
                        if (status && status.sensor_id) {
                            this.sensorStates.set(status.sensor_id, status);
                        }
                    } catch (e) {
                        console.error('Failed to parse sensor status:', e);
                    }
                });
            }
            
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

