/**
 * Simulation Engine - Virtual hardware for testing workflows without physical hardware
 * 
 * This simulates motors, relays, and sensors with realistic timing and state tracking.
 */
const SimulationEngine = {
    // Simulation state
    isActive: false,
    speedMultiplier: 1.0, // 1.0 = real-time, 2.0 = 2x faster, 0.5 = 2x slower
    simulationTime: 0, // Virtual time in milliseconds
    
    // Virtual motor states
    motors: {
        1: { position: 0, speed: 100, isMoving: false, stepsRemaining: 0, lastUpdateTime: 0 },
        2: { position: 0, speed: 100, isMoving: false, stepsRemaining: 0, lastUpdateTime: 0 }
    },
    
    // Virtual relay states
    relays: {
        1: { state: 'off', lastChangeTime: 0 },
        2: { state: 'off', lastChangeTime: 0 },
        3: { state: 'off', lastChangeTime: 0 },
        4: { state: 'off', lastChangeTime: 0 }
    },
    
    // Virtual sensor states (configurable)
    sensors: {
        // Digital sensors (limit switches, etc.)
        'limit_switch_1': { type: 'digital', value: false, lastChangeTime: 0 },
        'limit_switch_2': { type: 'digital', value: false, lastChangeTime: 0 },
        // Analog sensors
        'encoder_1': { type: 'analog', value: 0, lastChangeTime: 0 },
        'encoder_2': { type: 'analog', value: 0, lastChangeTime: 0 }
    },
    
    // Update interval for motor simulation
    updateInterval: null,
    
    /**
     * Initialize simulation engine
     */
    init() {
        this.reset();
        UIUtils.log('[SIMULATION] Simulation engine initialized', 'success');
    },
    
    /**
     * Start simulation mode
     */
    start() {
        if (this.isActive) {
            UIUtils.log('[SIMULATION] Simulation already active', 'warning');
            return;
        }
        
        this.isActive = true;
        this.simulationTime = performance.now();
        
        // Start update loop (simulate motor movement)
        this.startUpdateLoop();
        
        UIUtils.log('[SIMULATION] Simulation mode activated', 'success');
    },
    
    /**
     * Stop simulation mode
     */
    stop() {
        if (!this.isActive) {
            return;
        }
        
        this.isActive = false;
        this.stopUpdateLoop();
        
        // Stop all motors
        for (let motorId in this.motors) {
            this.motors[motorId].isMoving = false;
            this.motors[motorId].stepsRemaining = 0;
        }
        
        UIUtils.log('[SIMULATION] Simulation mode deactivated', 'warning');
    },
    
    /**
     * Reset simulation state
     */
    reset() {
        // Reset motors
        for (let motorId in this.motors) {
            this.motors[motorId] = {
                position: 0,
                speed: 100,
                isMoving: false,
                stepsRemaining: 0,
                lastUpdateTime: performance.now()
            };
        }
        
        // Reset relays
        for (let relayId in this.relays) {
            this.relays[relayId] = {
                state: 'off',
                lastChangeTime: performance.now()
            };
        }
        
        // Reset sensors (keep configuration but reset values)
        for (let sensorId in this.sensors) {
            const sensor = this.sensors[sensorId];
            if (sensor.type === 'digital') {
                sensor.value = false;
            } else {
                sensor.value = 0;
            }
            sensor.lastChangeTime = performance.now();
        }
        
        this.simulationTime = 0;
        UIUtils.log('[SIMULATION] Simulation state reset', 'info');
    },
    
    /**
     * Start update loop for motor simulation
     */
    startUpdateLoop() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
        }
        
        // Update motors every 50ms (20 times per second)
        this.updateInterval = setInterval(() => {
            this.updateMotors();
        }, 50);
    },
    
    /**
     * Stop update loop
     */
    stopUpdateLoop() {
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    },
    
    /**
     * Update motor positions based on speed and time
     */
    updateMotors() {
        const currentTime = performance.now();
        
        for (let motorId in this.motors) {
            const motor = this.motors[motorId];
            
            if (motor.isMoving && motor.stepsRemaining !== 0) {
                const elapsed = (currentTime - motor.lastUpdateTime) * this.speedMultiplier;
                const stepsPerSecond = motor.speed;
                const stepsToComplete = (stepsPerSecond * elapsed) / 1000;
                
                if (motor.stepsRemaining > 0) {
                    motor.stepsRemaining = Math.max(0, motor.stepsRemaining - stepsToComplete);
                    motor.position += stepsToComplete;
                } else {
                    motor.stepsRemaining = Math.min(0, motor.stepsRemaining + stepsToComplete);
                    motor.position -= stepsToComplete;
                }
                
                if (Math.abs(motor.stepsRemaining) < 0.5) {
                    motor.stepsRemaining = 0;
                    motor.isMoving = false;
                }
                
                motor.lastUpdateTime = currentTime;
            }
        }
    },
    
    /**
     * Simulate motor command
     * @param {number} motorId - Motor ID (1 or 2)
     * @param {number} steps - Number of steps to move
     * @returns {boolean} - Success status
     */
    simulateMotorCommand(motorId, steps) {
        if (!this.isActive) {
            return false;
        }
        
        const motor = this.motors[motorId];
        if (!motor) {
            UIUtils.log(`[SIMULATION] Invalid motor ID: ${motorId}`, 'error');
            return false;
        }
        
        if (steps === 0) {
            // Stop motor
            motor.isMoving = false;
            motor.stepsRemaining = 0;
            return true;
        }
        
        // Add steps to queue
        motor.stepsRemaining += steps;
        motor.isMoving = true;
        motor.lastUpdateTime = performance.now();
        
        UIUtils.log(`[SIMULATION] Motor ${motorId}: ${steps > 0 ? '+' : ''}${steps} steps (total remaining: ${motor.stepsRemaining.toFixed(1)})`, 'info');
        return true;
    },
    
    /**
     * Set motor speed
     * @param {number} motorId - Motor ID
     * @param {number} speed - Speed in steps per second
     */
    setMotorSpeed(motorId, speed) {
        const motor = this.motors[motorId];
        if (motor) {
            motor.speed = Math.max(1, Math.min(200, speed));
            UIUtils.log(`[SIMULATION] Motor ${motorId} speed set to ${motor.speed} steps/second`, 'info');
        }
    },
    
    /**
     * Get motor status
     * @param {number} motorId - Motor ID
     * @returns {Object} - Motor status
     */
    getMotorStatus(motorId) {
        const motor = this.motors[motorId];
        if (!motor) {
            return null;
        }
        
        return {
            motor_id: parseInt(motorId),
            steps_remaining: Math.round(motor.stepsRemaining),
            speed: motor.speed,
            is_moving: motor.isMoving,
            position: Math.round(motor.position)
        };
    },
    
    /**
     * Simulate relay command
     * @param {number} relayId - Relay ID (1-4)
     * @param {string} state - State: 'on' or 'off'
     * @returns {boolean} - Success status
     */
    simulateRelayCommand(relayId, state) {
        if (!this.isActive) {
            return false;
        }
        
        const relay = this.relays[relayId];
        if (!relay) {
            UIUtils.log(`[SIMULATION] Invalid relay ID: ${relayId}`, 'error');
            return false;
        }
        
        relay.state = state;
        relay.lastChangeTime = performance.now();
        
        UIUtils.log(`[SIMULATION] Relay ${relayId}: ${state.toUpperCase()}`, 'info');
        return true;
    },
    
    /**
     * Get relay status
     * @param {number} relayId - Relay ID
     * @returns {Object} - Relay status
     */
    getRelayStatus(relayId) {
        const relay = this.relays[relayId];
        if (!relay) {
            return null;
        }
        
        return {
            relay_id: parseInt(relayId),
            state: relay.state
        };
    },
    
    /**
     * Set sensor value
     * @param {string} sensorId - Sensor ID
     * @param {boolean|number} value - Sensor value (boolean for digital, number for analog)
     */
    setSensorValue(sensorId, value) {
        const sensor = this.sensors[sensorId];
        if (!sensor) {
            // Create new sensor
            const type = typeof value === 'boolean' ? 'digital' : 'analog';
            this.sensors[sensorId] = {
                type: type,
                value: value,
                lastChangeTime: performance.now()
            };
        } else {
            sensor.value = value;
            sensor.lastChangeTime = performance.now();
        }
    },
    
    /**
     * Get sensor value
     * @param {string} sensorId - Sensor ID
     * @returns {boolean|number|null} - Sensor value or null if not found
     */
    getSensorValue(sensorId) {
        const sensor = this.sensors[sensorId];
        return sensor ? sensor.value : null;
    },
    
    /**
     * Wait for sensor condition
     * @param {string} sensorId - Sensor ID
     * @param {string} condition - Condition: 'HIGH', 'LOW', '>', '<', '==', '!='
     * @param {number} threshold - Threshold value (for analog sensors)
     * @param {number} timeout - Timeout in milliseconds (0 = no timeout)
     * @returns {Promise<boolean>} - Resolves when condition is met or timeout
     */
    async waitForSensor(sensorId, condition, threshold = 0, timeout = 0) {
        return new Promise((resolve, reject) => {
            if (!this.isActive) {
                reject(new Error('Simulation not active'));
                return;
            }
            
            const sensor = this.sensors[sensorId];
            if (!sensor) {
                reject(new Error(`Sensor ${sensorId} not found`));
                return;
            }
            
            const startTime = performance.now();
            const checkInterval = 50; // Check every 50ms
            
            const checkCondition = () => {
                if (!this.isActive) {
                    clearInterval(intervalId);
                    reject(new Error('Simulation stopped'));
                    return;
                }
                
                const currentValue = sensor.value;
                let conditionMet = false;
                
                if (sensor.type === 'digital') {
                    // Digital sensor conditions
                    if (condition === 'HIGH' && currentValue === true) {
                        conditionMet = true;
                    } else if (condition === 'LOW' && currentValue === false) {
                        conditionMet = true;
                    }
                } else {
                    // Analog sensor conditions
                    switch (condition) {
                        case '>':
                            conditionMet = currentValue > threshold;
                            break;
                        case '<':
                            conditionMet = currentValue < threshold;
                            break;
                        case '==':
                            conditionMet = Math.abs(currentValue - threshold) < 0.01;
                            break;
                        case '!=':
                            conditionMet = Math.abs(currentValue - threshold) >= 0.01;
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
                if (timeout > 0) {
                    const elapsed = (performance.now() - startTime) * this.speedMultiplier;
                    if (elapsed >= timeout) {
                        clearInterval(intervalId);
                        if (timeoutId) clearTimeout(timeoutId);
                        reject(new Error(`Sensor ${sensorId} timeout after ${timeout}ms`));
                        return;
                    }
                }
            };
            
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
                }, timeout / this.speedMultiplier);
            }
        });
    },
    
    /**
     * Set speed multiplier
     * @param {number} multiplier - Speed multiplier (1.0 = real-time, 2.0 = 2x faster)
     */
    setSpeedMultiplier(multiplier) {
        this.speedMultiplier = Math.max(0.1, Math.min(10.0, multiplier));
        UIUtils.log(`[SIMULATION] Speed multiplier set to ${this.speedMultiplier}x`, 'info');
    },
    
    /**
     * Get simulation statistics
     * @returns {Object} - Simulation stats
     */
    getStats() {
        return {
            isActive: this.isActive,
            speedMultiplier: this.speedMultiplier,
            simulationTime: this.simulationTime,
            motors: Object.keys(this.motors).map(id => this.getMotorStatus(id)),
            relays: Object.keys(this.relays).map(id => this.getRelayStatus(id)),
            sensors: Object.keys(this.sensors).map(id => ({
                id: id,
                type: this.sensors[id].type,
                value: this.sensors[id].value
            }))
        };
    }
};

