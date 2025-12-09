/**
 * Motor Speed Manager - Manages motor speed settings
 */
const MotorSpeedManager = {
    speeds: {
        1: 100, // steps per second (default)
        2: 100
    },
    
    /**
     * Get speed for a motor (checks for block-specific speed first)
     * @param {number} motorId - Motor ID
     * @param {Object} block - Optional block data to check for custom speed
     * @returns {number} - Speed in steps per second
     */
    getSpeed(motorId, block = null) {
        // If block has a custom speed, use that
        if (block && block.speed !== undefined && block.speed !== null) {
            return block.speed;
        }
        // Otherwise use global motor speed
        return this.speeds[motorId] || Config.MOTOR_STEPS_PER_SECOND;
    },
    
    /**
     * Set global speed for a motor
     * @param {number} motorId - Motor ID
     * @param {number} speed - Speed in steps per second
     */
    setSpeed(motorId, speed) {
        if (motorId >= 1 && motorId <= 2) {
            const minSpeed = Config.MIN_MOTOR_SPEED || 1;
            const maxSpeed = Config.MAX_MOTOR_SPEED || 6500;
            this.speeds[motorId] = Math.max(minSpeed, Math.min(maxSpeed, parseInt(speed) || 100));
            this.updateUI(motorId);
            this.updateMotorBlocks(motorId);
            StorageManager.autoSave();
        }
    },
    
    /**
     * Update UI display for motor speed
     * @param {number} motorId - Motor ID
     */
    updateUI(motorId) {
        const speedSlider = document.getElementById(`motor${motorId}Speed`);
        const speedValue = document.getElementById(`motor${motorId}SpeedValue`);
        
        if (speedSlider) {
            speedSlider.value = this.speeds[motorId];
        }
        if (speedValue) {
            // Handle both span (textContent) and input (value) elements
            if (speedValue.tagName === 'INPUT') {
                speedValue.value = this.speeds[motorId];
            } else {
                speedValue.textContent = this.speeds[motorId];
            }
        }
    },
    
    /**
     * Update motor block displays when global speed changes
     * @param {number} motorId - Motor ID
     */
    updateMotorBlocks(motorId) {
        // Find all motor blocks for this motor and update their duration display
        document.querySelectorAll(`[data-type="motor"][data-motor-id="${motorId}"]`).forEach(blockEl => {
            const blockId = parseInt(blockEl.dataset.blockId);
            
            // Get block from WorkflowManager
            const block = WorkflowManager.blocks.get(blockId);
            if (block && block.type === 'motor' && block.motor_id === motorId) {
                // Only update blocks that use global speed (no custom speed set)
                const hasCustomSpeed = block.speed !== undefined && block.speed !== null;
                if (!hasCustomSpeed) {
                    const motorSpeed = this.getSpeed(block.motor_id);
                    const duration = motorSpeed > 0 ? (Math.abs(block.steps || 0) / motorSpeed).toFixed(2) : '0.00';
                    const durationDisplay = blockEl.querySelector('.motor-duration-display');
                    if (durationDisplay) {
                        durationDisplay.textContent = `${duration}s @ ${motorSpeed} sps (global)`;
                    }
                }
            }
        });
    },
    
    /**
     * Initialize motor speeds from storage or defaults
     */
    init() {
        try {
            const saved = localStorage.getItem('motorSpeeds');
            if (saved) {
                const speeds = JSON.parse(saved);
                // Validate and apply saved speeds
                const minSpeed = Config.MIN_MOTOR_SPEED || 1;
                const maxSpeed = Config.MAX_MOTOR_SPEED || 6500;
                
                if (speeds[1]) {
                    this.speeds[1] = Math.max(minSpeed, Math.min(maxSpeed, parseInt(speeds[1]) || 100));
                }
                if (speeds[2]) {
                    this.speeds[2] = Math.max(minSpeed, Math.min(maxSpeed, parseInt(speeds[2]) || 100));
                }
            }
        } catch (error) {
            console.warn('Failed to load motor speeds from storage:', error);
            // Use defaults
        }
        
        // Update UI and sliders
        this.updateUI(1);
        this.updateUI(2);
        this.updateSliderRanges();
    },
    
    /**
     * Update slider ranges based on config
     */
    updateSliderRanges() {
        const minSpeed = Config.MIN_MOTOR_SPEED || 1;
        const maxSpeed = Config.MAX_MOTOR_SPEED || 6500;
        
        for (let motorId = 1; motorId <= 2; motorId++) {
            const speedSlider = document.getElementById(`motor${motorId}Speed`);
            if (speedSlider) {
                speedSlider.min = minSpeed;
                speedSlider.max = maxSpeed;
            }
        }
    },
    
    /**
     * Save motor speeds to storage
     */
    save() {
        try {
            localStorage.setItem('motorSpeeds', JSON.stringify(this.speeds));
        } catch (error) {
            // Silently fail
        }
    }
};

