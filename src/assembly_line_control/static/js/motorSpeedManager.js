/**
 * Motor Speed Manager - Manages motor speed settings
 */
const MotorSpeedManager = {
    speeds: {
        1: 100, // steps per second
        2: 100
    },
    
    /**
     * Get speed for a motor
     * @param {number} motorId - Motor ID
     * @returns {number} - Speed in steps per second
     */
    getSpeed(motorId) {
        return this.speeds[motorId] || Config.MOTOR_STEPS_PER_SECOND;
    },
    
    /**
     * Set speed for a motor
     * @param {number} motorId - Motor ID
     * @param {number} speed - Speed in steps per second
     */
    setSpeed(motorId, speed) {
        if (motorId >= 1 && motorId <= 2) {
            this.speeds[motorId] = Math.max(1, Math.min(200, parseInt(speed) || 100));
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
            speedValue.textContent = this.speeds[motorId];
        }
    },
    
    /**
     * Update motor block displays when speed changes
     * @param {number} motorId - Motor ID
     */
    updateMotorBlocks(motorId) {
        // Find all motor blocks for this motor and update their duration display
        document.querySelectorAll(`[data-type="motor"][data-motor-id="${motorId}"]`).forEach(blockEl => {
            const blockId = parseInt(blockEl.dataset.blockId);
            
            // Get block from WorkflowManager
            const block = WorkflowManager.blocks.get(blockId);
            if (block && block.type === 'motor' && block.motor_id === motorId) {
                const motorSpeed = this.getSpeed(block.motor_id);
                const duration = motorSpeed > 0 ? ((block.steps || 0) / motorSpeed).toFixed(2) : '0.00';
                const durationDisplay = blockEl.querySelector('.text-gray-400');
                if (durationDisplay) {
                    durationDisplay.textContent = `${duration}s @ ${motorSpeed} sps`;
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
                this.speeds = { ...this.speeds, ...speeds };
            }
        } catch (error) {
            // Use defaults
        }
        
        // Update UI
        this.updateUI(1);
        this.updateUI(2);
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

