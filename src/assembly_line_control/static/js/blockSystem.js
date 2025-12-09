/**
 * Block System - Block creation, validation, and management
 * Enhanced with validation, error handling, and robustness features
 */
const BlockSystem = {
    /**
     * Extract block data from a DOM element
     * @param {HTMLElement} blockElement - The block DOM element
     * @returns {Object|null} - Block data object or null if invalid
     */
    extractBlockData(blockElement) {
        if (!blockElement) {
            UIUtils.log('[BLOCK] Cannot extract data from null element', 'error');
            return null;
        }
        
        const type = blockElement.dataset.type || blockElement.getAttribute('data-type');
        if (!type) {
            UIUtils.log('[BLOCK] Block element missing type attribute', 'error');
            return null;
        }
        
        const data = { type };
        
        try {
            if (type === 'event') {
                const eventType = blockElement.dataset.eventType || blockElement.getAttribute('data-event-type');
                if (eventType) {
                    data.eventType = eventType;
                }
            } else if (type === 'motor') {
                const motorId = blockElement.dataset.motorId || blockElement.getAttribute('data-motor-id');
                if (!motorId) {
                    UIUtils.log('[BLOCK] Motor block missing motor ID', 'error');
                    return null;
                }
                data.motor_id = parseInt(motorId);
                if (isNaN(data.motor_id) || data.motor_id < 1) {
                    UIUtils.log('[BLOCK] Invalid motor ID', 'error');
                    return null;
                }
                
                const stepsInput = blockElement.querySelector('[data-param="steps"]');
                const steps = stepsInput ? parseInt(stepsInput.value) || 0 : 0;
                data.steps = this.validateSteps(steps);
                
                // Check for custom speed
                const speedInput = blockElement.querySelector('[data-param="speed"]');
                if (speedInput && speedInput.value !== '') {
                    const speed = parseInt(speedInput.value);
                    if (!isNaN(speed) && speed >= 1) {
                        data.speed = this.validateSpeed(speed);
                    }
                }
                
            } else if (type === 'relay') {
                const relayId = blockElement.dataset.relayId || blockElement.getAttribute('data-relay-id');
                if (!relayId) {
                    UIUtils.log('[BLOCK] Relay block missing relay ID', 'error');
                    return null;
                }
                data.relay_id = parseInt(relayId);
                if (isNaN(data.relay_id) || data.relay_id < 1) {
                    UIUtils.log('[BLOCK] Invalid relay ID', 'error');
                    return null;
                }
                
                const stateSelect = blockElement.querySelector('[data-param="state"]');
                const state = stateSelect ? stateSelect.value : 'off';
                data.state = (state === 'on' || state === 'off') ? state : 'off';
                
            } else if (type === 'delay') {
                const durationInput = blockElement.querySelector('[data-param="duration"]');
                const duration = durationInput ? parseFloat(durationInput.value) || 1.0 : 1.0;
                data.duration = this.validateTime(duration);
            } else if (type === 'repeat') {
                const countInput = blockElement.querySelector('[data-param="count"]');
                const count = countInput ? parseInt(countInput.value) || 10 : 10;
                data.count = Math.max(1, count);
            } else if (type === 'forever' || type === 'break') {
                // These types don't need additional parameters
            } else if (type === 'wait-sensor') {
                const sensorIdInput = blockElement.querySelector('[data-param="sensorId"]');
                const conditionSelect = blockElement.querySelector('[data-param="condition"]');
                const thresholdInput = blockElement.querySelector('[data-param="threshold"]');
                const timeoutInput = blockElement.querySelector('[data-param="timeout"]');
                
                data.sensorId = sensorIdInput ? (sensorIdInput.value || '') : '';
                data.condition = conditionSelect ? (conditionSelect.value || 'HIGH') : 'HIGH';
                data.threshold = thresholdInput ? (parseFloat(thresholdInput.value) || 0.0) : 0.0;
                data.timeout = timeoutInput ? (parseFloat(timeoutInput.value) || 0.0) : 0.0;
            } else if (type === 'read-sensor') {
                const sensorIdInput = blockElement.querySelector('[data-param="sensorId"]');
                data.sensorId = sensorIdInput ? (sensorIdInput.value || '') : '';
            } else if (type === 'try' || type === 'catch') {
                // These types don't need additional parameters
            } else if (type === 'throw-error') {
                const errorMessageInput = blockElement.querySelector('[data-param="errorMessage"]');
                data.errorMessage = errorMessageInput ? (errorMessageInput.value || '') : '';
            } else if (type === 'ros-trigger') {
                // Handle both select dropdown and text input for topic
                const topicSelect = blockElement.querySelector('select[data-param="topic"]');
                const topicInput = blockElement.querySelector('input[data-param="topic"]');
                const expectedStringInput = blockElement.querySelector('[data-param="expectedString"]');
                
                // Get topic from select if available, otherwise from input
                if (topicSelect) {
                    const selectedValue = topicSelect.value;
                    if (selectedValue === '/topic' && topicInput) {
                        // Custom topic selected, use input value
                        data.topic = topicInput.value || '/rosout';
                    } else {
                        // Use selected value from dropdown
                        data.topic = selectedValue || '/rosout';
                    }
                } else if (topicInput) {
                    data.topic = topicInput.value || '/rosout';
                } else {
                    data.topic = '/rosout';
                }
                
                data.expectedString = expectedStringInput ? (expectedStringInput.value || '') : '';
            } else if (type === 'trigger' || type === 'pause') {
                // These types don't need additional data
            } else if (type === 'event') {
                // Event blocks are handled above
            } else {
                UIUtils.log(`[BLOCK] Unknown block type: ${type}`, 'error');
                return null;
            }
        } catch (error) {
            UIUtils.log(`[BLOCK] Error extracting block data: ${error}`, 'error');
            return null;
        }
        
        return data;
    },
    
    /**
     * Validate and clamp steps value
     * @param {number} steps - Steps value to validate
     * @returns {number} - Validated steps value
     */
    validateSteps(steps) {
        if (isNaN(steps)) return 0;
        return Math.max(Config.MIN_STEPS, Math.min(Config.MAX_STEPS, Math.round(steps)));
    },
    
    /**
     * Validate and clamp time delay value
     * @param {number} time - Time value to validate
     * @returns {number} - Validated time value
     */
    validateTime(time) {
        if (isNaN(time)) return 0;
        return Math.max(Config.MIN_TIME_DELAY, Math.min(Config.MAX_TIME_DELAY, parseFloat(time)));
    },
    
    /**
     * Validate and clamp motor speed value
     * @param {number} speed - Speed value to validate
     * @returns {number} - Validated speed value
     */
    validateSpeed(speed) {
        if (isNaN(speed)) return Config.MOTOR_STEPS_PER_SECOND;
        const minSpeed = Config.MIN_MOTOR_SPEED || 1;
        const maxSpeed = Config.MAX_MOTOR_SPEED || 6500;
        return Math.max(minSpeed, Math.min(maxSpeed, Math.round(speed)));
    },
    
    /**
     * Validate a block's data
     * @param {Object} blockData - Block data to validate
     * @returns {Object} - Validation result with isValid and errors array
     */
    validateBlock(blockData) {
        const errors = [];
        const warnings = [];
        
        if (!blockData) {
            return { isValid: false, errors: ['Block data is null or undefined'], warnings: [] };
        }
        
        if (!blockData.type) {
            errors.push('Block missing type');
        }
        
        if (blockData.type === 'motor') {
            if (!blockData.motor_id || blockData.motor_id < 1) {
                errors.push('Invalid motor ID');
            }
            if (blockData.steps === undefined || blockData.steps === null) {
                errors.push('Motor block missing steps value');
            } else {
                const validatedSteps = this.validateSteps(blockData.steps);
                if (validatedSteps !== blockData.steps) {
                    warnings.push(`Steps value clamped to ${validatedSteps}`);
                }
                if (Math.abs(blockData.steps) > 10000) {
                    warnings.push('Large step value may cause long execution time');
                }
            }
        } else if (blockData.type === 'relay') {
            if (!blockData.relay_id || blockData.relay_id < 1) {
                errors.push('Invalid relay ID');
            }
            if (blockData.state !== 'on' && blockData.state !== 'off') {
                errors.push('Invalid relay state (must be "on" or "off")');
            }
        }
        
        if (blockData.type === 'delay') {
            if (blockData.duration === undefined || blockData.duration === null) {
                errors.push('Delay block missing duration value');
            } else {
                const validatedDuration = this.validateTime(blockData.duration);
                if (validatedDuration !== blockData.duration) {
                    warnings.push(`Duration clamped to ${validatedDuration}s`);
                }
                if (blockData.duration > 60) {
                    warnings.push('Large delay duration may cause long wait times');
                }
            }
        } else if (blockData.type === 'ros-trigger') {
            if (!blockData.topic || blockData.topic.trim() === '') {
                errors.push('ROS trigger block missing topic name');
            }
            if (blockData.expectedString === undefined || blockData.expectedString === null) {
                errors.push('ROS trigger block missing expected string');
            }
        }
        
        return {
            isValid: errors.length === 0,
            errors,
            warnings
        };
    },
    
    /**
     * Create a new block instance from template data
     * @param {Object} templateData - Block template data
     * @param {number} blockId - Unique block ID
     * @returns {Object} - New block instance
     */
    createBlockInstance(templateData, blockId) {
        if (!templateData || !templateData.type) {
            UIUtils.log('[BLOCK] Cannot create block: invalid template data', 'error');
            return null;
        }
        
        const block = {
            id: blockId,
            type: templateData.type,
            connections: { prev: null, next: null }
        };
        
        if (templateData.type === 'event') {
            block.eventType = templateData.eventType || 'green-flag';
        } else if (templateData.type === 'motor') {
            block.motor_id = templateData.motor_id;
            block.steps = this.validateSteps(templateData.steps || 0);
            // Preserve custom speed if set
            if (templateData.speed !== undefined && templateData.speed !== null) {
                block.speed = this.validateSpeed(templateData.speed);
            }
        } else if (templateData.type === 'relay') {
            block.relay_id = templateData.relay_id;
            block.state = templateData.state || 'off';
        } else if (templateData.type === 'delay') {
            block.duration = this.validateTime(templateData.duration || 1.0);
        } else if (templateData.type === 'ros-trigger') {
            block.topic = templateData.topic || '/rosout';
            block.expectedString = templateData.expectedString || '';
        } else if (templateData.type === 'repeat') {
            block.count = templateData.count || 10;
        } else if (templateData.type === 'wait-sensor') {
            block.sensorId = templateData.sensorId || '';
            block.condition = templateData.condition || 'HIGH';
            block.threshold = templateData.threshold || 0.0;
            block.timeout = templateData.timeout || 0.0;
        } else if (templateData.type === 'read-sensor') {
            block.sensorId = templateData.sensorId || '';
        } else if (templateData.type === 'throw-error') {
            block.errorMessage = templateData.errorMessage || '';
        }
        // Note: pause, forever, break, try, catch don't need additional properties beyond type
        
        // Validate the created block
        const validation = this.validateBlock(block);
        if (!validation.isValid) {
            UIUtils.log(`[BLOCK] Created block has validation errors: ${validation.errors.join(', ')}`, 'error');
        } else if (validation.warnings.length > 0) {
            UIUtils.log(`[BLOCK] Block warnings: ${validation.warnings.join(', ')}`, 'warning');
        }
        
        return block;
    },
    
    /**
     * Calculate block width based on its properties
     * @param {Object} blockData - Block data
     * @returns {number} - Block width in pixels (always odd multiple of grid size)
     */
    calculateBlockWidth(blockData) {
        const gridSize = 20;
        const baseWidth = Config.BLOCK_DEFAULT_WIDTH;
        
        // Round to nearest odd multiple of grid size
        // This ensures connectors are always in the center of grid cells
        const gridUnits = Math.round(baseWidth / gridSize);
        // Make it odd (if even, add 1)
        const oddGridUnits = gridUnits % 2 === 0 ? gridUnits + 1 : gridUnits;
        // Ensure minimum of 5 grid units (100px)
        const finalGridUnits = Math.max(5, oddGridUnits);
        
        return finalGridUnits * gridSize;
    },
    
    /**
     * Calculate block height based on its content
     * @param {Object} blockData - Block data
     * @returns {number} - Block height in pixels (always odd multiple of grid size)
     */
    calculateBlockHeight(blockData) {
        const gridSize = 20;
        // Base height for most blocks - increased to prevent cutting
        let baseHeight = 100; // 5 grid units (already odd)
        
        // Adjust for blocks with more content - ensure enough space for all fields
        if (blockData.type === 'wait-sensor') {
            // Has: header + sensor input + condition dropdown + threshold input + timeout input
            // Each field ~20-25px, plus gaps and padding
            baseHeight = 200; // 10 grid units -> will become 11 (odd) = 220px
        } else if (blockData.type === 'ros-trigger') {
            // Has: header + topic section (label + select + conditional input) + wait for text (label + input + help)
            // Needs more space for all the inputs and help text
            baseHeight = 200; // 10 grid units -> will become 11 (odd) = 220px
        } else if (blockData.type === 'read-sensor' || blockData.type === 'throw-error') {
            // Has: header + one input field
            baseHeight = 120; // 6 grid units -> will become 7 (odd) = 140px
        } else if (blockData.type === 'motor') {
            baseHeight = 140; // 7 grid units = 140px (has steps + speed inputs + duration display)
        } else if (blockData.type === 'relay' || blockData.type === 'delay' || blockData.type === 'repeat') {
            baseHeight = 100; // 5 grid units (already odd) = 100px
        } else if (blockData.type === 'event') {
            baseHeight = 100; // 5 grid units (already odd) = 100px
        }
        
        // Round to nearest odd multiple of grid size
        const gridUnits = Math.round(baseHeight / gridSize);
        const oddGridUnits = gridUnits % 2 === 0 ? gridUnits + 1 : gridUnits;
        // Ensure minimum of 5 grid units (100px) for better content visibility
        const finalGridUnits = Math.max(5, oddGridUnits);
        
        return finalGridUnits * gridSize;
    },
    
    /**
     * Update block parameters from DOM input
     * @param {Object} blockData - Block data to update
     * @param {HTMLElement} blockEl - Block DOM element
     * @returns {boolean} - Whether update was successful
     */
    updateBlockFromDOM(blockData, blockEl) {
        if (!blockData || !blockEl) return false;
        
        try {
            if (blockData.type === 'delay') {
                const durationInput = blockEl.querySelector('[data-param="duration"]');
                if (durationInput) {
                    blockData.duration = this.validateTime(parseFloat(durationInput.value) || 1.0);
                }
            } else if (blockData.type === 'motor') {
                const stepsInput = blockEl.querySelector('[data-param="steps"]');
                const speedInput = blockEl.querySelector('[data-param="speed"]');
                
                if (stepsInput) {
                    blockData.steps = this.validateSteps(parseInt(stepsInput.value) || 0);
                }
                
                // Handle custom speed
                if (speedInput) {
                    if (speedInput.value !== '' && speedInput.value !== null) {
                        const speed = parseInt(speedInput.value);
                        if (!isNaN(speed) && speed >= 1) {
                            blockData.speed = this.validateSpeed(speed);
                        } else {
                            delete blockData.speed; // Remove custom speed if invalid
                        }
                    } else {
                        delete blockData.speed; // Remove custom speed if empty (use global)
                    }
                }
                
                // Determine which speed to display
                const hasCustomSpeed = blockData.speed !== undefined && blockData.speed !== null;
                const motorSpeed = hasCustomSpeed ? blockData.speed : MotorSpeedManager.getSpeed(blockData.motor_id);
                const duration = motorSpeed > 0 ? (Math.abs(blockData.steps || 0) / motorSpeed).toFixed(2) : '0.00';
                const speedLabel = hasCustomSpeed ? 'custom' : 'global';
                
                // Update visual display
                const timeDisplay = blockEl.querySelector('.motor-duration-display');
                if (timeDisplay) {
                    timeDisplay.textContent = `${duration}s @ ${motorSpeed} sps (${speedLabel})`;
                }
            } else if (blockData.type === 'relay') {
                const select = blockEl.querySelector('[data-param="state"]');
                if (select) {
                    blockData.state = (select.value === 'on' || select.value === 'off') ? select.value : 'off';
                }
            } else if (blockData.type === 'ros-trigger') {
                // Handle both select dropdown and text input for topic
                const topicSelect = blockEl.querySelector('select[data-param="topic"]');
                const topicInput = blockEl.querySelector('input[data-param="topic"]');
                const expectedStringInput = blockEl.querySelector('[data-param="expectedString"]');
                
                // Get topic from select if available, otherwise from input
                if (topicSelect) {
                    const selectedValue = topicSelect.value;
                    if (selectedValue === '/topic' && topicInput) {
                        // Custom topic selected, use input value
                        blockData.topic = topicInput.value || '/rosout';
                    } else {
                        // Use selected value from dropdown
                        blockData.topic = selectedValue || '/rosout';
                    }
                } else if (topicInput) {
                    blockData.topic = topicInput.value || '/rosout';
                }
                
                if (expectedStringInput) {
                    blockData.expectedString = expectedStringInput.value || '';
                }
            }
            // Note: trigger and pause blocks have no parameters
            
            // Validate and update visual state
            const validation = this.validateBlock(blockData);
            UIUtils.clearBlockStates(blockEl);
            if (!validation.isValid) {
                UIUtils.markBlockError(blockEl);
            } else if (validation.warnings.length > 0) {
                UIUtils.markBlockWarning(blockEl);
            }
            
            return true;
        } catch (error) {
            UIUtils.log(`[BLOCK] Error updating block from DOM: ${error}`, 'error');
            return false;
        }
    },
    
    /**
     * Get block type class name
     * @param {string} type - Block type
     * @returns {string} - CSS class name
     */
    getBlockTypeClass(type) {
        const classMap = {
            'motor': 'block-motor',
            'relay': 'block-relay',
            'trigger': 'block-trigger',
            'ros-trigger': 'block-trigger',
            'event': 'block-event',
            'pause': 'block-pause',
            'delay': 'block-delay',
            'repeat': 'block-loop',
            'forever': 'block-loop',
            'break': 'block-loop',
            'wait-sensor': 'block-sensor',
            'read-sensor': 'block-sensor',
            'try': 'block-error',
            'catch': 'block-error',
            'throw-error': 'block-error'
        };
        return classMap[type] || '';
    }
};

