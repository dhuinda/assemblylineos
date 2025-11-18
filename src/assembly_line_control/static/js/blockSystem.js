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
        } else if (templateData.type === 'relay') {
            block.relay_id = templateData.relay_id;
            block.state = templateData.state || 'off';
        } else if (templateData.type === 'delay') {
            block.duration = this.validateTime(templateData.duration || 1.0);
        } else if (templateData.type === 'ros-trigger') {
            block.topic = templateData.topic || '/rosout';
            block.expectedString = templateData.expectedString || '';
        }
        
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
     * @returns {number} - Block width in pixels
     */
    calculateBlockWidth(blockData) {
        // Keep blocks at a fixed width to prevent expansion with larger values
        // The width should accommodate the content but not grow excessively
        if (blockData.type === 'motor' || blockData.type === 'delay') {
            // Use a more reasonable calculation - minimal growth for very large values
            // Most blocks should stay close to default width
            return Config.BLOCK_DEFAULT_WIDTH;
        }
        return Config.BLOCK_DEFAULT_WIDTH;
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
                if (stepsInput) {
                    const oldSteps = blockData.steps;
                    blockData.steps = this.validateSteps(parseInt(stepsInput.value) || 0);
                    
                    // Update visual display with current motor speed
                    const motorSpeed = MotorSpeedManager.getSpeed(blockData.motor_id);
                    const duration = motorSpeed > 0 ? ((blockData.steps || 0) / motorSpeed).toFixed(2) : '0.00';
                    const timeDisplay = blockEl.querySelector('.text-gray-400');
                    if (timeDisplay) {
                        timeDisplay.textContent = `${duration}s @ ${motorSpeed} sps`;
                    }
                    
                    // Width is now fixed, so no need to update it
                    // Removed width recalculation to prevent block expansion
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
            'delay': 'block-delay'
        };
        return classMap[type] || '';
    }
};

