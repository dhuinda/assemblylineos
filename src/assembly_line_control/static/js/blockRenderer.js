/**
 * Block Renderer - Handles rendering of block DOM elements (Scratch-like)
 */
const BlockRenderer = {
    /**
     * Create a Scratch-style block DOM element
     * @param {Object} blockData - Block data
     * @returns {HTMLElement} - Block DOM element
     */
    createScratchBlock(blockData) {
        const block = document.createElement('div');
        const typeClass = BlockSystem.getBlockTypeClass(blockData.type);
        
        // Event blocks have special class
        if (blockData.type === 'event') {
            block.className = `scratch-block block-event`;
        } else {
            block.className = `scratch-block ${typeClass}`;
        }
        
        block.dataset.type = blockData.type;
        block.dataset.blockId = blockData.id;
        
        // Calculate width and height (always odd multiples of grid size)
        const width = BlockSystem.calculateBlockWidth(blockData);
        const height = BlockSystem.calculateBlockHeight(blockData);
        block.style.width = width + 'px';
        block.style.height = height + 'px';
        block.style.minHeight = height + 'px';
        // Allow slight overflow if needed, but prefer calculated height
        block.style.overflow = 'visible';
        
        // Block content
        block.innerHTML = this.generateScratchBlockContent(blockData);
        
        // Add top connector (for non-event blocks)
        if (blockData.type !== 'event') {
            const topConnector = document.createElement('div');
            topConnector.className = 'block-connector top';
            topConnector.dataset.connectorType = 'top';
            topConnector.onclick = (e) => {
                e.stopPropagation();
                BlockConnector.handleConnectorClick(blockData.id, 'top');
            };
            topConnector.oncontextmenu = (e) => {
                e.preventDefault();
                e.stopPropagation();
                // Disconnect if connected
                const conn = BlockConnector.connections.get(blockData.id);
                if (conn && conn.prev) {
                    BlockConnector.disconnectBlocks(conn.prev, blockData.id);
                }
            };
            block.appendChild(topConnector);
        }
        
        // Add bottom connector (for all blocks except those that end workflows)
        const bottomConnector = document.createElement('div');
        bottomConnector.className = 'block-connector bottom';
        bottomConnector.dataset.connectorType = 'bottom';
        bottomConnector.onclick = (e) => {
            e.stopPropagation();
            BlockConnector.handleConnectorClick(blockData.id, 'bottom');
        };
        bottomConnector.oncontextmenu = (e) => {
            e.preventDefault();
            e.stopPropagation();
            // Disconnect all connections if connected
            const conn = BlockConnector.connections.get(blockData.id);
            if (conn && conn.next) {
                const nextArray = Array.isArray(conn.next) ? conn.next : [conn.next];
                // Disconnect all connections
                nextArray.forEach(nextId => {
                    BlockConnector.disconnectBlocks(blockData.id, nextId);
                });
            }
        };
        block.appendChild(bottomConnector);
        
        // Add remove button
        const removeBtn = document.createElement('button');
        removeBtn.className = 'remove-btn';
        removeBtn.textContent = '×';
        removeBtn.title = 'Delete block';
        removeBtn.onclick = (e) => {
            e.stopPropagation();
            if (UIUtils.confirm(`Delete block ${blockData.id}?`)) {
                WorkflowManager.removeBlock(blockData.id);
            }
        };
        block.appendChild(removeBtn);
        
        // Add workflow trigger button (for non-event blocks)
        // Show indicator if this block triggers workflows
        if (blockData.type !== 'event') {
            const workflowBtn = document.createElement('button');
            workflowBtn.className = 'workflow-trigger-btn';
            workflowBtn.textContent = '⚡';
            
            // Check if this block already triggers workflows
            const triggersWorkflows = blockData.triggersWorkflows && blockData.triggersWorkflows.length > 0;
            if (triggersWorkflows) {
                workflowBtn.classList.add('active');
                workflowBtn.title = `Triggers ${blockData.triggersWorkflows.length} workflow(s) - Click to create another`;
            } else {
                workflowBtn.title = 'Create workflow from this block';
            }
            
            workflowBtn.onclick = (e) => {
                e.stopPropagation();
                WorkflowManager.createWorkflowFromBlock(blockData.id);
            };
            block.appendChild(workflowBtn);
            
            // Add visual indicator if this block triggers workflows
            if (triggersWorkflows) {
                const indicator = document.createElement('div');
                indicator.className = 'workflow-trigger-indicator';
                indicator.title = `This block triggers ${blockData.triggersWorkflows.length} workflow(s)`;
                block.appendChild(indicator);
            }
        }
        
        // Add input event listeners for parameter updates
        this.attachParameterListeners(block, blockData);
        
        // Update connector states
        setTimeout(() => {
            BlockConnector.updateConnectorVisuals(blockData.id);
        }, 10);
        
        return block;
    },
    
    /**
     * Generate HTML content for a Scratch-style block
     * @param {Object} blockData - Block data
     * @returns {string} - HTML content
     */
    generateScratchBlockContent(blockData) {
        if (blockData.type === 'event') {
            const eventType = blockData.eventType || 'green-flag';
            if (eventType === 'green-flag') {
                return `
                    <div class="flex items-center gap-2" style="font-size: 11px;">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <span class="accent-trigger font-semibold">▶ WHEN START CLICKED</span>
                    </div>
                `;
            } else if (eventType === 'workflow-complete') {
                const triggeredByText = blockData.triggeredBy ? ` (Block #${blockData.triggeredBy})` : '';
                return `
                    <div class="flex flex-col gap-1" style="font-size: 11px;">
                        <div class="flex items-center gap-2">
                            <span class="block-id-badge">#${blockData.id}</span>
                            <span class="accent-trigger font-semibold">⚡ WHEN WORKFLOW COMPLETE</span>
                        </div>
                        ${triggeredByText ? `<div class="text-xs text-gray-400">Triggered by${triggeredByText}</div>` : ''}
                    </div>
                `;
            }
        }
        
        // Reuse existing generateBlockContent for other block types
        return this.generateBlockContent(blockData);
    },
    
    /**
     * Generate HTML content for a block (legacy method, still used for non-event blocks)
     * @param {Object} blockData - Block data
     * @returns {string} - HTML content
     */
    generateBlockContent(blockData) {
        if (blockData.type === 'trigger') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-trigger text-xs">TRIGGER</div>
                    </div>
                </div>
            `;
        } else if (blockData.type === 'motor') {
            const motorSpeed = MotorSpeedManager.getSpeed(blockData.motor_id);
            const duration = motorSpeed > 0 ? ((blockData.steps || 0) / motorSpeed).toFixed(2) : '0.00';
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-motor">M${blockData.motor_id}</div>
                    </div>
                    <input type="number" class="w-full px-1 py-0.5 text-xs text-white accent-motor" 
                           placeholder="steps" data-param="steps" value="${blockData.steps || 0}"
                           onclick="event.stopPropagation()"
                           style="max-width: 100%;">
                    <div class="text-xs text-gray-400">
                        ${duration}s @ ${motorSpeed} sps
                    </div>
                </div>
            `;
        } else if (blockData.type === 'relay') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-relay">R${blockData.relay_id}</div>
                    </div>
                    <select class="w-full px-1 py-0.5 text-xs text-white accent-relay" 
                            data-param="state"
                            onclick="event.stopPropagation()"
                            style="max-width: 100%;">
                        <option value="on" ${blockData.state === 'on' ? 'selected' : ''}>ON</option>
                        <option value="off" ${blockData.state === 'off' ? 'selected' : ''}>OFF</option>
                    </select>
                </div>
            `;
        } else if (blockData.type === 'pause') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-pause text-xs">PAUSE</div>
                    </div>
                </div>
            `;
        } else if (blockData.type === 'delay') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-delay text-xs">DELAY</div>
                    </div>
                    <input type="number" step="0.1" class="w-full px-1 py-0.5 text-xs text-white accent-delay" 
                           placeholder="duration (s)" data-param="duration" value="${blockData.duration || 1.0}"
                           onclick="event.stopPropagation()"
                           style="max-width: 100%;">
                </div>
            `;
        } else if (blockData.type === 'repeat') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-loop text-xs">REPEAT</div>
                    </div>
                    <input type="number" class="w-full px-1 py-0.5 text-xs text-white accent-loop" 
                           placeholder="times" data-param="count" value="${blockData.count || 10}" min="1"
                           onclick="event.stopPropagation()"
                           style="max-width: 100%;">
                </div>
            `;
        } else if (blockData.type === 'forever') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-loop text-xs">FOREVER</div>
                    </div>
                </div>
            `;
        } else if (blockData.type === 'break') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-loop text-xs">BREAK</div>
                    </div>
                </div>
            `;
        } else if (blockData.type === 'wait-sensor') {
            const sensorId = blockData.sensorId || '';
            const condition = blockData.condition || 'HIGH';
            const threshold = blockData.threshold || 0.0;
            const timeout = blockData.timeout || 0.0;
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-sensor text-xs">WAIT FOR SENSOR</div>
                    </div>
                    <div class="flex flex-col gap-0.5">
                        <label class="text-xs text-gray-400">Sensor:</label>
                        <input type="text" class="w-full px-1 py-0.5 text-xs text-white accent-sensor" 
                               placeholder="sensor_id" data-param="sensorId" value="${sensorId}"
                               onclick="event.stopPropagation()"
                               style="max-width: 100%;">
                    </div>
                    <div class="flex flex-col gap-0.5">
                        <label class="text-xs text-gray-400">Condition:</label>
                        <select class="w-full px-1 py-0.5 text-xs text-white accent-sensor" 
                                data-param="condition"
                                onclick="event.stopPropagation()"
                                style="max-width: 100%;">
                            <option value="HIGH" ${condition === 'HIGH' ? 'selected' : ''}>HIGH</option>
                            <option value="LOW" ${condition === 'LOW' ? 'selected' : ''}>LOW</option>
                            <option value=">" ${condition === '>' ? 'selected' : ''}>Greater than</option>
                            <option value="<" ${condition === '<' ? 'selected' : ''}>Less than</option>
                            <option value="==" ${condition === '==' ? 'selected' : ''}>Equal to</option>
                        </select>
                    </div>
                    <div class="flex flex-col gap-0.5">
                        <label class="text-xs text-gray-400">Threshold:</label>
                        <input type="number" step="0.1" class="w-full px-1 py-0.5 text-xs text-white accent-sensor" 
                               placeholder="0.0" data-param="threshold" value="${threshold}"
                               onclick="event.stopPropagation()"
                               style="max-width: 100%;">
                    </div>
                    <div class="flex flex-col gap-0.5">
                        <label class="text-xs text-gray-400">Timeout (s):</label>
                        <input type="number" step="0.1" class="w-full px-1 py-0.5 text-xs text-white accent-sensor" 
                               placeholder="0 = no timeout" data-param="timeout" value="${timeout}" min="0"
                               onclick="event.stopPropagation()"
                               style="max-width: 100%;">
                    </div>
                </div>
            `;
        } else if (blockData.type === 'read-sensor') {
            const sensorId = blockData.sensorId || '';
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-sensor text-xs">READ SENSOR</div>
                    </div>
                    <div class="flex flex-col gap-0.5">
                        <label class="text-xs text-gray-400">Sensor:</label>
                        <input type="text" class="w-full px-1 py-0.5 text-xs text-white accent-sensor" 
                               placeholder="sensor_id" data-param="sensorId" value="${sensorId}"
                               onclick="event.stopPropagation()"
                               style="max-width: 100%;">
                    </div>
                </div>
            `;
        } else if (blockData.type === 'try') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-error text-xs">TRY</div>
                    </div>
                </div>
            `;
        } else if (blockData.type === 'catch') {
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-error text-xs">CATCH</div>
                    </div>
                </div>
            `;
        } else if (blockData.type === 'throw-error') {
            const errorMessage = blockData.errorMessage || '';
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-error text-xs">THROW ERROR</div>
                    </div>
                    <input type="text" class="w-full px-1 py-0.5 text-xs text-white accent-error" 
                           placeholder="error message" data-param="errorMessage" value="${errorMessage}"
                           onclick="event.stopPropagation()"
                           style="max-width: 100%;">
                </div>
            `;
        } else if (blockData.type === 'ros-trigger') {
            const topic = blockData.topic || '/rosout';
            const expectedString = blockData.expectedString || '';
            const isCustomTopic = !['/rosout', '/motor1/status', '/motor2/status'].includes(topic);
            
            // Get message type for display
            const messageType = ROSBridge ? ROSBridge.getTopicMessageType(topic) : 'std_msgs/String';
            const messageTypeShort = messageType.split('/').pop();
            
            return `
                <div class="flex flex-col gap-1" style="font-size: 10px;">
                    <div class="flex items-center gap-2">
                        <span class="block-id-badge">#${blockData.id}</span>
                        <div class="font-semibold accent-trigger text-xs">WAIT FOR ROS TOPIC</div>
                    </div>
                    <div class="flex flex-col gap-0.5">
                        <div class="flex items-center justify-between">
                            <label class="text-xs text-gray-400">Topic:</label>
                            <span class="text-xs text-gray-500">${messageTypeShort}</span>
                        </div>
                        <select class="w-full px-1 py-0.5 text-xs text-white accent-trigger" 
                                data-param="topic"
                                onclick="event.stopPropagation()"
                                onchange="this.nextElementSibling.style.display = this.value === '/topic' ? 'block' : 'none'; this.nextElementSibling.value = this.value === '/topic' ? '' : this.value;"
                                style="max-width: 100%;">
                            <option value="/rosout" ${topic === '/rosout' ? 'selected' : ''}>/rosout (Log)</option>
                            <option value="/motor1/status" ${topic === '/motor1/status' ? 'selected' : ''}>/motor1/status</option>
                            <option value="/motor2/status" ${topic === '/motor2/status' ? 'selected' : ''}>/motor2/status</option>
                            <option value="/topic" ${isCustomTopic ? 'selected' : ''}>Custom...</option>
                        </select>
                        <input type="text" class="w-full px-1 py-0.5 text-xs text-white accent-trigger" 
                               placeholder="/custom/topic" data-param="topic" value="${isCustomTopic ? topic : ''}"
                               onclick="event.stopPropagation()"
                               style="max-width: 100%; display: ${isCustomTopic ? 'block' : 'none'};">
                    </div>
                    <div class="flex flex-col gap-0.5">
                        <label class="text-xs text-gray-400">Wait for text:</label>
                        <input type="text" class="w-full px-1 py-0.5 text-xs text-white accent-trigger" 
                               placeholder="e.g., Motor 1 finished" data-param="expectedString" value="${expectedString}"
                               onclick="event.stopPropagation()"
                               style="max-width: 100%;">
                        <div class="text-xs text-gray-500 italic">Matches if text appears in message</div>
                    </div>
                </div>
            `;
        }
        return '';
    },
    
    /**
     * Attach parameter update listeners to block inputs
     * @param {HTMLElement} blockEl - Block DOM element
     * @param {Object} blockData - Block data
     */
    attachParameterListeners(blockEl, blockData) {
        blockEl.addEventListener('input', (e) => {
            const block = WorkflowManager.blocks.get(blockData.id);
            
            if (block) {
                BlockSystem.updateBlockFromDOM(block, blockEl);
                StorageManager.autoSave();
            }
        });
    },
    
};

