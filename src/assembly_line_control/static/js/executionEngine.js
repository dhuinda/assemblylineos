/**
 * Execution Engine - Runs workflows, like Scratch but for assembly lines
 * 
 * This handles executing blocks in parallel, pausing, resuming, and making
 * sure everything happens in the right order.
 */
const ExecutionEngine = {
    isExecuting: false,
    isPaused: false, // When true, everything pauses
    pauseResolves: [], // Functions to call when we resume
    activeWorkflows: new Map(), // Which workflows are currently running
    executingBlocks: new Set(), // Which blocks are running right now
    blockStartTimes: new Map(), // When each block started
    executionStartTime: null, // When execution started overall
    updateInterval: null, // Old way of updating (kept for compatibility)
    updateAnimationFrame: null, // New way of updating (using animation frames)
    loopStack: [], // Stack of active loops: [{blockId, type, count, currentIteration, shouldBreak}]
    errorStack: [], // Stack of active try blocks: [{blockId, catchBlockId, error}]
    currentError: null, // Current error being handled
    
    /**
     * Start running all the workflows
     */
    async start() {
        if (this.isExecuting) {
            UIUtils.log('[ERROR] Already executing', 'error');
            return;
        }
        
        // Check if we're in simulation mode or connected to ROS
        const inSimulation = typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive;
        if (!inSimulation && !ROSBridge.isConnected) {
            UIUtils.log('[ERROR] Not connected to ROS Bridge and simulation mode is not active', 'error');
            return;
        }
        
        // Reset everything to a clean state
        this.isExecuting = true;
        this.isPaused = false;
        this.pauseResolves = [];
        this.activeWorkflows.clear();
        this.executingBlocks.clear();
        this.blockStartTimes.clear();
        this.executionStartTime = performance.now();
        this.loopStack = [];
        this.errorStack = [];
        this.currentError = null;
        
        // Start updating the UI to show which blocks are running
        this.startActiveBlocksUpdates();
        
        UIUtils.log('[EXECUTE] Starting execution...');
        
        // Find all workflows that should start when we hit the green flag
        const startableWorkflows = WorkflowManager.getStartableWorkflows();
        
        if (startableWorkflows.length === 0) {
            UIUtils.log('[ERROR] No green-flag workflows found', 'error');
            this.isExecuting = false;
            return;
        }
        
        // Run all the workflows at the same time (in parallel)
        UIUtils.log(`[EXECUTE] Starting ${startableWorkflows.length} workflow(s) in parallel...`, 'success');
        const workflowPromises = startableWorkflows.map(workflowId => 
            this.executeWorkflow(workflowId).catch(error => {
                UIUtils.log(`[ERROR] Workflow ${workflowId} error: ${error}`, 'error');
                throw error;
            })
        );
        
        // Wait for everything to finish
        try {
            await Promise.all(workflowPromises);
        } catch (error) {
            UIUtils.log(`[ERROR] Workflow execution error: ${error}`, 'error');
        }
        
        this.isExecuting = false;
        this.stopActiveBlocksUpdates();
        UIUtils.log(`[EXECUTE] All ${startableWorkflows.length} workflow(s) completed`, 'success');
    },
    
    /**
     * Run a workflow from start to finish
     */
    async executeWorkflow(workflowId) {
        const workflow = WorkflowManager.workflows.get(workflowId);
        if (!workflow) return;
        
        const rootBlock = WorkflowManager.blocks.get(workflow.rootBlockId);
        if (!rootBlock) return;
        
        // Keep track of this workflow's state
        const workflowState = {
            isPaused: false,
            pauseResolve: null
        };
        this.activeWorkflows.set(workflowId, workflowState);
        
        UIUtils.log(`[WORKFLOW ${workflowId}] Starting workflow from event block`);
        
        // Run all the blocks in this workflow
        await this.executeBlockChain(rootBlock.id);
        
        // This workflow is done - trigger any workflows that were waiting for it
        this.triggerWorkflowCompleteEvents(rootBlock.id);
        
        // Clean up
        this.activeWorkflows.delete(workflowId);
    },
    
    /**
     * Run a chain of blocks, starting from a root block
     * If multiple blocks connect to the root, they all run at the same time
     */
    async executeBlockChain(rootBlockId) {
        const rootBlock = WorkflowManager.blocks.get(rootBlockId);
        if (!rootBlock) return;
        
        // If the root is an event block, we start from whatever comes after it
        let startBlockIds = [];
        if (rootBlock.type === 'event') {
            const nextBlocks = BlockConnector.getNextBlocks(rootBlockId);
            if (nextBlocks.length === 0) {
                // Nothing connected to this event block
                UIUtils.log(`[WORKFLOW] Event block ${rootBlockId} has no connected blocks`, 'warning');
                return;
            }
            startBlockIds = nextBlocks;
        } else {
            startBlockIds = [rootBlockId];
        }
        
        // Run all the starting blocks at the same time
        await this.executeBlocksInParallel(startBlockIds);
    },
    
    /**
     * Run multiple blocks at the same time
     */
    async executeBlocksInParallel(blockIds) {
        if (!blockIds || blockIds.length === 0) return;
        
        // Start all the blocks running at once
        const executionPromises = blockIds.map(blockId => this.executeBlockAndContinue(blockId));
        
        try {
            await Promise.all(executionPromises);
        } catch (error) {
            UIUtils.log(`[ERROR] Parallel execution error: ${error}`, 'error');
        }
    },
    
    /**
     * Execute a block and continue to its next blocks (supports parallel continuation)
     */
    async executeBlockAndContinue(blockId) {
        if (!this.isExecuting) return;
        
        const block = WorkflowManager.blocks.get(blockId);
        if (!block) return;
        
        // Check if we're in a loop and this block is the loop block itself
        // If so, we need to handle loop continuation differently
        if (this.loopStack.length > 0) {
            const currentLoop = this.loopStack[this.loopStack.length - 1];
            if (blockId === currentLoop.blockId && (block.type === 'repeat' || block.type === 'forever')) {
                // This is the loop block itself - don't execute it again, just continue the loop
                // The loop execution is handled in executeBlock()
                return;
            }
        }
        
        // If everything is paused, wait here until we resume
        if (this.isPaused) {
            await new Promise(resolve => {
                // Save this function so resume() can call it when we're ready to continue
                this.pauseResolves.push(resolve);
            });
        }
        
        // Check if this specific workflow is paused (old way, kept for compatibility)
        const workflowId = block.workflowId;
        if (workflowId) {
            const workflowState = this.activeWorkflows.get(workflowId);
            if (workflowState && workflowState.isPaused) {
                await new Promise(resolve => {
                    workflowState.pauseResolve = resolve;
                });
            }
        }
        
        // Actually run the block
        await this.executeBlock(blockId);
        
        // This block is done - check if any workflows were waiting for it
        this.triggerWorkflowCompleteEvents(blockId);
        
        // Check if we're in a loop and this block connects back to the loop
        if (this.loopStack.length > 0) {
            const currentLoop = this.loopStack[this.loopStack.length - 1];
            const nextBlocks = BlockConnector.getNextBlocks(blockId);
            const connectsBackToLoop = nextBlocks.includes(currentLoop.blockId);
            
            if (connectsBackToLoop && !currentLoop.shouldBreak) {
                // This block connects back to the loop - don't continue to next blocks
                // The loop will handle the next iteration
                return;
            }
        }
        
        // Now run whatever blocks come next (all at the same time if there are multiple)
        const nextBlocks = BlockConnector.getNextBlocks(blockId);
        if (nextBlocks.length > 0) {
            await this.executeBlocksInParallel(nextBlocks);
        }
    },
    
    /**
     * Execute a single block
     */
    async executeBlock(blockId) {
        const block = WorkflowManager.blocks.get(blockId);
        if (!block) return;
        
        // Mark this block as running
        this.executingBlocks.add(blockId);
        this.blockStartTimes.set(blockId, performance.now());
        this.markBlockExecuting(blockId, true);
        
        try {
            if (block.type === 'motor') {
                // Use custom speed if set, otherwise use global motor speed
                const hasCustomSpeed = block.speed !== undefined && block.speed !== null;
                const motorSpeed = hasCustomSpeed ? block.speed : MotorSpeedManager.getSpeed(block.motor_id);
                const speedLabel = hasCustomSpeed ? 'custom' : 'global';
                
                const success = ROSBridge.publishMotorCommand(block.motor_id, block.steps || 0, motorSpeed);
                if (success) {
                    const duration = motorSpeed > 0 ? (Math.abs(block.steps || 0) / motorSpeed) : 0;
                    UIUtils.log(`  → Motor ${block.motor_id}: ${block.steps || 0} steps (${duration.toFixed(2)}s @ ${motorSpeed} sps ${speedLabel})`, 'success');
                    // Wait for motor to complete - parallel branches will wait concurrently via Promise.all
                    await this.sleep(duration * 1000);
                }
            } else if (block.type === 'relay') {
                const success = ROSBridge.publishRelayCommand(block.relay_id, block.state || 'off');
                if (success) {
                    UIUtils.log(`  → Relay ${block.relay_id}: ${(block.state || 'off').toUpperCase()}`, 'success');
                }
            } else if (block.type === 'delay') {
                const duration = block.duration || 1.0;
                UIUtils.log(`  → Delay: ${duration.toFixed(2)}s`, 'success');
                await this.sleep(duration * 1000);
            } else if (block.type === 'pause') {
                // Pause everything and stop all motors
                // Only do this if we're not already paused (so multiple pause blocks don't interfere)
                if (!this.isPaused) {
                    this.isPaused = true;
                    ROSBridge.stopAllMotors();
                    UIUtils.showPauseOverlay(true);
                    UIUtils.log(`[PAUSE] All workflows paused at block ${blockId} - All motors stopped`, 'warning');
                }
                
                // Wait here until someone hits resume
                await new Promise(resolve => {
                    this.pauseResolves.push(resolve);
                });
                
                // Note: The actual resume happens in the resume() function, not here
            } else if (block.type === 'ros-trigger') {
                // Wait for a specific message from a ROS topic
                const topic = block.topic || '/topic';
                const expectedString = block.expectedString || '';
                
                if (!topic || topic.trim() === '') {
                    UIUtils.log(`[ERROR] ROS trigger block ${blockId} missing topic name`, 'error');
                    return;
                }
                
                if (expectedString === '') {
                    UIUtils.log(`[ERROR] ROS trigger block ${blockId} missing expected string`, 'error');
                    return;
                }
                
                // Only process messages that arrive after this block started
            } else if (block.type === 'repeat') {
                // REPEAT loop: execute body N times
                const count = block.count || 10;
                UIUtils.log(`  → REPEAT: ${count} times`, 'success');
                
                // Push loop onto stack
                const loopState = {
                    blockId: blockId,
                    type: 'repeat',
                    count: count,
                    currentIteration: 0,
                    shouldBreak: false
                };
                this.loopStack.push(loopState);
                
                // Execute loop body
                for (let i = 0; i < count; i++) {
                    if (!this.isExecuting) break;
                    if (loopState.shouldBreak) break;
                    
                    loopState.currentIteration = i + 1;
                    UIUtils.log(`  → REPEAT iteration ${loopState.currentIteration}/${count}`, 'info');
                    
                    // Execute the body (blocks connected to this repeat block)
                    const bodyBlocks = BlockConnector.getNextBlocks(blockId);
                    if (bodyBlocks.length > 0) {
                        // Execute body blocks and wait for them to complete
                        // They may connect back to the loop block, which will be handled in executeBlockAndContinue
                        await this.executeBlocksInParallel(bodyBlocks);
                    }
                    
                    // Small delay between iterations to prevent tight loop
                    await this.sleep(10);
                }
                
                // Pop loop from stack
                this.loopStack.pop();
                UIUtils.log(`  → REPEAT completed`, 'success');
            } else if (block.type === 'forever') {
                // FOREVER loop: execute body until BREAK
                UIUtils.log(`  → FOREVER: starting infinite loop`, 'success');
                
                // Push loop onto stack
                const loopState = {
                    blockId: blockId,
                    type: 'forever',
                    count: Infinity,
                    currentIteration: 0,
                    shouldBreak: false
                };
                this.loopStack.push(loopState);
                
                // Execute loop body until break
                while (this.isExecuting && !loopState.shouldBreak) {
                    loopState.currentIteration++;
                    UIUtils.log(`  → FOREVER iteration ${loopState.currentIteration}`, 'info');
                    
                    // Execute the body (blocks connected to this forever block)
                    const bodyBlocks = BlockConnector.getNextBlocks(blockId);
                    if (bodyBlocks.length > 0) {
                        // Execute body blocks - they will handle connecting back to loop
                        await this.executeBlocksInParallel(bodyBlocks);
                    }
                    
                    // Small delay to prevent tight loop
                    await this.sleep(10);
                }
                
                // Pop loop from stack
                this.loopStack.pop();
                UIUtils.log(`  → FOREVER completed`, 'success');
            } else if (block.type === 'break') {
                // BREAK: exit the current loop
                if (this.loopStack.length > 0) {
                    const currentLoop = this.loopStack[this.loopStack.length - 1];
                    currentLoop.shouldBreak = true;
                    UIUtils.log(`  → BREAK: exiting ${currentLoop.type} loop (was at iteration ${currentLoop.currentIteration})`, 'warning');
                } else {
                    UIUtils.log(`  → BREAK: no active loop to break`, 'warning');
                }
            } else if (block.type === 'ros-trigger') {
                // Wait for a specific message from a ROS topic
                const topic = block.topic || '/topic';
                const expectedString = block.expectedString || '';
                
                if (!topic || topic.trim() === '') {
                    UIUtils.log(`[ERROR] ROS trigger block ${blockId} missing topic name`, 'error');
                    return;
                }
                
                if (expectedString === '') {
                    UIUtils.log(`[ERROR] ROS trigger block ${blockId} missing expected string`, 'error');
                    return;
                }
                
                // Only process messages that arrive after this block started
                const blockStartTime = this.blockStartTimes.get(blockId) || Date.now();
                
                // Figure out what type of message this topic uses
                const messageType = ROSBridge.getTopicMessageType(topic);
                UIUtils.log(`  → Block ${blockId}: Waiting for "${expectedString}" on topic ${topic} (${messageType})...`, 'success');
                UIUtils.log(`  → Block ${blockId}: Execution is now BLOCKED until message received`, 'warning');
                UIUtils.log(`  → Block ${blockId}: Only processing messages received after ${new Date(blockStartTime).toISOString()}`, 'info');
                
                try {
                    // Wait here until we get the message we're looking for
                    const receivedString = await ROSBridge.waitForTopicString(topic, expectedString, messageType, blockStartTime);
                    UIUtils.log(`  → Block ${blockId}: Received "${receivedString}" on topic ${topic} - Continuing execution`, 'success');
                } catch (error) {
                    UIUtils.log(`[ERROR] ROS trigger block ${blockId} error: ${error}`, 'error');
                    // Keep going even if there was an error - don't stop the whole workflow
                }
            } else if (block.type === 'wait-sensor') {
                // Wait for sensor condition
                const sensorId = block.sensorId || '';
                const condition = block.condition || 'HIGH';
                const threshold = block.threshold || 0.0;
                const timeout = block.timeout || 0.0;
                
                if (!sensorId) {
                    UIUtils.log(`[ERROR] Wait sensor block ${blockId} missing sensor ID`, 'error');
                    return;
                }
                
                UIUtils.log(`  → Block ${blockId}: Waiting for sensor ${sensorId} (${condition})...`, 'success');
                
                try {
                    // Check if simulation mode is active
                    if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
                        await SimulationEngine.waitForSensor(sensorId, condition, threshold, timeout * 1000);
                        UIUtils.log(`  → Block ${blockId}: Sensor ${sensorId} condition met`, 'success');
                    } else {
                        // Use ROS Bridge to wait for sensor
                        await ROSBridge.waitForSensor(sensorId, condition, threshold, timeout * 1000);
                        UIUtils.log(`  → Block ${blockId}: Sensor ${sensorId} condition met`, 'success');
                    }
                } catch (error) {
                    UIUtils.log(`[ERROR] Wait sensor block ${blockId} error: ${error}`, 'error');
                    // Keep going even if there was an error - don't stop the whole workflow
                }
            } else if (block.type === 'read-sensor') {
                // Read sensor value
                const sensorId = block.sensorId || '';
                
                if (!sensorId) {
                    UIUtils.log(`[ERROR] Read sensor block ${blockId} missing sensor ID`, 'error');
                    return;
                }
                
                try {
                    let value = null;
                    // Check if simulation mode is active
                    if (typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive) {
                        value = SimulationEngine.getSensorValue(sensorId);
                    } else {
                        value = ROSBridge.readSensor(sensorId);
                    }
                    
                    if (value !== null) {
                        UIUtils.log(`  → Block ${blockId}: Sensor ${sensorId} value: ${value}`, 'success');
                    } else {
                        UIUtils.log(`[WARNING] Sensor ${sensorId} not found or not connected`, 'warning');
                    }
                } catch (error) {
                    UIUtils.log(`[ERROR] Read sensor block ${blockId} error: ${error}`, 'error');
                }
            } else if (block.type === 'try') {
                // TRY block: start error handling scope
                UIUtils.log(`  → TRY: starting error handling scope`, 'success');
                
                // Find the associated CATCH block
                const nextBlocks = BlockConnector.getNextBlocks(blockId);
                let catchBlockId = null;
                
                // Look for CATCH block in the next blocks
                for (const nextId of nextBlocks) {
                    const nextBlock = WorkflowManager.blocks.get(nextId);
                    if (nextBlock && nextBlock.type === 'catch') {
                        catchBlockId = nextId;
                        break;
                    }
                }
                
                // Push try block onto error stack
                const tryState = {
                    blockId: blockId,
                    catchBlockId: catchBlockId,
                    error: null
                };
                this.errorStack.push(tryState);
                
                // Execute the try body (blocks connected to TRY, but not the CATCH)
                const tryBodyBlocks = nextBlocks.filter(id => id !== catchBlockId);
                if (tryBodyBlocks.length > 0) {
                    try {
                        await this.executeBlocksInParallel(tryBodyBlocks);
                    } catch (error) {
                        // Error occurred - set error state
                        tryState.error = error;
                        this.currentError = error;
                        UIUtils.log(`  → TRY: error caught: ${error.message || error}`, 'error');
                        
                        // Execute CATCH block if it exists
                        if (catchBlockId) {
                            const catchBodyBlocks = BlockConnector.getNextBlocks(catchBlockId);
                            if (catchBodyBlocks.length > 0) {
                                await this.executeBlocksInParallel(catchBodyBlocks);
                            }
                        }
                    }
                }
                
                // Pop try block from error stack
                this.errorStack.pop();
                if (this.errorStack.length === 0) {
                    this.currentError = null;
                }
                
                UIUtils.log(`  → TRY: error handling scope completed`, 'success');
            } else if (block.type === 'catch') {
                // CATCH block: this is handled by the TRY block
                // Just log that we're in the catch handler
                UIUtils.log(`  → CATCH: error handler`, 'warning');
            } else if (block.type === 'throw-error') {
                // THROW ERROR: manually trigger an error
                const errorMessage = block.errorMessage || 'Error thrown';
                UIUtils.log(`  → THROW ERROR: ${errorMessage}`, 'error');
                
                // Throw error to be caught by nearest TRY block
                throw new Error(errorMessage);
            } else if (block.type === 'event') {
                // Event blocks don't do anything themselves - they just start workflows
                UIUtils.log(`  → Event: ${block.eventType || 'unknown'}`, 'success');
            }
        } finally {
            // Mark this block as done
            this.executingBlocks.delete(blockId);
            this.blockStartTimes.delete(blockId);
            this.markBlockExecuting(blockId, false);
        }
    },
    
    /**
     * Check if any workflows are waiting for this block to finish, and start them
     */
    triggerWorkflowCompleteEvents(blockId) {
        // Check if this is the last block in a chain (nothing comes after it)
        const nextBlocks = BlockConnector.getNextBlocks(blockId);
        const isEndOfChain = nextBlocks.length === 0;
        
        // Look for workflows that are waiting for this block to complete
        WorkflowManager.workflows.forEach((workflow, workflowId) => {
            // Skip workflows that are already running
            if (this.activeWorkflows.has(workflowId)) return;
            
            const rootBlock = WorkflowManager.blocks.get(workflow.rootBlockId);
            if (rootBlock && 
                rootBlock.type === 'event' && 
                rootBlock.eventType === 'workflow-complete') {
                
                // Check if this workflow is waiting for this specific block
                if (rootBlock.triggeredBy === blockId || workflow.triggeredBy === blockId) {
                    // Start the workflow
                    UIUtils.log(`[WORKFLOW ${workflowId}] Triggered by block ${blockId} completion`);
                    this.executeWorkflow(workflowId);
                }
            }
        });
        
        // Also check for workflows triggered by this block (block-triggered workflows)
        const blockTriggeredWorkflows = WorkflowManager.getWorkflowsTriggeredByBlock(blockId);
        blockTriggeredWorkflows.forEach(workflowId => {
            // Skip if already active
            if (this.activeWorkflows.has(workflowId)) return;
            
            UIUtils.log(`[WORKFLOW ${workflowId}] Triggered by block ${blockId} completion`);
            this.executeWorkflow(workflowId);
        });
        
        // Check for workflows that are linked to the workflow containing this block
        const block = WorkflowManager.blocks.get(blockId);
        if (block && block.workflowId) {
            // This block is the last in a workflow, check if any workflows are waiting for this workflow
            WorkflowManager.workflows.forEach((targetWorkflow, targetWorkflowId) => {
                if (this.activeWorkflows.has(targetWorkflowId)) return;
                if (targetWorkflow.triggeredBy === blockId) {
                    // This workflow is triggered when the block completes
                    UIUtils.log(`[WORKFLOW ${targetWorkflowId}] Triggered by workflow completion (block ${blockId})`);
                    this.executeWorkflow(targetWorkflowId);
                }
            });
        }
    },
    
    /**
     * Mark a block as executing (visual feedback)
     */
    markBlockExecuting(blockId, isExecuting) {
        const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
        if (blockEl) {
            if (isExecuting) {
                blockEl.classList.add('executing');
                // Update active blocks panel if available
                this.updateActiveBlocksPanel();
            } else {
                blockEl.classList.remove('executing');
                // Update active blocks panel if available
                this.updateActiveBlocksPanel();
            }
        }
    },
    
    /**
     * Update the playback panel to show currently executing blocks with timing info
     * Optimized with DOM caching and batch updates
     */
    updateActiveBlocksPanel() {
        const activePanel = document.getElementById('activeBlocksPanel');
        if (!activePanel) return;
        
        // Calculate total elapsed time (use performance.now() for higher precision)
        const now = performance.now();
        const executionStart = this.executionStartTime || now;
        const totalElapsed = this.executionStartTime ? ((now - executionStart) / 1000) : 0;
        
        if (this.executingBlocks.size === 0 && totalElapsed === 0) {
            // Only update if content changed to avoid unnecessary DOM manipulation
            if (activePanel.innerHTML !== '<p class="text-xs text-gray-500 text-center py-4">No blocks executing</p>') {
                activePanel.innerHTML = '<p class="text-xs text-gray-500 text-center py-4">No blocks executing</p>';
            }
            return;
        }
        
        // Use DocumentFragment for batch DOM updates (more efficient)
        const fragment = document.createDocumentFragment();
        
        // Show total elapsed time at the top
        if (this.executionStartTime) {
            const elapsedEl = document.createElement('div');
            elapsedEl.className = 'text-xs text-gray-400 mb-3 pb-2 border-b border-gray-700';
            elapsedEl.innerHTML = `<span class="font-semibold text-white">Total Elapsed:</span> <span class="text-green-400">${totalElapsed.toFixed(2)}s</span>`;
            fragment.appendChild(elapsedEl);
        }
        
        // Show each executing block
        this.executingBlocks.forEach(blockId => {
            const block = WorkflowManager.blocks.get(blockId);
            if (!block) return;
            
            const startTime = this.blockStartTimes.get(blockId);
            // Use performance.now() for higher precision timing
            const blockElapsed = startTime ? ((now - startTime) / 1000) : 0;
            
            const blockEl = document.createElement('div');
            const typeClass = BlockSystem.getBlockTypeClass(block.type);
            blockEl.className = `p-2 ${typeClass} card mb-2`;
            
            let content = '';
            let estimatedDuration = 0;
            let timeRemaining = 0;
            
            if (block.type === 'motor') {
                // Use custom speed if set, otherwise use global motor speed
                const hasCustomSpeed = block.speed !== undefined && block.speed !== null;
                const motorSpeed = hasCustomSpeed ? block.speed : MotorSpeedManager.getSpeed(block.motor_id);
                const speedLabel = hasCustomSpeed ? 'custom' : 'global';
                
                const motorStatus = ROSBridge.getMotorStatus(block.motor_id);
                const stepsRemaining = motorStatus ? motorStatus.steps_remaining : null;
                const totalSteps = block.steps || 0;
                
                // Calculate estimated duration and remaining time
                estimatedDuration = motorSpeed > 0 ? (Math.abs(totalSteps) / motorSpeed) : 0;
                
                if (stepsRemaining !== null && stepsRemaining !== undefined && motorSpeed > 0) {
                    // Calculate remaining time based on actual steps remaining
                    timeRemaining = Math.abs(stepsRemaining) / motorSpeed;
                } else {
                    // Estimate based on elapsed time and total duration
                    timeRemaining = Math.max(0, estimatedDuration - blockElapsed);
                }
                
                let stepsDisplay = `${totalSteps} steps`;
                if (stepsRemaining !== null && stepsRemaining !== undefined) {
                    // Round to avoid display glitches from floating point values
                    const displayRemaining = Math.round(Math.abs(stepsRemaining));
                    stepsDisplay = `${displayRemaining} remaining / ${totalSteps} total`;
                }
                
                content = `
                    <div class="flex items-center justify-between text-xs mb-1">
                        <span class="accent-motor font-semibold">MOTOR ${block.motor_id}</span>
                        <span class="text-gray-400 font-mono">#${blockId}</span>
                    </div>
                    <div class="text-xs text-gray-400 mb-1">${stepsDisplay}</div>
                    <div class="flex items-center justify-between text-xs text-gray-500">
                        <span>Elapsed: ${blockElapsed.toFixed(2)}s</span>
                        <span>Remaining: ${timeRemaining.toFixed(2)}s</span>
                    </div>
                    <div class="text-xs text-gray-500 mt-1">Est. total: ${estimatedDuration.toFixed(2)}s @ ${motorSpeed} sps (${speedLabel})</div>
                `;
            } else if (block.type === 'relay') {
                estimatedDuration = 0; // Relays are instant
                timeRemaining = 0;
                
                content = `
                    <div class="flex items-center justify-between text-xs mb-1">
                        <span class="accent-relay font-semibold">RELAY ${block.relay_id}</span>
                        <span class="text-gray-400 font-mono">#${blockId}</span>
                    </div>
                    <div class="text-xs text-gray-400 mb-1">${(block.state || 'off').toUpperCase()}</div>
                    <div class="text-xs text-gray-500">Elapsed: ${blockElapsed.toFixed(2)}s</div>
                `;
            } else if (block.type === 'delay') {
                estimatedDuration = block.duration || 1.0;
                timeRemaining = Math.max(0, estimatedDuration - blockElapsed);
                
                content = `
                    <div class="flex items-center justify-between text-xs mb-1">
                        <span class="accent-delay font-semibold">DELAY</span>
                        <span class="text-gray-400 font-mono">#${blockId}</span>
                    </div>
                    <div class="flex items-center justify-between text-xs text-gray-500">
                        <span>Elapsed: ${blockElapsed.toFixed(2)}s</span>
                        <span>Remaining: ${timeRemaining.toFixed(2)}s</span>
                    </div>
                    <div class="text-xs text-gray-500 mt-1">Est. total: ${estimatedDuration.toFixed(2)}s</div>
                `;
            } else if (block.type === 'pause') {
                estimatedDuration = Infinity; // Pause is indefinite
                timeRemaining = Infinity;
                
                content = `
                    <div class="flex items-center justify-between text-xs mb-1">
                        <span class="accent-pause font-semibold">PAUSE</span>
                        <span class="text-gray-400 font-mono">#${blockId}</span>
                    </div>
                    <div class="text-xs text-gray-500">Elapsed: ${blockElapsed.toFixed(2)}s</div>
                    <div class="text-xs text-yellow-400 mt-1">Waiting for resume...</div>
                `;
            } else if (block.type === 'ros-trigger') {
                estimatedDuration = Infinity; // Wait is indefinite until message received
                timeRemaining = Infinity;
                
                const topic = block.topic || '/topic';
                const expectedString = block.expectedString || '';
                
                content = `
                    <div class="flex items-center justify-between text-xs mb-1">
                        <span class="accent-trigger font-semibold">WAIT FOR ROS TOPIC</span>
                        <span class="text-gray-400 font-mono">#${blockId}</span>
                    </div>
                    <div class="text-xs text-gray-400 mb-1">Topic: ${topic}</div>
                    <div class="text-xs text-gray-400 mb-1">Waiting for: "${expectedString}"</div>
                    <div class="text-xs text-gray-500">Elapsed: ${blockElapsed.toFixed(2)}s</div>
                    <div class="text-xs text-yellow-400 mt-1">Waiting for message...</div>
                `;
            } else if (block.type === 'event') {
                estimatedDuration = 0; // Events are instant
                timeRemaining = 0;
                
                content = `
                    <div class="flex items-center justify-between text-xs mb-1">
                        <span class="accent-trigger font-semibold">EVENT</span>
                        <span class="text-gray-400 font-mono">#${blockId}</span>
                    </div>
                    <div class="text-xs text-gray-400 mb-1">${block.eventType || 'unknown'}</div>
                    <div class="text-xs text-gray-500">Elapsed: ${blockElapsed.toFixed(2)}s</div>
                `;
            }
            blockEl.innerHTML = content;
            fragment.appendChild(blockEl);
        });
        
        // Show count if multiple blocks executing in parallel
        if (this.executingBlocks.size > 1) {
            const countEl = document.createElement('div');
            countEl.className = 'text-xs text-gray-400 text-center mt-2 pt-2 border-t border-gray-700';
            countEl.textContent = `${this.executingBlocks.size} blocks executing in parallel`;
            fragment.appendChild(countEl);
        }
        
        // Batch update: clear and append fragment in one operation
        activePanel.innerHTML = '';
        activePanel.appendChild(fragment);
    },
    
    /**
     * Resume paused execution
     */
    resume() {
        // Resume global pause (affects ALL workflows)
        // Resolve ALL pending pause promises to ensure nothing gets stuck
        if (this.isPaused) {
            this.isPaused = false;
            UIUtils.showPauseOverlay(false);
            UIUtils.log(`[RESUME] All workflows resumed`, 'success');
            
            // Resolve all pending promises (there may be multiple if multiple pause blocks are waiting)
            const resolves = [...this.pauseResolves]; // Copy array
            this.pauseResolves = []; // Clear array before resolving to prevent race conditions
            resolves.forEach(resolve => {
                try {
                    resolve();
                } catch (error) {
                    console.error('[ERROR] Error resolving pause promise:', error);
                }
            });
        }
        
        // Also resume individual workflow pauses (legacy support)
        this.activeWorkflows.forEach((workflowState, workflowId) => {
            if (workflowState.isPaused && workflowState.pauseResolve) {
                workflowState.isPaused = false;
                const resolve = workflowState.pauseResolve;
                workflowState.pauseResolve = null;
                try {
                    resolve();
                } catch (error) {
                    console.error(`[ERROR] Error resolving workflow ${workflowId} pause promise:`, error);
                }
            }
        });
    },
    
    /**
     * Stop all execution
     */
    stop() {
        this.isExecuting = false;
        
        // Clear all executing blocks
        this.executingBlocks.forEach(blockId => {
            this.markBlockExecuting(blockId, false);
        });
        this.executingBlocks.clear();
        this.blockStartTimes.clear();
        this.executionStartTime = null;
        
        // Clear active blocks panel
        this.updateActiveBlocksPanel();
        
        // Clean up any active ROS topic subscriptions
        // Note: We don't clean up all subscriptions, only those that were created during execution
        // The rosBridge will handle cleanup on disconnect
        
        // Resume all paused workflows to allow them to exit
        this.activeWorkflows.forEach((workflowState, workflowId) => {
            if (workflowState.isPaused && workflowState.pauseResolve) {
                workflowState.isPaused = false;
                const resolve = workflowState.pauseResolve;
                workflowState.pauseResolve = null;
                resolve();
            }
        });
        
        this.activeWorkflows.clear();
        this.isPaused = false;
        // Resolve any pending pause promises to prevent them from getting stuck
        const resolves = [...this.pauseResolves];
        this.pauseResolves = [];
        resolves.forEach(resolve => {
            try {
                resolve();
            } catch (error) {
                console.error('[ERROR] Error resolving pause promise on stop:', error);
            }
        });
        this.stopActiveBlocksUpdates();
        UIUtils.showPauseOverlay(false);
        UIUtils.log('[EXECUTE] Execution stopped', 'warning');
    },
    
    /**
     * Start periodic updates for active blocks panel
     * Uses requestAnimationFrame for smoother, more efficient updates
     */
    startActiveBlocksUpdates() {
        // Use requestAnimationFrame for smoother updates
        // Match the ROS status update rate (10Hz) to avoid display jitter
        let lastUpdateTime = 0;
        const targetFPS = 10; // Update at 10fps to match ROS publisher rate (100ms)
        const minUpdateInterval = 1000 / targetFPS;
        
        const updateLoop = (currentTime) => {
            if (!this.isExecuting) {
                this.updateAnimationFrame = null;
                return;
            }
            
            // Throttle updates to target FPS
            if (currentTime - lastUpdateTime >= minUpdateInterval) {
                if (this.executingBlocks.size > 0) {
                    this.updateActiveBlocksPanel();
                }
                lastUpdateTime = currentTime;
            }
            
            // Continue animation loop
            this.updateAnimationFrame = requestAnimationFrame(updateLoop);
        };
        
        this.updateAnimationFrame = requestAnimationFrame(updateLoop);
    },
    
    /**
     * Stop periodic updates for active blocks panel
     */
    stopActiveBlocksUpdates() {
        if (this.updateAnimationFrame !== null && this.updateAnimationFrame !== undefined) {
            cancelAnimationFrame(this.updateAnimationFrame);
            this.updateAnimationFrame = null;
        }
        // Legacy support for setInterval (if it exists)
        if (this.updateInterval) {
            clearInterval(this.updateInterval);
            this.updateInterval = null;
        }
    },
    
    /**
     * Sleep utility with high precision timing for real-time control
     * Uses performance.now() for more accurate timing than setTimeout
     * @param {number} ms - Milliseconds to sleep
     */
    sleep(ms) {
        return new Promise(resolve => {
            if (ms <= 0) {
                resolve();
                return;
            }
            
            const start = performance.now();
            const checkInterval = () => {
                const elapsed = performance.now() - start;
                if (elapsed >= ms) {
                    resolve();
                } else {
                    // Use requestAnimationFrame for short delays (< 16ms) for better precision
                    // Otherwise use setTimeout with smaller intervals for longer delays
                    if (ms < 16) {
                        requestAnimationFrame(checkInterval);
                    } else {
                        setTimeout(checkInterval, Math.min(ms - elapsed, 16));
                    }
                }
            };
            
            if (ms < 16) {
                requestAnimationFrame(checkInterval);
            } else {
                setTimeout(checkInterval, Math.min(ms, 16));
            }
        });
    },
    
    /**
     * Build execution order for playback visualization
     * Returns an array of {block, lane, startTime, endTime} objects
     */
    buildExecutionOrder() {
        const executionOrder = [];
        const startableWorkflows = WorkflowManager.getStartableWorkflows();
        
        // Process each workflow
        startableWorkflows.forEach((workflowId, laneIndex) => {
            const workflow = WorkflowManager.workflows.get(workflowId);
            if (!workflow) return;
            
            // Find the root block (green flag event)
            let rootBlockId = null;
            workflow.blocks.forEach(blockId => {
                const block = WorkflowManager.blocks.get(blockId);
                if (block && block.type === 'event' && block.eventType === 'green-flag') {
                    rootBlockId = blockId;
                }
            });
            
            if (!rootBlockId) return;
            
            // Walk through the chain of blocks
            let currentTime = 0;
            let currentBlockId = rootBlockId;
            const visited = new Set();
            
            while (currentBlockId && !visited.has(currentBlockId)) {
                visited.add(currentBlockId);
                const block = WorkflowManager.blocks.get(currentBlockId);
                if (!block) break;
                
                // Calculate block duration
                let duration = 0.01; // Minimum duration for instant blocks
                
                if (block.type === 'motor') {
                    const hasCustomSpeed = block.speed !== undefined && block.speed !== null;
                    const motorSpeed = hasCustomSpeed ? block.speed : MotorSpeedManager.getSpeed(block.motor_id);
                    duration = motorSpeed > 0 ? Math.abs(block.steps || 0) / motorSpeed : 0.01;
                } else if (block.type === 'delay') {
                    duration = block.duration || 1.0;
                } else if (block.type === 'pause') {
                    duration = 2.0; // Assume pause is ~2 seconds for visualization
                }
                
                // Add to execution order
                executionOrder.push({
                    block: currentBlockId,
                    lane: laneIndex,
                    startTime: currentTime,
                    endTime: currentTime + Math.max(0.01, duration)
                });
                
                currentTime += Math.max(0.01, duration);
                
                // Get next block in chain
                const connections = BlockConnector.connections.get(currentBlockId);
                if (connections && connections.next) {
                    const nextBlocks = Array.isArray(connections.next) ? connections.next : [connections.next];
                    currentBlockId = nextBlocks[0]; // Follow the first connection for now
                } else {
                    currentBlockId = null;
                }
            }
        });
        
        return executionOrder;
    }
};
