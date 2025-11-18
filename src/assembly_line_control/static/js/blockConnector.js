/**
 * Block Connector - Handles Scratch-like block snapping and connections
 */
const BlockConnector = {
    connections: new Map(), // blockId -> {prev: blockId | null, next: blockId[] | blockId | null}
    customPaths: new Map(), // connectionKey -> {waypoints: [{x, y}], enabled: boolean}
    snapDistance: 50, // pixels - distance threshold for snapping blocks together
    connectionLines: new Map(), // blockId -> HTMLElement
    connectingFrom: null, // {blockId, connectorType: 'bottom'|'top'} - current connection being made
    previewLine: null, // Preview connection line when dragging
    previewTarget: null, // Target block for preview connection
    autoWireEnabled: true, // Auto-wire mode toggle
    selectedWaypoint: null, // Currently selected waypoint for editing
    editingConnection: null, // Currently editing connection key
    
    // Performance optimization caches
    blockPositionCache: new Map(), // blockId -> position data
    blockElementCache: new Map(), // blockId -> DOM element
    
    /**
     * Check if a block can snap to nearby blocks
     */
    checkSnapping(blockEl, blockData) {
        if (!blockEl || !blockData) return false;
        
        // Remove snapping class from all blocks
        document.querySelectorAll('.scratch-block').forEach(el => {
            el.classList.remove('snapping');
        });
        
        // Event blocks can't snap below other blocks (they are roots)
        if (blockData.type === 'event') return false;
        
        const blockPos = this.getBlockPosition(blockEl);
        if (!blockPos) return false;
        
        let foundSnap = false;
        let bestTarget = null;
        let minDistance = Infinity;
        
        WorkflowManager.blocks.forEach((otherBlock, otherId) => {
            if (otherId === blockData.id) return;
            
            // Don't snap if already connected
            if (this.isConnected(blockData.id, otherId)) return;
            
            const otherEl = document.querySelector(`[data-block-id="${otherId}"]`);
            if (!otherEl) return;
            
            const otherPos = this.getBlockPosition(otherEl);
            if (!otherPos) return;
            
            const dx = Math.abs(blockPos.centerX - otherPos.centerX);
            
            // Check if this block should connect ABOVE the other block
            // (this block's bottom is near other block's top)
            // Note: Event blocks can't have blocks above them, so skip this case for event blocks
            if (otherBlock.type !== 'event') {
                const dyAbove = otherPos.top - blockPos.bottom;
                if (dx < this.snapDistance && dyAbove > 0 && dyAbove < this.snapDistance) {
                    const distance = Math.sqrt(dx * dx + dyAbove * dyAbove);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestTarget = { block: otherBlock, element: otherEl, dx, dy: dyAbove };
                        foundSnap = true;
                    }
                }
            }
            
            // Check if this block should connect BELOW the other block
            // (this block's top is near other block's bottom)
            // This works for both regular blocks and event blocks (event blocks can have blocks below them)
            const dyBelow = blockPos.top - otherPos.bottom;
            if (dx < this.snapDistance && dyBelow > 0 && dyBelow < this.snapDistance) {
                const distance = Math.sqrt(dx * dx + dyBelow * dyBelow);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestTarget = { block: otherBlock, element: otherEl, dx, dy: dyBelow };
                    foundSnap = true;
                }
            }
        });
        
        if (foundSnap && bestTarget) {
            blockEl.classList.add('snapping');
            bestTarget.element.classList.add('snapping');
            
            // Draw preview connection line
            this.drawPreviewConnection(blockEl, blockPos, bestTarget.element, bestTarget);
            this.previewTarget = { block: bestTarget.block, element: bestTarget.element, dx: bestTarget.dx, dy: bestTarget.dy };
        } else {
            // Remove preview line if no snap target
            this.clearPreviewConnection();
            this.previewTarget = null;
        }
        
        return foundSnap;
    },
    
    /**
     * Draw a preview connection line when dragging near another block
     */
    drawPreviewConnection(fromEl, fromPos, toEl, targetInfo) {
        // Remove existing preview
        this.clearPreviewConnection();
        
        const toPos = this.getBlockPosition(toEl);
        if (!toPos) return;
        
        const canvas = document.getElementById('workspaceCanvas');
        if (!canvas) return;
        
        // Connections always go from bottom connector to top connector
        // Determine which block is above and which is below
        // dyAbove: positive when fromEl is above toEl (fromEl.bottom < toEl.top)
        // dyBelow: positive when fromEl is below toEl (fromEl.top > toEl.bottom)
        
        // Check if fromEl is above toEl by comparing positions
        const fromElIsAbove = fromPos.bottom < toPos.top;
        
        let fromX, fromY, toX, toY;
        
        if (fromElIsAbove) {
            // From block is above: connect from its bottom to target's top
            fromX = fromPos.centerX;
            fromY = fromPos.bottom;
            toX = toPos.centerX;
            toY = toPos.top;
        } else {
            // From block is below: connect from target's bottom to from block's top
            fromX = toPos.centerX;
            fromY = toPos.bottom;
            toX = fromPos.centerX;
            toY = fromPos.top;
        }
        
        // Calculate smart path segments for preview (use block IDs if available)
        let segments;
        const fromBlockId = fromEl ? parseInt(fromEl.dataset.blockId) : null;
        const toBlockId = toEl ? parseInt(toEl.dataset.blockId) : null;
        
        if (fromBlockId && toBlockId) {
            // Use smart pathfinding for preview
            segments = this.calculateSmartPath(fromX, fromY, toX, toY, WorkflowManager.gridSize, fromBlockId, toBlockId);
        } else {
            // Fallback to simple path for preview
            segments = this.calculateRightAnglePathFallback(fromX, fromY, toX, toY, WorkflowManager.gridSize);
        }
        
        // Create container for preview connection segments
        const previewContainer = document.createElement('div');
        previewContainer.className = 'block-connection-container preview';
        previewContainer.style.position = 'absolute';
        previewContainer.style.left = '0';
        previewContainer.style.top = '0';
        previewContainer.style.width = '100%';
        previewContainer.style.height = '100%';
        previewContainer.style.pointerEvents = 'none';
        previewContainer.style.zIndex = '100';
        
        // Create segments for the preview path
        segments.forEach((segment) => {
            const segmentEl = document.createElement('div');
            segmentEl.className = 'block-connection-line preview';
            segmentEl.style.position = 'absolute';
            segmentEl.style.left = segment.x + 'px';
            segmentEl.style.top = segment.y + 'px';
            segmentEl.style.width = segment.width + 'px';
            segmentEl.style.height = segment.height + 'px';
            segmentEl.style.backgroundColor = '#10b981'; // Green for preview
            segmentEl.style.boxShadow = '0 0 6px rgba(16, 185, 129, 0.8)';
            segmentEl.style.transition = 'all 0.2s ease';
            segmentEl.style.pointerEvents = 'none';
            
            previewContainer.appendChild(segmentEl);
        });
        
        canvas.appendChild(previewContainer);
        this.previewLine = previewContainer;
    },
    
    /**
     * Clear preview connection line
     */
    clearPreviewConnection() {
        if (this.previewLine) {
            if (this.previewLine.parentNode) {
                this.previewLine.remove();
            }
            // If it's a container, remove all child segments
            if (this.previewLine.querySelectorAll) {
                const segments = this.previewLine.querySelectorAll('.block-connection-line');
                segments.forEach(seg => seg.remove());
            }
        }
        this.previewLine = null;
    },
    
    /**
     * Get cached block element or query and cache it
     * @param {number} blockId - Block ID
     * @returns {HTMLElement|null} - Block element
     */
    getBlockElement(blockId) {
        if (this.blockElementCache.has(blockId)) {
            const el = this.blockElementCache.get(blockId);
            // Verify element is still in DOM
            if (el && el.isConnected) {
                return el;
            }
            // Element was removed, clear cache
            this.blockElementCache.delete(blockId);
            this.blockPositionCache.delete(blockId);
        }
        
        const el = document.querySelector(`[data-block-id="${blockId}"]`);
        if (el) {
            this.blockElementCache.set(blockId, el);
        }
        return el;
    },
    
    /**
     * Clear block cache for a specific block or all blocks
     * @param {number|null} blockId - Block ID or null to clear all
     */
    clearBlockCache(blockId = null) {
        if (blockId !== null) {
            this.blockElementCache.delete(blockId);
            this.blockPositionCache.delete(blockId);
        } else {
            this.blockElementCache.clear();
            this.blockPositionCache.clear();
        }
    },
    
    /**
     * Get block position in canvas coordinates (with caching for performance)
     * @param {HTMLElement} blockEl - Block element
     * @param {number|null} blockId - Block ID for caching (optional)
     * @returns {Object|null} - Position data
     */
    getBlockPosition(blockEl, blockId = null) {
        if (!blockEl) return null;
        
        // Try to get from cache if blockId provided
        if (blockId !== null && this.blockPositionCache.has(blockId)) {
            return this.blockPositionCache.get(blockId);
        }
        
        const canvas = blockEl.closest('.workspace-canvas');
        if (!canvas) return null;
        
        const rect = blockEl.getBoundingClientRect();
        const canvasRect = canvas.getBoundingClientRect();
        
        const left = parseFloat(blockEl.style.left) || 0;
        const top = parseFloat(blockEl.style.top) || 0;
        const width = rect.width;
        const height = rect.height;
        
        const position = {
            left: left,
            top: top,
            width: width,
            height: height,
            centerX: left + width / 2,
            centerY: top + height / 2,
            bottom: top + height,
            right: left + width
        };
        
        // Cache the position if blockId provided
        if (blockId !== null) {
            this.blockPositionCache.set(blockId, position);
        }
        
        return position;
    },
    
    /**
     * Snap blocks together
     */
    snapBlocks(blockEl, blockData) {
        if (!blockEl || !blockData) return;
        
        // Event blocks can't be snapped to other blocks (they are root blocks)
        if (blockData.type === 'event') return;
        
        const blockPos = this.getBlockPosition(blockEl);
        if (!blockPos) return;
        
        let bestSnap = null;
        let minDistance = Infinity;
        let snapDirection = null; // 'above' or 'below'
        
        WorkflowManager.blocks.forEach((otherBlock, otherId) => {
            if (otherId === blockData.id) return;
            
            // Don't snap if already connected
            if (this.isConnected(blockData.id, otherId)) return;
            
            const otherEl = document.querySelector(`[data-block-id="${otherId}"]`);
            if (!otherEl) return;
            
            const otherPos = this.getBlockPosition(otherEl);
            if (!otherPos) return;
            
            const dx = Math.abs(blockPos.centerX - otherPos.centerX);
            
            // Check if this block should connect ABOVE the other block
            // (this block's bottom is near other block's top)
            // Note: Event blocks can't have blocks above them, so skip this case for event blocks
            if (otherBlock.type !== 'event') {
                const dyAbove = otherPos.top - blockPos.bottom;
                if (dx < this.snapDistance && dyAbove > 0 && dyAbove < this.snapDistance) {
                    const distance = Math.sqrt(dx * dx + dyAbove * dyAbove);
                    if (distance < minDistance) {
                        minDistance = distance;
                        bestSnap = {
                            block: otherBlock,
                            element: otherEl,
                            x: otherPos.left + (otherPos.width / 2) - (blockPos.width / 2),
                            y: otherPos.top - blockPos.height - 5 // 5px gap between blocks
                        };
                        snapDirection = 'above';
                    }
                }
            }
            
            // Check if this block should connect BELOW the other block
            // (this block's top is near other block's bottom)
            // This works for both regular blocks and event blocks (event blocks can have blocks below them)
            const dyBelow = blockPos.top - otherPos.bottom;
            if (dx < this.snapDistance && dyBelow > 0 && dyBelow < this.snapDistance) {
                const distance = Math.sqrt(dx * dx + dyBelow * dyBelow);
                if (distance < minDistance) {
                    minDistance = distance;
                    bestSnap = {
                        block: otherBlock,
                        element: otherEl,
                        x: otherPos.left + (otherPos.width / 2) - (blockPos.width / 2),
                        y: otherPos.bottom + 5 // 5px gap between blocks
                    };
                    snapDirection = 'below';
                }
            }
        });
        
        if (bestSnap && snapDirection) {
            // Remove snapping class from all blocks before snapping
            document.querySelectorAll('.scratch-block').forEach(el => {
                el.classList.remove('snapping');
            });
            
            // Snap the block
            blockEl.style.left = bestSnap.x + 'px';
            blockEl.style.top = bestSnap.y + 'px';
            blockData.x = bestSnap.x;
            blockData.y = bestSnap.y;
            
            // Connect the blocks based on direction
            if (snapDirection === 'above') {
                // This block connects to the other block below it
                this.connectBlocks(blockData.id, bestSnap.block.id);
            } else if (snapDirection === 'below') {
                // The other block connects to this block below it
                this.connectBlocks(bestSnap.block.id, blockData.id);
            }
        } else {
            // No snap found - remove snapping class from all blocks
            document.querySelectorAll('.scratch-block').forEach(el => {
                el.classList.remove('snapping');
            });
        }
    },
    
    /**
     * Connect two blocks (supports multiple parallel connections)
     */
    connectBlocks(fromBlockId, toBlockId) {
        if (!fromBlockId || !toBlockId || fromBlockId === toBlockId) return;
        
        // Initialize connections if needed
        if (!this.connections.has(fromBlockId)) {
            this.connections.set(fromBlockId, { prev: null, next: [] });
        }
        if (!this.connections.has(toBlockId)) {
            this.connections.set(toBlockId, { prev: null, next: [] });
        }
        
        const fromConn = this.connections.get(fromBlockId);
        const toConn = this.connections.get(toBlockId);
        
        // Normalize next to array (for backward compatibility)
        if (!Array.isArray(fromConn.next)) {
            if (fromConn.next) {
                fromConn.next = [fromConn.next];
            } else {
                fromConn.next = [];
            }
        }
        
        // If toBlock already has a previous connection, remove fromBlock from it if it exists
        if (toConn.prev && toConn.prev !== fromBlockId) {
            // If toBlock was connected to another block, remove that connection
            const oldFromConn = this.connections.get(toConn.prev);
            if (oldFromConn) {
                if (Array.isArray(oldFromConn.next)) {
                    const index = oldFromConn.next.indexOf(toBlockId);
                    if (index !== -1) {
                        oldFromConn.next.splice(index, 1);
                    }
                } else if (oldFromConn.next === toBlockId) {
                    oldFromConn.next = [];
                }
                this.updateConnectorVisuals(toConn.prev);
            }
        }
        
        // Add toBlock to fromBlock's next connections (allow multiple)
        if (!fromConn.next.includes(toBlockId)) {
            fromConn.next.push(toBlockId);
        }
        
        // Set toBlock's prev to fromBlock
        toConn.prev = fromBlockId;
        
        // Update block data
        const fromBlock = WorkflowManager.blocks.get(fromBlockId);
        const toBlock = WorkflowManager.blocks.get(toBlockId);
        
        if (fromBlock) {
            if (!fromBlock.connections) fromBlock.connections = {};
            // Store as array for parallel connections
            if (!Array.isArray(fromBlock.connections.next)) {
                fromBlock.connections.next = fromBlock.connections.next ? [fromBlock.connections.next] : [];
            }
            if (!fromBlock.connections.next.includes(toBlockId)) {
                fromBlock.connections.next.push(toBlockId);
            }
        }
        
        if (toBlock) {
            if (!toBlock.connections) toBlock.connections = {};
            toBlock.connections.prev = fromBlockId;
        }
        
        // Ensure blocks are in the same workflow
        if (fromBlock && toBlock) {
            // If fromBlock has a workflow, add toBlock to it
            if (fromBlock.workflowId && !toBlock.workflowId) {
                toBlock.workflowId = fromBlock.workflowId;
                const workflow = WorkflowManager.workflows.get(fromBlock.workflowId);
                if (workflow) {
                    workflow.blocks.add(toBlock.id);
                    // Add all blocks in toBlock's chain to the workflow
                    const chain = this.getBlockChain(toBlock.id);
                    chain.forEach(bid => {
                        workflow.blocks.add(bid);
                        const b = WorkflowManager.blocks.get(bid);
                        if (b) {
                            b.workflowId = workflow.id;
                        }
                    });
                }
            } else if (toBlock.workflowId && !fromBlock.workflowId) {
                // If toBlock has a workflow, add fromBlock to it
                fromBlock.workflowId = toBlock.workflowId;
                const workflow = WorkflowManager.workflows.get(toBlock.workflowId);
                if (workflow) {
                    workflow.blocks.add(fromBlock.id);
                    // Add all blocks in fromBlock's chain to the workflow
                    const chain = this.getBlockChain(fromBlock.id);
                    chain.forEach(bid => {
                        workflow.blocks.add(bid);
                        const b = WorkflowManager.blocks.get(bid);
                        if (b) {
                            b.workflowId = workflow.id;
                        }
                    });
                }
            } else if (fromBlock.workflowId && toBlock.workflowId && fromBlock.workflowId !== toBlock.workflowId) {
                // Both have different workflows - merge them by adding toBlock's chain to fromBlock's workflow
                const fromWorkflow = WorkflowManager.workflows.get(fromBlock.workflowId);
                const toWorkflow = WorkflowManager.workflows.get(toBlock.workflowId);
                if (fromWorkflow && toWorkflow) {
                    // Move all blocks from toWorkflow to fromWorkflow
                    toWorkflow.blocks.forEach(bid => {
                        fromWorkflow.blocks.add(bid);
                        const b = WorkflowManager.blocks.get(bid);
                        if (b) {
                            b.workflowId = fromBlock.workflowId;
                        }
                    });
                    // Delete the toWorkflow
                    WorkflowManager.workflows.delete(toBlock.workflowId);
                }
            }
        }
        
        // Update connector visuals
        this.updateConnectorVisuals(fromBlockId);
        this.updateConnectorVisuals(toBlockId);
        
        // Update connection lines
        this.updateConnections();
        
        StorageManager.autoSave();
        UIUtils.log(`[CONNECTOR] Connected block ${fromBlockId} -> ${toBlockId}`);
    },
    
    /**
     * Disconnect blocks
     */
    disconnectBlocks(fromBlockId, toBlockId) {
        if (!this.connections.has(fromBlockId) || !this.connections.has(toBlockId)) return;
        
        const fromConn = this.connections.get(fromBlockId);
        const toConn = this.connections.get(toBlockId);
        
        // Normalize next to array
        const nextArray = Array.isArray(fromConn.next) ? fromConn.next : (fromConn.next ? [fromConn.next] : []);
        
        if (nextArray.includes(toBlockId) && toConn.prev === fromBlockId) {
            // Remove toBlockId from fromBlock's next array
            const index = nextArray.indexOf(toBlockId);
            if (index !== -1) {
                nextArray.splice(index, 1);
            }
            fromConn.next = nextArray.length > 0 ? nextArray : null;
            toConn.prev = null;
            
            // Remove custom path data
            const key = `${fromBlockId}-${toBlockId}`;
            this.customPaths.delete(key);
            
            // Update block data
            const fromBlock = WorkflowManager.blocks.get(fromBlockId);
            const toBlock = WorkflowManager.blocks.get(toBlockId);
            
            if (fromBlock && fromBlock.connections) {
                if (Array.isArray(fromBlock.connections.next)) {
                    const idx = fromBlock.connections.next.indexOf(toBlockId);
                    if (idx !== -1) {
                        fromBlock.connections.next.splice(idx, 1);
                        if (fromBlock.connections.next.length === 0) {
                            fromBlock.connections.next = null;
                        }
                    }
                } else if (fromBlock.connections.next === toBlockId) {
                    fromBlock.connections.next = null;
                }
            }
            
            if (toBlock && toBlock.connections) {
                toBlock.connections.prev = null;
            }
            
            // Update visuals
            this.updateConnectorVisuals(fromBlockId);
            this.updateConnectorVisuals(toBlockId);
            this.updateConnections();
            
            StorageManager.autoSave();
            UIUtils.log(`[CONNECTOR] Disconnected block ${fromBlockId} -> ${toBlockId}`);
        }
    },
    
    /**
     * Check if two blocks are connected
     */
    isConnected(blockId1, blockId2) {
        const conn1 = this.connections.get(blockId1);
        const conn2 = this.connections.get(blockId2);
        
        if (!conn1 || !conn2) return false;
        
        // Normalize next to array
        const next1 = Array.isArray(conn1.next) ? conn1.next : (conn1.next ? [conn1.next] : []);
        const next2 = Array.isArray(conn2.next) ? conn2.next : (conn2.next ? [conn2.next] : []);
        
        return (next1.includes(blockId2) && conn2.prev === blockId1) ||
               (conn1.prev === blockId2 && next2.includes(blockId1));
    },
    
    /**
     * Update connector visual states
     */
    updateConnectorVisuals(blockId) {
        const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
        if (!blockEl) return;
        
        const conn = this.connections.get(blockId);
        
        // Update top connector (connected if has previous)
        const topConnector = blockEl.querySelector('.block-connector.top');
        if (topConnector) {
            if (conn && conn.prev) {
                topConnector.classList.add('connected');
                topConnector.title = 'Right-click to disconnect';
            } else {
                topConnector.classList.remove('connected');
                topConnector.title = 'Click to connect';
            }
        }
        
        // Update bottom connector (connected if has next)
        const bottomConnector = blockEl.querySelector('.block-connector.bottom');
        if (bottomConnector) {
            const hasNext = conn && conn.next && (Array.isArray(conn.next) ? conn.next.length > 0 : true);
            if (hasNext) {
                bottomConnector.classList.add('connected');
                const nextCount = Array.isArray(conn.next) ? conn.next.length : 1;
                bottomConnector.title = `Connected to ${nextCount} block(s). Right-click to disconnect`;
            } else {
                bottomConnector.classList.remove('connected');
                bottomConnector.title = 'Click to connect';
            }
        }
    },
    
    /**
     * Update all connection lines
     */
    updateConnections() {
        // Remove all existing connection lines and waypoints
        // Use a more efficient method with batch removal
        const canvas = document.getElementById('workspaceCanvas');
        if (!canvas) return;
        
        // Batch DOM operations for better performance
        const elementsToRemove = canvas.querySelectorAll('.block-connection-line, .connection-waypoint, .block-connection-container, .block-trigger-link, .block-trigger-link-arrow');
        elementsToRemove.forEach(el => el.remove());
        this.connectionLines.clear();
        
        // Clear position cache before redrawing (blocks may have moved)
        this.blockPositionCache.clear();
        
        // Draw connections
        this.connections.forEach((conn, blockId) => {
            if (conn.next) {
                // Handle both array and single value for backward compatibility
                const nextArray = Array.isArray(conn.next) ? conn.next : [conn.next];
                nextArray.forEach(nextId => {
                    if (nextId) {
                        this.drawConnection(blockId, nextId, canvas);
                    }
                });
            }
        });
        
        // Draw trigger links (workflow-complete event blocks to their triggering blocks)
        this.drawTriggerLinks(canvas);
    },
    
    /**
     * Draw trigger links from triggering blocks to workflow-complete event blocks
     */
    drawTriggerLinks(canvas) {
        // Remove existing trigger links and arrows
        document.querySelectorAll('.block-trigger-link, .block-trigger-link-arrow').forEach(el => {
            el.remove();
        });
        
        if (!canvas) {
            canvas = document.getElementById('workspaceCanvas');
            if (!canvas) return;
        }
        
        WorkflowManager.blocks.forEach((block, blockId) => {
            if (block.type === 'event' && block.eventType === 'workflow-complete' && block.triggeredBy) {
                const triggeringBlockId = block.triggeredBy;
                const triggeringBlock = WorkflowManager.blocks.get(triggeringBlockId);
                
                if (triggeringBlock) {
                    // Use cached elements for better performance
                    const triggeringEl = this.getBlockElement(triggeringBlockId);
                    const eventEl = this.getBlockElement(blockId);
                    
                    if (triggeringEl && eventEl) {
                        this.drawTriggerLink(triggeringEl, eventEl, canvas, triggeringBlockId, blockId);
                    }
                }
            }
        });
    },
    
    /**
     * Draw a trigger link from a triggering block to a workflow-complete event block
     * @param {HTMLElement} fromEl - Source element
     * @param {HTMLElement} toEl - Target element
     * @param {HTMLElement} canvas - Canvas element
     * @param {number} fromBlockId - Source block ID for caching (optional)
     * @param {number} toBlockId - Target block ID for caching (optional)
     */
    drawTriggerLink(fromEl, toEl, canvas, fromBlockId = null, toBlockId = null) {
        const fromPos = this.getBlockPosition(fromEl, fromBlockId);
        const toPos = this.getBlockPosition(toEl, toBlockId);
        
        if (!fromPos || !toPos) return;
        
        // Draw from right side of triggering block to left side of event block
        const fromX = fromPos.right;
        const fromY = fromPos.centerY;
        const toX = toPos.left;
        const toY = toPos.centerY;
        
        // Calculate length and angle
        const dx = toX - fromX;
        const dy = toY - fromY;
        const length = Math.sqrt(dx * dx + dy * dy);
        const angle = Math.atan2(dy, dx) * 180 / Math.PI;
        
        // Create trigger link element (dashed line)
        const line = document.createElement('div');
        line.className = 'block-trigger-link';
        line.style.position = 'absolute';
        line.style.left = fromX + 'px';
        line.style.top = fromY + 'px';
        line.style.width = length + 'px';
        line.style.height = '3px';
        line.style.zIndex = '4';
        line.style.pointerEvents = 'none';
        line.style.borderTop = '2px dashed #8b5cf6'; // Purple dashed line
        line.style.borderBottom = 'none';
        line.style.transform = `rotate(${angle}deg)`;
        line.style.transformOrigin = '0 50%';
        line.style.opacity = '0.7';
        line.style.transition = 'opacity 0.2s';
        
        canvas.appendChild(line);
        
        // Add arrow head at the end (pointing to the event block)
        // Position arrow just before the target block, pointing in the direction of the line
        const arrowDistance = 10; // Distance from target block
        const arrowAngleRad = angle * Math.PI / 180;
        const arrowX = toX - arrowDistance * Math.cos(arrowAngleRad);
        const arrowY = toY - arrowDistance * Math.sin(arrowAngleRad);
        
        const arrow = document.createElement('div');
        arrow.className = 'block-trigger-link-arrow';
        arrow.style.position = 'absolute';
        arrow.style.left = arrowX + 'px';
        arrow.style.top = arrowY + 'px';
        arrow.style.width = '0';
        arrow.style.height = '0';
        // Arrow points right by default, so we rotate it to point in the direction of the line
        arrow.style.borderLeft = '8px solid #8b5cf6';
        arrow.style.borderTop = '5px solid transparent';
        arrow.style.borderBottom = '5px solid transparent';
        arrow.style.transform = `rotate(${angle}deg)`;
        arrow.style.transformOrigin = '0 50%'; // Rotate around left center point
        arrow.style.zIndex = '5';
        arrow.style.pointerEvents = 'none';
        arrow.style.opacity = '0.7';
        canvas.appendChild(arrow);
    },
    
    /**
     * Toggle auto-wire mode
     */
    setAutoWire(enabled) {
        this.autoWireEnabled = enabled;
        UIUtils.log(`[CONNECTOR] Auto-wire ${enabled ? 'enabled' : 'disabled'}`);
        // Redraw all connections with new mode
        this.updateConnections();
    },
    
    /**
     * Get or create custom path for a connection
     */
    getCustomPath(fromBlockId, toBlockId) {
        const key = `${fromBlockId}-${toBlockId}`;
        if (!this.customPaths.has(key)) {
            this.customPaths.set(key, {
                waypoints: [],
                enabled: false
            });
        }
        return this.customPaths.get(key);
    },
    
    /**
     * Add a waypoint to a connection
     */
    addWaypoint(fromBlockId, toBlockId, x, y) {
        const customPath = this.getCustomPath(fromBlockId, toBlockId);
        customPath.waypoints.push({ x, y });
        customPath.enabled = true;
        this.updateConnections();
        StorageManager.autoSave();
        UIUtils.log(`[CONNECTOR] Added waypoint at (${x}, ${y}) to connection ${fromBlockId}->${toBlockId}`);
    },
    
    /**
     * Remove a waypoint from a connection
     */
    removeWaypoint(fromBlockId, toBlockId, waypointIndex) {
        const customPath = this.getCustomPath(fromBlockId, toBlockId);
        if (waypointIndex >= 0 && waypointIndex < customPath.waypoints.length) {
            customPath.waypoints.splice(waypointIndex, 1);
            if (customPath.waypoints.length === 0) {
                customPath.enabled = false;
            }
            this.updateConnections();
            StorageManager.autoSave();
            UIUtils.log(`[CONNECTOR] Removed waypoint from connection ${fromBlockId}->${toBlockId}`);
        }
    },
    
    /**
     * Update waypoint position
     */
    updateWaypoint(fromBlockId, toBlockId, waypointIndex, x, y) {
        const customPath = this.getCustomPath(fromBlockId, toBlockId);
        if (waypointIndex >= 0 && waypointIndex < customPath.waypoints.length) {
            customPath.waypoints[waypointIndex] = { x, y };
            this.updateConnections();
            StorageManager.autoSave();
        }
    },
    
    /**
     * Calculate path segments using custom waypoints
     */
    calculateCustomPath(fromX, fromY, toX, toY, waypoints) {
        const segments = [];
        let currentX = fromX;
        let currentY = fromY;
        
        // Draw path through each waypoint
        for (let i = 0; i < waypoints.length; i++) {
            const waypoint = waypoints[i];
            
            // Vertical segment to waypoint Y
            if (Math.abs(currentY - waypoint.y) > 1) {
                const minY = Math.min(currentY, waypoint.y);
                const height = Math.abs(waypoint.y - currentY);
                segments.push({
                    x: currentX - 1,
                    y: minY,
                    width: 2,
                    height: height
                });
                currentY = waypoint.y;
            }
            
            // Horizontal segment to waypoint X
            if (Math.abs(currentX - waypoint.x) > 1) {
                const minX = Math.min(currentX, waypoint.x);
                const width = Math.abs(waypoint.x - currentX);
                segments.push({
                    x: minX,
                    y: currentY - 1,
                    width: width,
                    height: 2
                });
                currentX = waypoint.x;
            }
        }
        
        // Final segments to target
        if (Math.abs(currentY - toY) > 1) {
            const minY = Math.min(currentY, toY);
            const height = Math.abs(toY - currentY);
            segments.push({
                x: currentX - 1,
                y: minY,
                width: 2,
                height: height
            });
            currentY = toY;
        }
        
        if (Math.abs(currentX - toX) > 1) {
            const minX = Math.min(currentX, toX);
            const width = Math.abs(toX - currentX);
            segments.push({
                x: minX,
                y: currentY - 1,
                width: width,
                height: 2
            });
        }
        
        return segments;
    },
    
    /**
     * Get all block obstacles (excluding source and destination blocks)
     * @param {number} fromBlockId - Source block ID
     * @param {number} toBlockId - Destination block ID
     * @returns {Array} - Array of obstacle rectangles: [{left, top, right, bottom, width, height}, ...]
     */
    getAllBlockObstacles(fromBlockId, toBlockId) {
        const obstacles = [];
        
        WorkflowManager.blocks.forEach((block, blockId) => {
            // Exclude source and destination blocks
            if (blockId === fromBlockId || blockId === toBlockId) return;
            
            const blockEl = this.getBlockElement(blockId);
            if (!blockEl) return;
            
            const pos = this.getBlockPosition(blockEl, blockId);
            if (!pos) return;
            
            obstacles.push({
                left: pos.left,
                top: pos.top,
                right: pos.right,
                bottom: pos.bottom,
                width: pos.width,
                height: pos.height
            });
        });
        
        return obstacles;
    },
    
    /**
     * Convert a coordinate to grid cell index
     * @param {number} coord - Coordinate value
     * @param {number} gridSize - Grid size in pixels
     * @returns {number} - Grid cell index
     */
    coordToGridCell(coord, gridSize) {
        return Math.floor(coord / gridSize);
    },
    
    /**
     * Get all grid cells occupied by a block rectangle
     * @param {Object} blockRect - Block rectangle {left, top, right, bottom}
     * @param {number} gridSize - Grid size in pixels
     * @returns {Set} - Set of grid cell keys (e.g., "x,y")
     */
    getBlockOccupiedGridCells(blockRect, gridSize) {
        const cells = new Set();
        
        // Get grid cell bounds for the block
        const startCellX = this.coordToGridCell(blockRect.left, gridSize);
        const endCellX = this.coordToGridCell(blockRect.right, gridSize);
        const startCellY = this.coordToGridCell(blockRect.top, gridSize);
        const endCellY = this.coordToGridCell(blockRect.bottom, gridSize);
        
        // Mark all cells within the block as occupied
        for (let x = startCellX; x <= endCellX; x++) {
            for (let y = startCellY; y <= endCellY; y++) {
                cells.add(`${x},${y}`);
            }
        }
        
        return cells;
    },
    
    /**
     * Get all grid cells occupied by all obstacles
     * @param {number} fromBlockId - Source block ID
     * @param {number} toBlockId - Destination block ID
     * @param {number} gridSize - Grid size in pixels
     * @returns {Set} - Set of grid cell keys
     */
    getAllObstacleGridCells(fromBlockId, toBlockId, gridSize) {
        const obstacles = this.getAllBlockObstacles(fromBlockId, toBlockId);
        const occupiedCells = new Set();
        
        obstacles.forEach(blockRect => {
            const cells = this.getBlockOccupiedGridCells(blockRect, gridSize);
            cells.forEach(cell => occupiedCells.add(cell));
        });
        
        return occupiedCells;
    },
    
    /**
     * Check if a point is inside a block rectangle
     * @param {number} x - X coordinate
     * @param {number} y - Y coordinate
     * @param {Object} blockRect - Block rectangle {left, top, right, bottom}
     * @returns {boolean} - True if point is in block
     */
    isPointInBlock(x, y, blockRect) {
        return x >= blockRect.left && x <= blockRect.right &&
               y >= blockRect.top && y <= blockRect.bottom;
    },
    
    /**
     * Check if a path segment intersects a block rectangle
     * @param {Object} segment - Path segment {x, y, width, height}
     * @param {Object} blockRect - Block rectangle {left, top, right, bottom}
     * @returns {boolean} - True if segment intersects block
     */
    doesSegmentIntersectBlock(segment, blockRect) {
        // Check if segment rectangle overlaps with block rectangle
        const segmentLeft = segment.x;
        const segmentRight = segment.x + segment.width;
        const segmentTop = segment.y;
        const segmentBottom = segment.y + segment.height;
        
        return !(segmentRight < blockRect.left || 
                 segmentLeft > blockRect.right ||
                 segmentBottom < blockRect.top ||
                 segmentTop > blockRect.bottom);
    },
    
    /**
     * Check if a path segment passes through any obstacle grid cells
     * @param {Object} segment - Path segment {x, y, width, height}
     * @param {Set} occupiedCells - Set of occupied grid cell keys
     * @param {number} gridSize - Grid size in pixels
     * @returns {boolean} - True if segment passes through obstacles
     */
    doesSegmentPassThroughObstacles(segment, occupiedCells, gridSize) {
        // Get all grid cells the segment passes through
        const segmentCells = new Set();
        
        if (segment.width === 2) {
            // Vertical segment
            const cellX = this.coordToGridCell(segment.x, gridSize);
            const startCellY = this.coordToGridCell(segment.y, gridSize);
            const endCellY = this.coordToGridCell(segment.y + segment.height, gridSize);
            
            for (let y = startCellY; y <= endCellY; y++) {
                segmentCells.add(`${cellX},${y}`);
            }
        } else if (segment.height === 2) {
            // Horizontal segment
            const cellY = this.coordToGridCell(segment.y, gridSize);
            const startCellX = this.coordToGridCell(segment.x, gridSize);
            const endCellX = this.coordToGridCell(segment.x + segment.width, gridSize);
            
            for (let x = startCellX; x <= endCellX; x++) {
                segmentCells.add(`${x},${cellY}`);
            }
        }
        
        // Check if any segment cell is occupied
        for (const cell of segmentCells) {
            if (occupiedCells.has(cell)) {
                return true;
            }
        }
        
        return false;
    },
    
    /**
     * Calculate Manhattan distance between two points
     * @param {number} x1 - First point X
     * @param {number} y1 - First point Y
     * @param {number} x2 - Second point X
     * @param {number} y2 - Second point Y
     * @returns {number} - Manhattan distance
     */
    manhattanDistance(x1, y1, x2, y2) {
        return Math.abs(x2 - x1) + Math.abs(y2 - y1);
    },
    
    /**
     * A* pathfinding algorithm with Manhattan distance heuristic
     * Finds a right-angle path avoiding obstacles
     * @param {number} fromX - Start X coordinate
     * @param {number} fromY - Start Y coordinate
     * @param {number} toX - End X coordinate
     * @param {number} toY - End Y coordinate
     * @param {number} gridSize - Grid size in pixels
     * @param {number} fromBlockId - Source block ID
     * @param {number} toBlockId - Destination block ID
     * @returns {Array|null} - Array of path segments or null if no path found
     */
    calculateSmartPath(fromX, fromY, toX, toY, gridSize, fromBlockId, toBlockId) {
        // Calculate distance between connectors
        const dx = Math.abs(toX - fromX);
        const dy = toY - fromY; // Positive if destination is below source
        const distance = Math.sqrt(dx * dx + dy * dy);
        
        // For blocks very close together (less than 3 grid cells), use simple L-shaped path
        // Exit segments mess up connections at close distances
        const closeDistanceThreshold = gridSize * 3; // 60px
        if (distance < closeDistanceThreshold) {
            return this.calculateRightAnglePathFallback(fromX, fromY, toX, toY, gridSize);
        }
        
        // Get block rectangles for edge case handling
        const fromRect = this.getBlockRect(fromBlockId);
        const toRect = this.getBlockRect(toBlockId);
        
        // Edge case: If destination top connector is above source bottom connector (toY < fromY)
        // and the destination block overlaps with the source block, we need special handling
        // to prevent pathfinding from trying to route up through the source block
        if (toY < fromY && fromRect && toRect) {
            // Check if blocks overlap horizontally
            const horizontalOverlap = !(toRect.right < fromRect.left || toRect.left > fromRect.right);
            
            if (horizontalOverlap) {
                // Destination is above source and they overlap - use simple L-shaped path
                // that goes around the blocks instead of through them
                return this.calculateRightAnglePathFallback(fromX, fromY, toX, toY, gridSize);
            }
        }
        
        // Get all obstacle grid cells (excluding source and destination blocks)
        // Note: We exclude source/destination from obstacles so paths can start/end at connectors,
        // but we validate segments later to ensure they don't pass through block interiors
        const occupiedCells = this.getAllObstacleGridCells(fromBlockId, toBlockId, gridSize);
        
        // Add source and destination blocks as obstacles (but exclude connector cells)
        // This prevents pathfinding from routing through the blocks
        if (fromRect) {
            const fromCells = this.getBlockOccupiedGridCells(fromRect, gridSize);
            // Exclude the connector cell (bottom center of source block)
            const fromConnectorCellX = this.coordToGridCell(fromX, gridSize);
            const fromConnectorCellY = this.coordToGridCell(fromY, gridSize);
            fromCells.forEach(cell => {
                // Don't mark the connector cell as occupied
                if (cell !== `${fromConnectorCellX},${fromConnectorCellY}`) {
                    occupiedCells.add(cell);
                }
            });
        }
        
        if (toRect) {
            const toCells = this.getBlockOccupiedGridCells(toRect, gridSize);
            // Exclude the connector cell (top center of destination block)
            const toConnectorCellX = this.coordToGridCell(toX, gridSize);
            const toConnectorCellY = this.coordToGridCell(toY, gridSize);
            toCells.forEach(cell => {
                // Don't mark the connector cell as occupied
                if (cell !== `${toConnectorCellX},${toConnectorCellY}`) {
                    occupiedCells.add(cell);
                }
            });
        }
        
        // Create automatic waypoints to make lines "come out" of blocks
        // Source connector is at bottom, so add waypoint below it
        // Destination connector is at top, so add waypoint above it
        const exitOffset = gridSize; // One grid cell offset to clearly exit the block
        const sourceExitY = fromY + exitOffset; // Below source (bottom connector)
        const destExitY = toY - exitOffset; // Above destination (top connector)
        
        // Snap exit waypoints to grid centers for cleaner paths
        const sourceExitYSnapped = WorkflowManager.snapToGridCenter(sourceExitY);
        const destExitYSnapped = WorkflowManager.snapToGridCenter(destExitY);
        
        // Check for direct vertical alignment with no obstacles
        if (Math.abs(toX - fromX) < gridSize / 2) {
            // Use exit waypoints for direct path
            const startY = Math.min(sourceExitYSnapped, destExitYSnapped);
            const endY = Math.max(sourceExitYSnapped, destExitYSnapped);
            const cellX = this.coordToGridCell(fromX, gridSize);
            let hasObstacle = false;
            
            // Check if direct path has obstacles (checking between exit waypoints)
            const startCellY = this.coordToGridCell(startY, gridSize);
            const endCellY = this.coordToGridCell(endY, gridSize);
            for (let y = startCellY; y <= endCellY; y++) {
                if (occupiedCells.has(`${cellX},${y}`)) {
                    hasObstacle = true;
                    break;
                }
            }
            
            if (!hasObstacle) {
                // Direct path is clear - create segments with exit waypoints
                const segments = [];
                
                // Segment from source connector to exit waypoint (down)
                // ALWAYS add this segment
                const sourceExitHeight = sourceExitYSnapped - fromY;
                if (sourceExitHeight > 0) {
                    segments.push({
                        x: fromX - 1,
                        y: fromY,
                        width: 2,
                        height: Math.max(sourceExitHeight, 2) // Minimum 2px height
                    });
                }
                
                // Main vertical segment between exit waypoints
                segments.push({
                    x: fromX - 1,
                    y: startY,
                    width: 2,
                    height: endY - startY
                });
                
                // Segment from exit waypoint to destination connector (up)
                // ALWAYS add this segment
                const destExitHeight = toY - destExitYSnapped;
                if (destExitHeight > 0) {
                    segments.push({
                        x: toX - 1,
                        y: destExitYSnapped,
                        width: 2,
                        height: Math.max(destExitHeight, 2) // Minimum 2px height
                    });
                }
                
                return segments;
            }
        }
        
        // Convert exit waypoints to grid coordinates for A* pathfinding
        const startGridX = this.coordToGridCell(fromX, gridSize);
        const startGridY = this.coordToGridCell(sourceExitYSnapped, gridSize);
        const endGridX = this.coordToGridCell(toX, gridSize);
        const endGridY = this.coordToGridCell(destExitYSnapped, gridSize);
        
        // A* algorithm
        const openSet = [];
        const closedSet = new Set();
        const cameFrom = new Map();
        const gScore = new Map();
        const fScore = new Map();
        
        const startKey = `${startGridX},${startGridY}`;
        const endKey = `${endGridX},${endGridY}`;
        
        // Initialize start node
        gScore.set(startKey, 0);
        fScore.set(startKey, this.manhattanDistance(startGridX, startGridY, endGridX, endGridY));
        openSet.push({
            key: startKey,
            x: startGridX,
            y: startGridY,
            f: fScore.get(startKey)
        });
        
        // Sort open set by f-score (priority queue)
        const sortOpenSet = () => {
            openSet.sort((a, b) => a.f - b.f);
        };
        
        // Maximum iterations to prevent infinite loops
        const maxIterations = 10000;
        let iterations = 0;
        
        while (openSet.length > 0 && iterations < maxIterations) {
            iterations++;
            sortOpenSet();
            
            const current = openSet.shift();
            const currentKey = current.key;
            
            if (currentKey === endKey) {
                // Reconstruct path
                const path = [];
                let nodeKey = endKey;
                
                while (nodeKey !== startKey) {
                    const [x, y] = nodeKey.split(',').map(Number);
                    path.unshift({ x, y });
                    nodeKey = cameFrom.get(nodeKey);
                }
                
                // Add start node
                path.unshift({ x: startGridX, y: startGridY });
                
                // Convert grid path to segments, including exit waypoints
                return this.gridPathToSegments(path, fromX, fromY, toX, toY, gridSize, fromBlockId, toBlockId, sourceExitYSnapped, destExitYSnapped);
            }
            
            closedSet.add(currentKey);
            
            // Check neighbors (only horizontal/vertical)
            const neighbors = [
                { x: current.x + 1, y: current.y }, // Right
                { x: current.x - 1, y: current.y }, // Left
                { x: current.x, y: current.y + 1 }, // Down
                { x: current.x, y: current.y - 1 }  // Up
            ];
            
            for (const neighbor of neighbors) {
                const neighborKey = `${neighbor.x},${neighbor.y}`;
                
                // Skip if in closed set
                if (closedSet.has(neighborKey)) continue;
                
                // Skip if occupied by obstacle
                if (occupiedCells.has(neighborKey)) continue;
                
                // Check if the edge between current and neighbor passes through obstacles
                // This prevents paths that go through blocks and immediately backtrack
                if (this.doesEdgePassThroughObstacles(current.x, current.y, neighbor.x, neighbor.y, occupiedCells)) {
                    continue;
                }
                
                // Calculate tentative g-score
                const tentativeG = gScore.get(currentKey) + 1;
                
                // Check if we've seen this neighbor before
                if (!gScore.has(neighborKey) || tentativeG < gScore.get(neighborKey)) {
                    cameFrom.set(neighborKey, currentKey);
                    gScore.set(neighborKey, tentativeG);
                    const h = this.manhattanDistance(neighbor.x, neighbor.y, endGridX, endGridY);
                    const f = tentativeG + h;
                    fScore.set(neighborKey, f);
                    
                    // Add to open set if not already there
                    const existingIndex = openSet.findIndex(n => n.key === neighborKey);
                    if (existingIndex >= 0) {
                        openSet[existingIndex].f = f;
                    } else {
                        openSet.push({
                            key: neighborKey,
                            x: neighbor.x,
                            y: neighbor.y,
                            f: f
                        });
                    }
                }
            }
        }
        
        // A* failed - try BFS as fallback (no heuristic, guaranteed to find path if one exists)
        console.warn(`[PATHFINDER] A* failed, trying BFS fallback from (${fromX}, ${fromY}) to (${toX}, ${toY})`);
        const bfsPath = this.bfsPathfinding(startGridX, startGridY, endGridX, endGridY, occupiedCells);
        
        if (bfsPath && bfsPath.length > 0) {
            // Convert grid path to segments, including exit waypoints
            return this.gridPathToSegments(bfsPath, fromX, fromY, toX, toY, gridSize, fromBlockId, toBlockId, sourceExitYSnapped, destExitYSnapped);
        }
        
        // BFS also failed - fallback to simple L-shaped path
        console.warn(`[PATHFINDER] BFS also failed, using simple fallback`);
        return this.calculateRightAnglePathFallback(fromX, fromY, toX, toY, gridSize);
    },
    
    /**
     * Check if an edge between two grid cells passes through obstacles
     * @param {number} x1 - First cell X
     * @param {number} y1 - First cell Y
     * @param {number} x2 - Second cell X
     * @param {number} y2 - Second cell Y
     * @param {Set} occupiedCells - Set of occupied grid cell keys
     * @returns {boolean} - True if edge passes through obstacles
     */
    doesEdgePassThroughObstacles(x1, y1, x2, y2, occupiedCells) {
        // For horizontal/vertical edges, check all cells the edge passes through
        if (x1 === x2) {
            // Vertical edge
            const startY = Math.min(y1, y2);
            const endY = Math.max(y1, y2);
            for (let y = startY; y <= endY; y++) {
                if (occupiedCells.has(`${x1},${y}`)) {
                    return true;
                }
            }
        } else if (y1 === y2) {
            // Horizontal edge
            const startX = Math.min(x1, x2);
            const endX = Math.max(x1, x2);
            for (let x = startX; x <= endX; x++) {
                if (occupiedCells.has(`${x},${y1}`)) {
                    return true;
                }
            }
        }
        return false;
    },
    
    /**
     * BFS pathfinding algorithm (fallback when A* fails)
     * Guaranteed to find shortest path if one exists, no heuristic
     * @param {number} startX - Start grid X
     * @param {number} startY - Start grid Y
     * @param {number} endX - End grid X
     * @param {number} endY - End grid Y
     * @param {Set} occupiedCells - Set of occupied grid cell keys
     * @returns {Array|null} - Array of {x, y} grid coordinates or null if no path
     */
    bfsPathfinding(startX, startY, endX, endY, occupiedCells) {
        const queue = [{ x: startX, y: startY, path: [{ x: startX, y: startY }] }];
        const visited = new Set();
        const startKey = `${startX},${startY}`;
        const endKey = `${endX},${endY}`;
        visited.add(startKey);
        
        const maxIterations = 10000;
        let iterations = 0;
        
        while (queue.length > 0 && iterations < maxIterations) {
            iterations++;
            const current = queue.shift();
            const currentKey = `${current.x},${current.y}`;
            
            if (currentKey === endKey) {
                return current.path;
            }
            
            // Check neighbors (only horizontal/vertical)
            const neighbors = [
                { x: current.x + 1, y: current.y },
                { x: current.x - 1, y: current.y },
                { x: current.x, y: current.y + 1 },
                { x: current.x, y: current.y - 1 }
            ];
            
            for (const neighbor of neighbors) {
                const neighborKey = `${neighbor.x},${neighbor.y}`;
                
                // Skip if visited
                if (visited.has(neighborKey)) continue;
                
                // Skip if occupied by obstacle
                if (occupiedCells.has(neighborKey)) continue;
                
                // Check if edge passes through obstacles
                if (this.doesEdgePassThroughObstacles(current.x, current.y, neighbor.x, neighbor.y, occupiedCells)) {
                    continue;
                }
                
                visited.add(neighborKey);
                queue.push({
                    x: neighbor.x,
                    y: neighbor.y,
                    path: [...current.path, { x: neighbor.x, y: neighbor.y }]
                });
            }
        }
        
        return null;
    },
    
    /**
     * Get block rectangle for validation
     * @param {number} blockId - Block ID
     * @returns {Object|null} - Block rectangle or null
     */
    getBlockRect(blockId) {
        const blockEl = this.getBlockElement(blockId);
        if (!blockEl) return null;
        const pos = this.getBlockPosition(blockEl, blockId);
        if (!pos) return null;
        return {
            left: pos.left,
            top: pos.top,
            right: pos.right,
            bottom: pos.bottom
        };
    },
    
    /**
     * Check if a segment passes through source or destination blocks
     * @param {Object} segment - Path segment
     * @param {number} fromBlockId - Source block ID
     * @param {number} toBlockId - Destination block ID
     * @returns {boolean} - True if segment passes through source/destination blocks
     */
    doesSegmentPassThroughSourceOrDest(segment, fromBlockId, toBlockId) {
        const fromRect = this.getBlockRect(fromBlockId);
        const toRect = this.getBlockRect(toBlockId);
        
        if (fromRect && this.doesSegmentIntersectBlock(segment, fromRect)) {
            return true;
        }
        if (toRect && this.doesSegmentIntersectBlock(segment, toRect)) {
            return true;
        }
        return false;
    },
    
    /**
     * Convert grid path to segment format
     * @param {Array} gridPath - Array of {x, y} grid coordinates
     * @param {number} fromX - Actual start X (connector position)
     * @param {number} fromY - Actual start Y (connector position)
     * @param {number} toX - Actual end X (connector position)
     * @param {number} toY - Actual end Y (connector position)
     * @param {number} gridSize - Grid size in pixels
     * @param {number} fromBlockId - Source block ID (for validation)
     * @param {number} toBlockId - Destination block ID (for validation)
     * @param {number} sourceExitY - Exit waypoint Y below source connector
     * @param {number} destExitY - Exit waypoint Y above destination connector
     * @returns {Array} - Array of segments
     */
    gridPathToSegments(gridPath, fromX, fromY, toX, toY, gridSize, fromBlockId, toBlockId, sourceExitY, destExitY) {
        if (gridPath.length < 2) {
            return [];
        }
        
        const segments = [];
        
        // Convert grid coordinates to pixel coordinates (grid cell centers)
        const gridToPixel = (gridCoord) => {
            return gridCoord * gridSize + gridSize / 2;
        };
        
        // First segment: from source connector (bottom) down to exit waypoint
        // ALWAYS add this segment - it goes from connector (at block edge) to exit waypoint (outside block)
        // so it should never pass through the block interior
        // Ensure minimum height to make it visible
        const sourceExitHeight = sourceExitY - fromY;
        if (sourceExitHeight > 0) {
            segments.push({
                x: fromX - 1,
                y: fromY,
                width: 2,
                height: Math.max(sourceExitHeight, 2) // Minimum 2px height
            });
        }
        
        // Start pathfinding from exit waypoint
        let currentX = fromX;
        let currentY = sourceExitY;
        
        for (let i = 1; i < gridPath.length; i++) {
            const prev = gridPath[i - 1];
            const curr = gridPath[i];
            
            const nextX = gridToPixel(curr.x);
            const nextY = gridToPixel(curr.y);
            
            // Determine direction and create segment
            if (curr.x !== prev.x) {
                // Horizontal movement
                const minX = Math.min(currentX, nextX);
                const maxX = Math.max(currentX, nextX);
                const segment = {
                    x: minX,
                    y: currentY - 1,
                    width: maxX - minX,
                    height: 2
                };
                // Only add segment if it doesn't pass through source/destination blocks
                if (!this.doesSegmentPassThroughSourceOrDest(segment, fromBlockId, toBlockId)) {
                    segments.push(segment);
                }
                currentX = nextX;
            } else if (curr.y !== prev.y) {
                // Vertical movement
                const minY = Math.min(currentY, nextY);
                const maxY = Math.max(currentY, nextY);
                const segment = {
                    x: currentX - 1,
                    y: minY,
                    width: 2,
                    height: maxY - minY
                };
                // Only add segment if it doesn't pass through source/destination blocks
                if (!this.doesSegmentPassThroughSourceOrDest(segment, fromBlockId, toBlockId)) {
                    segments.push(segment);
                }
                currentY = nextY;
            }
        }
        
        // Final segments: from pathfinding end to exit waypoint, then to destination connector
        // First, connect to destination exit waypoint
        if (Math.abs(currentX - toX) > 1) {
            const minX = Math.min(currentX, toX);
            const maxX = Math.max(currentX, toX);
            const segment = {
                x: minX,
                y: currentY - 1,
                width: maxX - minX,
                height: 2
            };
            // Only add segment if it doesn't pass through source/destination blocks
            if (!this.doesSegmentPassThroughSourceOrDest(segment, fromBlockId, toBlockId)) {
                segments.push(segment);
            }
            currentX = toX;
        }
        
        if (Math.abs(currentY - destExitY) > 1) {
            const minY = Math.min(currentY, destExitY);
            const maxY = Math.max(currentY, destExitY);
            const segment = {
                x: currentX - 1,
                y: minY,
                width: 2,
                height: maxY - minY
            };
            // Only add segment if it doesn't pass through source/destination blocks
            if (!this.doesSegmentPassThroughSourceOrDest(segment, fromBlockId, toBlockId)) {
                segments.push(segment);
            }
            currentY = destExitY;
        }
        
        // Final segment: from exit waypoint (above destination) up to destination connector (top)
        // ALWAYS add this segment - it goes from exit waypoint (outside block) to connector (at block edge)
        // so it should never pass through the block interior
        // Ensure minimum height to make it visible
        const destExitHeight = toY - destExitY;
        if (destExitHeight > 0) {
            segments.push({
                x: toX - 1,
                y: destExitY,
                width: 2,
                height: Math.max(destExitHeight, 2) // Minimum 2px height
            });
        }
        
        return segments;
    },
    
    /**
     * Fallback to simple L-shaped path when A* fails or blocks are close
     * @param {number} fromX - Start X
     * @param {number} fromY - Start Y
     * @param {number} toX - End X
     * @param {number} toY - End Y
     * @param {number} gridSize - Grid size
     * @param {boolean} useExitSegments - Whether to use exit segments (default: false for close blocks)
     * @returns {Array} - Array of segments
     */
    calculateRightAnglePathFallback(fromX, fromY, toX, toY, gridSize = 20, useExitSegments = false) {
        const segments = [];
        
        const dx = toX - fromX;
        const dy = toY - fromY;
        
        // Calculate distance to determine if we should use exit segments
        const distance = Math.sqrt(dx * dx + dy * dy);
        const closeDistanceThreshold = gridSize * 3;
        const shouldUseExitSegments = useExitSegments && distance >= closeDistanceThreshold;
        
        let sourceExitY, destExitY;
        if (shouldUseExitSegments) {
            // Create automatic exit waypoints
            const exitOffset = gridSize;
            sourceExitY = WorkflowManager.snapToGridCenter(fromY + exitOffset);
            destExitY = WorkflowManager.snapToGridCenter(toY - exitOffset);
        } else {
            // For close blocks, use connector positions directly
            sourceExitY = fromY;
            destExitY = toY;
        }
        
        // If blocks are vertically aligned (same X), go straight
        if (Math.abs(dx) < gridSize / 2) {
            if (shouldUseExitSegments) {
                // Segment from source connector to exit waypoint (down)
                const sourceExitHeight = sourceExitY - fromY;
                if (sourceExitHeight > 0) {
                    segments.push({
                        x: fromX - 1,
                        y: fromY,
                        width: 2,
                        height: Math.max(sourceExitHeight, 2)
                    });
                }
                
                // Main vertical segment between exit waypoints
                const startY = Math.min(sourceExitY, destExitY);
                const endY = Math.max(sourceExitY, destExitY);
                segments.push({
                    x: fromX - 1,
                    y: startY,
                    width: 2,
                    height: endY - startY
                });
                
                // Segment from exit waypoint to destination connector (up)
                const destExitHeight = toY - destExitY;
                if (destExitHeight > 0) {
                    segments.push({
                        x: toX - 1,
                        y: destExitY,
                        width: 2,
                        height: Math.max(destExitHeight, 2)
                    });
                }
            } else {
                // Simple straight line for close blocks
                const startY = Math.min(fromY, toY);
                const endY = Math.max(fromY, toY);
                segments.push({
                    x: fromX - 1,
                    y: startY,
                    width: 2,
                    height: endY - startY
                });
            }
            
            return segments;
        }
        
        // Calculate a good intermediate Y position for the horizontal turn
        const midY = shouldUseExitSegments 
            ? WorkflowManager.snapToGridCenter((sourceExitY + destExitY) / 2)
            : WorkflowManager.snapToGridCenter((fromY + toY) / 2);
        const minVerticalSpace = gridSize * 2;
        let turnY = midY;
        
        const exitDy = destExitY - sourceExitY;
        if (Math.abs(exitDy) < minVerticalSpace) {
            if (exitDy > 0) {
                turnY = WorkflowManager.snapToGridCenter(sourceExitY + minVerticalSpace / 2);
            } else {
                turnY = WorkflowManager.snapToGridCenter(sourceExitY - minVerticalSpace / 2);
            }
        }
        
        // Create L-shaped path
        if (shouldUseExitSegments) {
            // First segment: from source connector to exit waypoint (down)
            const sourceExitHeight = sourceExitY - fromY;
            if (sourceExitHeight > 0) {
                segments.push({
                    x: fromX - 1,
                    y: fromY,
                    width: 2,
                    height: Math.max(sourceExitHeight, 2) // Minimum 2px height
                });
            }
        }
        
        // Pathfinding segments from source exit to destination exit
        if (exitDy >= 0) {
            // Going down from source exit
            const firstSegmentHeight = turnY - sourceExitY;
            if (firstSegmentHeight > 0) {
                segments.push({
                    x: fromX - 1,
                    y: sourceExitY,
                    width: 2,
                    height: firstSegmentHeight
                });
            }
            
            const horizontalStartX = Math.min(fromX, toX);
            const horizontalEndX = Math.max(fromX, toX);
            const horizontalWidth = horizontalEndX - horizontalStartX;
            segments.push({
                x: horizontalStartX,
                y: turnY - 1,
                width: horizontalWidth,
                height: 2
            });
            
            const finalSegmentHeight = destExitY - turnY;
            if (finalSegmentHeight > 0) {
                segments.push({
                    x: toX - 1,
                    y: turnY,
                    width: 2,
                    height: finalSegmentHeight
                });
            } else if (finalSegmentHeight < 0) {
                segments.push({
                    x: toX - 1,
                    y: destExitY,
                    width: 2,
                    height: Math.abs(finalSegmentHeight)
                });
            }
        } else {
            // Going up from source exit (destination is above source)
            // For close blocks without exit segments, we need to handle this differently
            if (shouldUseExitSegments) {
                const firstSegmentHeight = sourceExitY - turnY;
                if (firstSegmentHeight > 0) {
                    segments.push({
                        x: fromX - 1,
                        y: turnY,
                        width: 2,
                        height: firstSegmentHeight
                    });
                }
            } else {
                // For close blocks going up, go down first, then horizontal, then up
                const firstSegmentHeight = turnY - fromY;
                if (firstSegmentHeight > 0) {
                    segments.push({
                        x: fromX - 1,
                        y: fromY,
                        width: 2,
                        height: firstSegmentHeight
                    });
                }
            }
            
            const horizontalStartX = Math.min(fromX, toX);
            const horizontalEndX = Math.max(fromX, toX);
            const horizontalWidth = horizontalEndX - horizontalStartX;
            segments.push({
                x: horizontalStartX,
                y: turnY - 1,
                width: horizontalWidth,
                height: 2
            });
            
            if (shouldUseExitSegments) {
                const finalSegmentHeight = turnY - destExitY;
                if (finalSegmentHeight > 0) {
                    segments.push({
                        x: toX - 1,
                        y: destExitY,
                        width: 2,
                        height: finalSegmentHeight
                    });
                } else if (finalSegmentHeight < 0) {
                    segments.push({
                        x: toX - 1,
                        y: turnY,
                        width: 2,
                        height: Math.abs(finalSegmentHeight)
                    });
                }
            } else {
                // For close blocks, final segment goes to destination connector
                const finalSegmentHeight = toY - turnY;
                if (finalSegmentHeight > 0) {
                    segments.push({
                        x: toX - 1,
                        y: turnY,
                        width: 2,
                        height: finalSegmentHeight
                    });
                } else if (finalSegmentHeight < 0) {
                    segments.push({
                        x: toX - 1,
                        y: toY,
                        width: 2,
                        height: Math.abs(finalSegmentHeight)
                    });
                }
            }
        }
        
        // Final segment: from destination exit waypoint to destination connector (up)
        if (shouldUseExitSegments) {
            const destExitHeight = toY - destExitY;
            if (destExitHeight > 0) {
                segments.push({
                    x: toX - 1,
                    y: destExitY,
                    width: 2,
                    height: Math.max(destExitHeight, 2) // Minimum 2px height
                });
            }
        }
        
        return segments;
    },
    
    /**
     * Calculate a good-looking right-angle path between two points
     * Returns an array of segments: [{x, y, width, height}, ...]
     * @deprecated Use calculateSmartPath instead
     */
    calculateRightAnglePath(fromX, fromY, toX, toY, gridSize = 20) {
        const segments = [];
        
        // Don't snap connector positions - they're already at block centers which are grid-aligned
        // Only snap waypoints/turn points to grid cell centers
        const dx = toX - fromX;
        const dy = toY - fromY;
        
        // If blocks are vertically aligned (same X), go straight
        if (Math.abs(dx) < gridSize / 2) {
            const startY = Math.min(fromY, toY);
            const height = Math.abs(dy);
            segments.push({
                x: fromX - 1, // Center the 2px line
                y: startY,
                width: 2,
                height: height
            });
            return segments;
        }
        
        // Calculate a good intermediate Y position for the horizontal turn
        // Use the midpoint, snapped to grid cell center for cleaner look
        const midY = WorkflowManager.snapToGridCenter((fromY + toY) / 2);
        
        // Ensure we have enough vertical space for the turn (at least 2 grid cells)
        const minVerticalSpace = gridSize * 2; // 40px = 2 grid cells
        let turnY = midY;
        
        // If blocks are too close vertically, adjust the turn point
        if (Math.abs(dy) < minVerticalSpace) {
            // If going down, place turn below source
            if (dy > 0) {
                turnY = WorkflowManager.snapToGridCenter(fromY + minVerticalSpace / 2);
            } else {
                // If going up, place turn above source
                turnY = WorkflowManager.snapToGridCenter(fromY - minVerticalSpace / 2);
            }
        }
        
        // Create L-shaped path: down/up, then right/left, then up/down
        // Connector positions (fromX, fromY, toX, toY) are not snapped - they're already at block centers
        // Only waypoints (turnY) are snapped to grid cell centers
        if (dy >= 0) {
            // Going down from source (most common case)
            
            // First vertical segment: down from source to turn point
            const firstSegmentHeight = turnY - fromY;
            if (firstSegmentHeight > 0) {
                segments.push({
                    x: fromX - 1, // Center the 2px line
                    y: fromY,
                    width: 2,
                    height: firstSegmentHeight
                });
            }
            
            // Horizontal segment: connect horizontally between blocks
            // Use actual connector X positions, not snapped
            const horizontalStartX = Math.min(fromX, toX);
            const horizontalEndX = Math.max(fromX, toX);
            const horizontalWidth = horizontalEndX - horizontalStartX;
            segments.push({
                x: horizontalStartX,
                y: turnY - 1, // Center the 2px line
                width: horizontalWidth,
                height: 2
            });
            
            // Final vertical segment: up/down from turn point to target
            const finalSegmentHeight = toY - turnY;
            if (finalSegmentHeight > 0) {
                segments.push({
                    x: toX - 1, // Center the 2px line
                    y: turnY,
                    width: 2,
                    height: finalSegmentHeight
                });
            } else if (finalSegmentHeight < 0) {
                // Need to go up to target
                segments.push({
                    x: toX - 1, // Center the 2px line
                    y: toY,
                    width: 2,
                    height: Math.abs(finalSegmentHeight)
                });
            }
        } else {
            // Going up from source (less common)
            
            // First vertical segment: up from source to turn point
            const firstSegmentHeight = fromY - turnY;
            if (firstSegmentHeight > 0) {
                segments.push({
                    x: fromX - 1, // Center the 2px line
                    y: turnY,
                    width: 2,
                    height: firstSegmentHeight
                });
            }
            
            // Horizontal segment: connect horizontally between blocks
            // Use actual connector X positions, not snapped
            const horizontalStartX = Math.min(fromX, toX);
            const horizontalEndX = Math.max(fromX, toX);
            const horizontalWidth = horizontalEndX - horizontalStartX;
            segments.push({
                x: horizontalStartX,
                y: turnY - 1, // Center the 2px line
                width: horizontalWidth,
                height: 2
            });
            
            // Final vertical segment: up/down from turn point to target
            const finalSegmentHeight = turnY - toY;
            if (finalSegmentHeight > 0) {
                // Going up to target
                segments.push({
                    x: toX - 1, // Center the 2px line
                    y: toY,
                    width: 2,
                    height: finalSegmentHeight
                });
            } else if (finalSegmentHeight < 0) {
                // Going down to target
                segments.push({
                    x: toX - 1, // Center the 2px line
                    y: turnY,
                    width: 2,
                    height: Math.abs(finalSegmentHeight)
                });
            }
        }
        
        return segments;
    },
    
    /**
     * Draw a connection line between two blocks using right-angle routing
     */
    drawConnection(fromBlockId, toBlockId, canvas) {
        // Use cached elements for better performance
        const fromEl = this.getBlockElement(fromBlockId);
        const toEl = this.getBlockElement(toBlockId);
        
        if (!fromEl || !toEl || !canvas) return;
        
        // Use cached positions with blockId for caching
        const fromPos = this.getBlockPosition(fromEl, fromBlockId);
        const toPos = this.getBlockPosition(toEl, toBlockId);
        
        if (!fromPos || !toPos) return;
        
        // Calculate connection points (bottom of from, top of to)
        const fromX = fromPos.centerX;
        const fromY = fromPos.bottom;
        const toX = toPos.centerX;
        const toY = toPos.top;
        
        // Get custom path if available and not in auto-wire mode
        const customPath = this.getCustomPath(fromBlockId, toBlockId);
        const useCustomPath = !this.autoWireEnabled && customPath.enabled && customPath.waypoints.length > 0;
        
        // Calculate path segments
        let segments;
        if (useCustomPath) {
            segments = this.calculateCustomPath(fromX, fromY, toX, toY, customPath.waypoints);
        } else {
            segments = this.calculateSmartPath(fromX, fromY, toX, toY, WorkflowManager.gridSize, fromBlockId, toBlockId);
        }
        
        // Create container for connection segments (so we can handle hover/click on all segments)
        const connectionContainer = document.createElement('div');
        connectionContainer.className = 'block-connection-container';
        connectionContainer.style.position = 'absolute';
        connectionContainer.style.left = '0';
        connectionContainer.style.top = '0';
        connectionContainer.style.width = '100%';
        connectionContainer.style.height = '100%';
        connectionContainer.style.pointerEvents = 'none';
        connectionContainer.dataset.fromBlockId = fromBlockId;
        connectionContainer.dataset.toBlockId = toBlockId;
        
        // Create segments for the path
        segments.forEach((segment, index) => {
            // Create a wrapper with larger hit area to prevent flickering
            const segmentWrapper = document.createElement('div');
            segmentWrapper.style.position = 'absolute';
            segmentWrapper.style.pointerEvents = 'auto';
            segmentWrapper.style.cursor = 'pointer';
            segmentWrapper.dataset.segmentIndex = index;
            
            // Determine if this is a vertical or horizontal segment and add padding for easier hovering
            const isVertical = segment.width === 2;
            const isHorizontal = segment.height === 2;
            
            if (isVertical) {
                // Vertical segment: add horizontal padding for easier hovering
                segmentWrapper.style.left = (segment.x - 6) + 'px';
                segmentWrapper.style.top = segment.y + 'px';
                segmentWrapper.style.width = '14px'; // 2px line + 6px padding on each side
                segmentWrapper.style.height = segment.height + 'px';
            } else if (isHorizontal) {
                // Horizontal segment: add vertical padding for easier hovering
                segmentWrapper.style.left = segment.x + 'px';
                segmentWrapper.style.top = (segment.y - 6) + 'px';
                segmentWrapper.style.width = segment.width + 'px';
                segmentWrapper.style.height = '14px'; // 2px line + 6px padding on each side
            } else {
                // Non-standard segment
                segmentWrapper.style.left = segment.x + 'px';
                segmentWrapper.style.top = segment.y + 'px';
                segmentWrapper.style.width = segment.width + 'px';
                segmentWrapper.style.height = segment.height + 'px';
            }
            
            // Create the actual visible line element inside the wrapper
            const segmentEl = document.createElement('div');
            segmentEl.className = 'block-connection-line';
            segmentEl.style.position = 'absolute';
            segmentEl.style.backgroundColor = '#3b82f6';
            segmentEl.style.transition = 'all 0.2s ease';
            segmentEl.style.pointerEvents = 'none'; // Let wrapper handle events
            
            // Position the line within the wrapper
            if (isVertical) {
                segmentEl.style.left = '6px'; // Center in the 14px wrapper
                segmentEl.style.top = '0';
                segmentEl.style.width = '2px';
                segmentEl.style.height = segment.height + 'px';
            } else if (isHorizontal) {
                segmentEl.style.left = '0';
                segmentEl.style.top = '6px'; // Center in the 14px wrapper
                segmentEl.style.width = segment.width + 'px';
                segmentEl.style.height = '2px';
            } else {
                segmentEl.style.left = '0';
                segmentEl.style.top = '0';
                segmentEl.style.width = segment.width + 'px';
                segmentEl.style.height = segment.height + 'px';
            }
            
            segmentWrapper.appendChild(segmentEl);
            
            // Add hover effect to the wrapper (not the line itself)
            segmentWrapper.addEventListener('mouseenter', () => {
                segments.forEach((_, segIndex) => {
                    const wrapper = connectionContainer.querySelector(`[data-segment-index="${segIndex}"]`);
                    if (wrapper) {
                        const line = wrapper.querySelector('.block-connection-line');
                        if (line) {
                            line.style.backgroundColor = '#ef4444';
                            line.style.boxShadow = '0 0 8px rgba(239, 68, 68, 0.6)';
                            // Expand the line without moving its position
                            const currentWidth = parseFloat(line.style.width);
                            const currentHeight = parseFloat(line.style.height);
                            if (currentHeight === 2) {
                                // Horizontal line: expand vertically, stay centered
                                line.style.height = '4px';
                                line.style.top = '5px'; // Adjust to keep centered in wrapper
                            } else if (currentWidth === 2) {
                                // Vertical line: expand horizontally, stay centered
                                line.style.width = '4px';
                                line.style.left = '5px'; // Adjust to keep centered in wrapper
                            }
                        }
                    }
                });
            });
            
            segmentWrapper.addEventListener('mouseleave', () => {
                segments.forEach((_, segIndex) => {
                    const wrapper = connectionContainer.querySelector(`[data-segment-index="${segIndex}"]`);
                    if (wrapper) {
                        const line = wrapper.querySelector('.block-connection-line');
                        if (line) {
                            line.style.backgroundColor = '#3b82f6';
                            line.style.boxShadow = 'none';
                            const currentWidth = parseFloat(line.style.width);
                            const currentHeight = parseFloat(line.style.height);
                            if (currentHeight === 4) {
                                // Horizontal line: shrink back
                                line.style.height = '2px';
                                line.style.top = '6px'; // Reset to centered position
                            } else if (currentWidth === 4) {
                                // Vertical line: shrink back
                                line.style.width = '2px';
                                line.style.left = '6px'; // Reset to centered position
                            }
                        }
                    }
                });
            });
            
            // Add click handler - different behavior for auto-wire vs manual mode
            segmentWrapper.addEventListener('click', (e) => {
                e.stopPropagation();
                if (this.autoWireEnabled) {
                    // In auto-wire mode, clicking disconnects
                this.disconnectBlocks(fromBlockId, toBlockId);
                } else {
                    // In manual mode, clicking adds a waypoint
                    const canvas = document.getElementById('workspaceCanvas');
                    const canvasRect = canvas.getBoundingClientRect();
                    const x = e.clientX - canvasRect.left + canvas.scrollLeft;
                    const y = e.clientY - canvasRect.top + canvas.scrollTop;
                    this.addWaypoint(fromBlockId, toBlockId, x, y);
                }
            });
            
            segmentWrapper.addEventListener('contextmenu', (e) => {
                e.preventDefault();
                e.stopPropagation();
                // Right-click always disconnects
                this.disconnectBlocks(fromBlockId, toBlockId);
            });
            
            connectionContainer.appendChild(segmentWrapper);
        });
        
        // Store reference to container for cleanup
        connectionContainer.dataset.connectionKey = `${fromBlockId}-${toBlockId}`;
        
        // Add waypoint markers if in manual mode and waypoints exist
        if (!this.autoWireEnabled && customPath.waypoints.length > 0) {
            customPath.waypoints.forEach((waypoint, index) => {
                const waypointEl = document.createElement('div');
                waypointEl.className = 'connection-waypoint';
                waypointEl.style.left = (waypoint.x - 6) + 'px';
                waypointEl.style.top = (waypoint.y - 6) + 'px';
                waypointEl.dataset.waypointIndex = index;
                waypointEl.dataset.fromBlockId = fromBlockId;
                waypointEl.dataset.toBlockId = toBlockId;
                waypointEl.title = 'Drag to move, right-click to delete';
                
                // Make waypoint draggable
                let isDragging = false;
                let startX, startY;
                
                waypointEl.addEventListener('mousedown', (e) => {
                    e.stopPropagation();
                    if (e.button === 0) { // Left click
                        isDragging = true;
                        startX = e.clientX;
                        startY = e.clientY;
                        waypointEl.classList.add('selected');
                    }
                });
                
                const handleMouseMove = (e) => {
                    if (!isDragging) return;
                    e.preventDefault();
                    
                    const canvas = document.getElementById('workspaceCanvas');
                    const canvasRect = canvas.getBoundingClientRect();
                    const newX = e.clientX - canvasRect.left + canvas.scrollLeft;
                    const newY = e.clientY - canvasRect.top + canvas.scrollTop;
                    
                    // Update waypoint position visually
                    waypointEl.style.left = (newX - 6) + 'px';
                    waypointEl.style.top = (newY - 6) + 'px';
                    
                    // Update the waypoint data and redraw connection
                    this.updateWaypoint(fromBlockId, toBlockId, index, newX, newY);
                };
                
                const handleMouseUp = () => {
                    if (isDragging) {
                        isDragging = false;
                        waypointEl.classList.remove('selected');
                        document.removeEventListener('mousemove', handleMouseMove);
                        document.removeEventListener('mouseup', handleMouseUp);
                    }
                };
                
                waypointEl.addEventListener('mousedown', () => {
                    document.addEventListener('mousemove', handleMouseMove);
                    document.addEventListener('mouseup', handleMouseUp);
                });
                
                // Right-click to delete waypoint
                waypointEl.addEventListener('contextmenu', (e) => {
                    e.preventDefault();
                    e.stopPropagation();
                    this.removeWaypoint(fromBlockId, toBlockId, index);
                });
                
                canvas.appendChild(waypointEl);
            });
        }
        
        canvas.appendChild(connectionContainer);
        this.connectionLines.set(`${fromBlockId}-${toBlockId}`, connectionContainer);
    },
    
    /**
     * Remove connections for a block
     */
    removeBlockConnections(blockId) {
        const conn = this.connections.get(blockId);
        if (!conn) return;
        
        // Disconnect from previous
        if (conn.prev) {
            const prevConn = this.connections.get(conn.prev);
            if (prevConn) {
                prevConn.next = null;
                this.updateConnectorVisuals(conn.prev);
            }
        }
        
        // Disconnect from next (handle array)
        if (conn.next) {
            const nextArray = Array.isArray(conn.next) ? conn.next : [conn.next];
            nextArray.forEach(nextId => {
                if (nextId) {
                    const nextConn = this.connections.get(nextId);
                    if (nextConn) {
                        nextConn.prev = null;
                        this.updateConnectorVisuals(nextId);
                    }
                }
            });
        }
        
        // Remove this block's connections
        this.connections.delete(blockId);
        
        // Clear cache for this block
        this.clearBlockCache(blockId);
        
        // Remove connection lines
        const lineKeys = Array.from(this.connectionLines.keys()).filter(
            key => key.startsWith(blockId + '-') || key.endsWith('-' + blockId)
        );
        lineKeys.forEach(key => {
            const line = this.connectionLines.get(key);
            if (line && line.parentNode) line.remove();
            this.connectionLines.delete(key);
        });
        
        this.updateConnections();
    },
    
    /**
     * Get the chain of blocks starting from a root block (sequential path)
     * For parallel execution, use getNextBlocks instead
     */
    getBlockChain(rootBlockId) {
        const chain = [];
        let currentId = rootBlockId;
        const visited = new Set(); // Prevent infinite loops
        
        while (currentId && !visited.has(currentId)) {
            visited.add(currentId);
            const block = WorkflowManager.blocks.get(currentId);
            if (!block) break;
            
            chain.push(currentId);
            
            const conn = this.connections.get(currentId);
            if (conn && conn.next) {
                // For sequential chain, take the first next block
                const nextArray = Array.isArray(conn.next) ? conn.next : [conn.next];
                if (nextArray.length > 0) {
                    currentId = nextArray[0];
                } else {
                    break;
                }
            } else {
                break;
            }
        }
        
        return chain;
    },
    
    /**
     * Get all next blocks from a given block (for parallel execution)
     */
    getNextBlocks(blockId) {
        const conn = this.connections.get(blockId);
        if (!conn || !conn.next) return [];
        
        // Return array of next block IDs
        return Array.isArray(conn.next) ? conn.next : [conn.next];
    },
    
    /**
     * Initialize connections from saved data
     */
    initializeConnections() {
        // Ensure all connections are initialized from block data
        WorkflowManager.blocks.forEach((block, blockId) => {
            if (block.connections) {
                // Initialize connection entry if needed
                if (!this.connections.has(blockId)) {
                    // Normalize next to array
                    let next = block.connections.next || null;
                    if (next && !Array.isArray(next)) {
                        next = [next];
                    }
                    this.connections.set(blockId, { 
                        prev: block.connections.prev || null, 
                        next: next
                    });
                } else {
                    // Sync with block data
                    const conn = this.connections.get(blockId);
                    if (block.connections.prev) conn.prev = block.connections.prev;
                    if (block.connections.next) {
                        // Normalize to array
                        conn.next = Array.isArray(block.connections.next) ? block.connections.next : [block.connections.next];
                    }
                }
            }
        });
        
        // Update visuals and connection lines
        this.connections.forEach((conn, blockId) => {
            this.updateConnectorVisuals(blockId);
        });
        this.updateConnections();
    },
    
    /**
     * Handle connector click to create connections
     */
    handleConnectorClick(blockId, connectorType) {
        if (!this.connectingFrom) {
            // Start a connection
            const block = WorkflowManager.blocks.get(blockId);
            if (!block) return;
            
            // Can only start from bottom connector (or top for event blocks)
            if (connectorType === 'bottom' || (connectorType === 'top' && block.type === 'event')) {
                this.connectingFrom = { blockId, connectorType };
                
                // Highlight the connector
                const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
                if (blockEl) {
                    const connector = blockEl.querySelector(`.block-connector.${connectorType}`);
                    if (connector) {
                        connector.classList.add('connecting');
                    }
                }
                
                UIUtils.log(`[CONNECTOR] Click another connector to connect to block ${blockId}`);
            }
        } else {
            // Complete a connection
            const fromBlockId = this.connectingFrom.blockId;
            const fromConnectorType = this.connectingFrom.connectorType;
            
            // Clear connecting state
            const fromBlockEl = document.querySelector(`[data-block-id="${fromBlockId}"]`);
            if (fromBlockEl) {
                const fromConnector = fromBlockEl.querySelector(`.block-connector.${fromConnectorType}`);
                if (fromConnector) {
                    fromConnector.classList.remove('connecting');
                }
            }
            
            // Validate connection
            if (fromBlockId === blockId) {
                // Can't connect to self
                this.connectingFrom = null;
                return;
            }
            
            // Connect bottom to top
            if (fromConnectorType === 'bottom' && connectorType === 'top') {
                this.connectBlocks(fromBlockId, blockId);
            } else if (fromConnectorType === 'top' && connectorType === 'bottom') {
                // Connecting from top (event block) to bottom of another
                // This doesn't make sense in our model, but allow it and reverse
                this.connectBlocks(blockId, fromBlockId);
            } else {
                UIUtils.log('[CONNECTOR] Invalid connection: must connect bottom to top', 'warning');
            }
            
            this.connectingFrom = null;
        }
    },
    
    /**
     * Cancel current connection attempt
     */
    cancelConnection() {
        if (this.connectingFrom) {
            const fromBlockEl = document.querySelector(`[data-block-id="${this.connectingFrom.blockId}"]`);
            if (fromBlockEl) {
                const connector = fromBlockEl.querySelector(`.block-connector.${this.connectingFrom.connectorType}`);
                if (connector) {
                    connector.classList.remove('connecting');
                }
            }
            this.connectingFrom = null;
        }
    },
    
    /**
     * Clear all connections
     */
    clear() {
        this.connections.clear();
        this.customPaths.clear();
        this.connectingFrom = null;
        this.selectedWaypoint = null;
        this.editingConnection = null;
        
        // Clear caches
        this.clearBlockCache();
        
        document.querySelectorAll('.block-connection-line, .connection-waypoint, .block-connection-container, .block-trigger-link, .block-trigger-link-arrow').forEach(el => {
            el.remove();
        });
        this.connectionLines.clear();
    },
    
    /**
     * Export custom paths for storage
     */
    exportCustomPaths() {
        const paths = {};
        this.customPaths.forEach((pathData, key) => {
            if (pathData.waypoints.length > 0) {
                paths[key] = {
                    waypoints: pathData.waypoints,
                    enabled: pathData.enabled
                };
            }
        });
        return paths;
    },
    
    /**
     * Import custom paths from storage
     */
    importCustomPaths(paths) {
        if (!paths) return;
        this.customPaths.clear();
        Object.keys(paths).forEach(key => {
            this.customPaths.set(key, {
                waypoints: paths[key].waypoints || [],
                enabled: paths[key].enabled || false
            });
        });
    }
};

