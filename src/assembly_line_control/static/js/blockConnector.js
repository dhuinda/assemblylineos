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
        
        // Calculate right-angle path segments for preview
        const segments = this.calculateRightAnglePath(fromX, fromY, toX, toY, WorkflowManager.gridSize);
        
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
     * Calculate a good-looking right-angle path between two points
     * Returns an array of segments: [{x, y, width, height}, ...]
     */
    calculateRightAnglePath(fromX, fromY, toX, toY, gridSize = 20) {
        const segments = [];
        const dx = toX - fromX;
        const dy = toY - fromY;
        
        // If blocks are vertically aligned (same X), go straight
        if (Math.abs(dx) < 5) {
            const height = Math.abs(dy);
            segments.push({
                x: fromX - 1, // Center the 2px line
                y: Math.min(fromY, toY),
                width: 2,
                height: height
            });
            return segments;
        }
        
        // Calculate a good intermediate Y position for the horizontal turn
        // Use the midpoint, snapped to grid for cleaner look
        const midY = WorkflowManager.snapToGrid((fromY + toY) / 2);
        
        // Ensure we have enough vertical space for the turn
        const minVerticalSpace = 40; // Minimum space needed for a clean turn
        let turnY = midY;
        
        // If blocks are too close vertically, adjust the turn point
        if (Math.abs(dy) < minVerticalSpace) {
            // If going down, place turn below source
            if (dy > 0) {
                turnY = WorkflowManager.snapToGrid(fromY + minVerticalSpace / 2);
            } else {
                // If going up, place turn above source
                turnY = WorkflowManager.snapToGrid(fromY - minVerticalSpace / 2);
            }
        }
        
        // Create L-shaped path: down/up, then right/left, then up/down
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
            const horizontalWidth = Math.abs(dx);
            const horizontalX = Math.min(fromX, toX);
            segments.push({
                x: horizontalX,
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
            const horizontalWidth = Math.abs(dx);
            const horizontalX = Math.min(fromX, toX);
            segments.push({
                x: horizontalX,
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
            segments = this.calculateRightAnglePath(fromX, fromY, toX, toY, WorkflowManager.gridSize);
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
