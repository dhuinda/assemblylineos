/**
 * Workflow Manager - Handles all the blocks and workflows on the canvas
 * 
 * This is like Scratch - you drag blocks around, connect them together,
 * and they run in sequence (or in parallel if multiple blocks connect).
 */
const WorkflowManager = {
    blocks: new Map(), // All the blocks we have
    workflows: new Map(), // All the workflows (groups of blocks)
    blockIdCounter: 0,
    workflowIdCounter: 0,
    canvas: null,
    blockPalette: null,
    selectedBlockId: null,
    draggedBlock: null,
    dragOffset: { x: 0, y: 0 },
    gridSize: 20, // How big the snap-to-grid squares are (in pixels)
    
    // Caches to make things faster
    blockElementCache: new Map(), // Remember which DOM element is which block
    blockPositionCache: new Map(), // Remember where blocks are positioned
    canvasSizeUpdateScheduled: false,
    connectionsUpdateScheduled: false,
    lastDragUpdate: 0,
    dragUpdateThrottle: 16, // Update at about 60 frames per second
    
    /**
     * Set everything up
     */
    init() {
        this.canvas = document.getElementById('workspaceCanvas');
        this.blockPalette = document.getElementById('blockPalette');
        
        if (!this.canvas) {
            UIUtils.log('[WORKFLOW] Canvas not found', 'error');
            return;
        }
        
        this.initializePalette();
        this.initializeCanvas();
        UIUtils.log('[WORKFLOW] Workflow manager initialized');
    },
    
    /**
     * Snap a position to the nearest grid point
     * @param {number} coord - The coordinate to snap
     * @returns {number} - The snapped coordinate
     */
    snapToGrid(coord) {
        return Math.round(coord / this.gridSize) * this.gridSize;
    },
    
    /**
     * Snap a position to the center of the nearest grid cell
     * @param {number} coord - The coordinate to snap
     * @returns {number} - The snapped coordinate (center of grid cell)
     */
    snapToGridCenter(coord) {
        // Snap to grid boundary first, then add half grid size to get center
        const gridBoundary = Math.round(coord / this.gridSize) * this.gridSize;
        return gridBoundary + this.gridSize / 2;
    },
    
    /**
     * Get the DOM element for a block, using cache if possible
     * @param {number} blockId - Which block we want
     * @returns {HTMLElement|null} - The block's element, or null if not found
     */
    getBlockElement(blockId) {
        if (this.blockElementCache.has(blockId)) {
            const el = this.blockElementCache.get(blockId);
            // Make sure it's still actually in the page
            if (el && el.isConnected) {
                return el;
            }
            // It was removed, so forget about it
            this.blockElementCache.delete(blockId);
        }
        
        // Look it up and remember it for next time
        const el = document.querySelector(`[data-block-id="${blockId}"]`);
        if (el) {
            this.blockElementCache.set(blockId, el);
        }
        return el;
    },
    
    /**
     * Clear the cache for a block (or all blocks)
     * @param {number|null} blockId - Which block to clear, or null for all
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
     * Schedule an update of the connection lines (but don't do it too often)
     */
    scheduleConnectionsUpdate() {
        if (this.connectionsUpdateScheduled) return;
        
        this.connectionsUpdateScheduled = true;
        requestAnimationFrame(() => {
            this.connectionsUpdateScheduled = false;
            BlockConnector.updateConnections();
        });
    },
    
    /**
     * Schedule a throttled update of canvas size
     */
    scheduleCanvasSizeUpdate() {
        if (this.canvasSizeUpdateScheduled) return;
        
        this.canvasSizeUpdateScheduled = true;
        requestAnimationFrame(() => {
            this.canvasSizeUpdateScheduled = false;
            this.updateCanvasSize();
        });
    },
    
    /**
     * Initialize block palette with drag handlers
     */
    initializePalette() {
        if (!this.blockPalette) return;
        
        this.blockPalette.querySelectorAll('.block').forEach(blockEl => {
            blockEl.addEventListener('dragstart', (e) => {
                if (e.target.tagName === 'INPUT' || e.target.tagName === 'SELECT') {
                    e.preventDefault();
                    return false;
                }
                
                e.dataTransfer.effectAllowed = 'copy';
                e.dataTransfer.setData('text/plain', JSON.stringify({
                    type: blockEl.dataset.type,
                    motorId: blockEl.dataset.motorId,
                    relayId: blockEl.dataset.relayId,
                    eventType: blockEl.dataset.eventType
                }));
                
                // Create a ghost image
                const clone = blockEl.cloneNode(true);
                clone.style.position = 'absolute';
                clone.style.top = '-1000px';
                document.body.appendChild(clone);
                e.dataTransfer.setDragImage(clone, 0, 0);
                setTimeout(() => document.body.removeChild(clone), 0);
            });
        });
    },
    
    /**
     * Initialize canvas for drop and interactions
     */
    initializeCanvas() {
        // Ensure canvas has minimum dimensions for scrolling
        this.updateCanvasSize();
        
        // Handle drops
        this.canvas.addEventListener('dragover', (e) => {
            e.preventDefault();
            e.dataTransfer.dropEffect = 'copy';
        });
        
        this.canvas.addEventListener('drop', (e) => {
            e.preventDefault();
            const rect = this.canvas.getBoundingClientRect();
            const x = e.clientX - rect.left + this.canvas.scrollLeft;
            const y = e.clientY - rect.top + this.canvas.scrollTop;
            this.handleBlockDrop(e, x, y);
        });
        
        // Handle clicks for selection and cancel connections
        this.canvas.addEventListener('click', (e) => {
            // Don't deselect if we just finished panning
            if (this.wasPanning) {
                this.wasPanning = false;
                e.preventDefault();
                e.stopPropagation();
                return;
            }
            
            if (e.target === this.canvas || e.target.classList.contains('block-connection-line')) {
                this.deselectBlock();
                BlockConnector.cancelConnection(); // Cancel any pending connections
            }
        });
        
        // Handle canvas panning/navigation with multiple methods:
        // 1. Right-click drag (most intuitive)
        // 2. Middle mouse button drag
        // 3. Click and drag on empty canvas (without shift)
        let isPanning = false;
        let panStart = { x: 0, y: 0 };
        let panStartScroll = { left: 0, top: 0 };
        this.wasPanning = false;
        
        this.canvas.addEventListener('mousedown', (e) => {
            // Only start panning if clicking on empty canvas or connection lines
            const isOnCanvas = e.target === this.canvas || 
                              e.target.classList.contains('block-connection-line') ||
                              e.target.classList.contains('block-connection-container');
            
            if (isOnCanvas) {
                // Right mouse button for panning
                if (e.button === 2 || e.which === 3) {
                    isPanning = true;
                    panStart = { x: e.clientX, y: e.clientY };
                    panStartScroll = { left: this.canvas.scrollLeft, top: this.canvas.scrollTop };
                    this.canvas.style.cursor = 'grabbing';
                    e.preventDefault();
                    e.stopPropagation();
                    return;
                }
                
                // Middle mouse button for panning
                if (e.button === 1 || e.which === 2) {
                    isPanning = true;
                    panStart = { x: e.clientX, y: e.clientY };
                    panStartScroll = { left: this.canvas.scrollLeft, top: this.canvas.scrollTop };
                    this.canvas.style.cursor = 'grabbing';
                    e.preventDefault();
                    e.stopPropagation();
                    return;
                }
                
                // Left click on empty canvas (no modifier) - allow panning
                if (e.button === 0 && !e.shiftKey && !e.ctrlKey && !e.metaKey && !e.altKey) {
                    // Check if there's a block at this position
                    const elementBelow = document.elementFromPoint(e.clientX, e.clientY);
                    if (elementBelow === this.canvas || 
                        elementBelow.classList.contains('block-connection-line') ||
                        elementBelow.classList.contains('block-connection-container')) {
                        isPanning = true;
                        panStart = { x: e.clientX, y: e.clientY };
                        panStartScroll = { left: this.canvas.scrollLeft, top: this.canvas.scrollTop };
                        this.canvas.style.cursor = 'grabbing';
                        e.preventDefault();
                        e.stopPropagation();
                        return;
                    }
                }
            }
        });
        
        // Prevent context menu on right-click when panning
        this.canvas.addEventListener('contextmenu', (e) => {
            if (isPanning || e.target === this.canvas) {
                e.preventDefault();
                e.stopPropagation();
                return false;
            }
        });
        
        window.addEventListener('mousemove', (e) => {
            if (isPanning) {
                const dx = e.clientX - panStart.x;
                const dy = e.clientY - panStart.y;
                
                // Update scroll position
                this.canvas.scrollLeft = panStartScroll.left - dx;
                this.canvas.scrollTop = panStartScroll.top - dy;
                
                e.preventDefault();
                e.stopPropagation();
            }
        });
        
        window.addEventListener('mouseup', (e) => {
            if (isPanning) {
                isPanning = false;
                this.canvas.style.cursor = 'default';
                this.wasPanning = true;
                
                // Small delay to prevent click event after panning
                setTimeout(() => {
                    this.wasPanning = false;
                }, 100);
            }
        });
        
        // Ensure wheel events (mouse scroll) work normally
        this.canvas.addEventListener('wheel', (e) => {
            // Don't prevent default - allow natural scrolling
            // The browser's default scroll behavior will work
        }, { passive: true });
    },
    
    /**
     * Update canvas scroll area size to accommodate all blocks
     * This updates the internal scrollable area without changing the panel size
     */
    updateCanvasSize() {
        if (!this.canvas) return;
        
        let maxX = 0;
        let maxY = 0;
        
        // Find the rightmost and bottommost block positions
        // Use cached block data instead of querying DOM
        this.blocks.forEach((block) => {
            // Use block data directly (more efficient than DOM queries)
            const blockRight = block.x + 140; // Default block width
            const blockBottom = block.y + 100; // Default block height
            maxX = Math.max(maxX, blockRight);
            maxY = Math.max(maxY, blockBottom);
        });
        
        // Calculate minimum scrollable content size (add padding for scrollability)
        // Ensure both width and height accommodate all blocks for horizontal and vertical scrolling
        const canvasClientWidth = this.canvas.clientWidth || 800;
        const minContentWidth = Math.max(canvasClientWidth + 100, maxX + 200);
        const minContentHeight = Math.max(600, maxY + 200);
        
        // Create or update a content wrapper div if needed to set scroll dimensions
        // Instead of changing canvas size, we'll ensure the content area is large enough
        // The canvas itself maintains its display size from CSS
        let contentWrapper = this.canvas.querySelector('.canvas-content-wrapper');
        if (!contentWrapper) {
            contentWrapper = document.createElement('div');
            contentWrapper.className = 'canvas-content-wrapper';
            contentWrapper.style.position = 'absolute';
            contentWrapper.style.width = minContentWidth + 'px';
            contentWrapper.style.height = minContentHeight + 'px';
            contentWrapper.style.top = '0';
            contentWrapper.style.left = '0';
            contentWrapper.style.pointerEvents = 'none'; // Don't block interactions
            this.canvas.appendChild(contentWrapper);
        } else {
            // Only update if size changed (avoid unnecessary reflows)
            const currentWidth = parseInt(contentWrapper.style.width);
            const currentHeight = parseInt(contentWrapper.style.height);
            if (currentWidth !== minContentWidth || currentHeight !== minContentHeight) {
                contentWrapper.style.width = minContentWidth + 'px';
                contentWrapper.style.height = minContentHeight + 'px';
            }
        }
    },
    
    /**
     * Handle block drop on canvas
     */
    handleBlockDrop(e, x, y) {
        try {
            const dataStr = e.dataTransfer.getData('text/plain');
            if (!dataStr) return;
            
            const data = JSON.parse(dataStr);
            
            // Get template block from palette
            let templateBlock = null;
            if (data.type === 'event') {
                templateBlock = this.blockPalette.querySelector(
                    `[data-type="event"][data-event-type="${data.eventType}"]`
                );
            } else if (data.type === 'motor') {
                templateBlock = this.blockPalette.querySelector(
                    `[data-type="motor"][data-motor-id="${data.motorId}"]`
                );
            } else if (data.type === 'relay') {
                templateBlock = this.blockPalette.querySelector(
                    `[data-type="relay"][data-relay-id="${data.relayId}"]`
                );
            } else {
                // Handle all other block types (pause, delay, ros-trigger, repeat, forever, break, wait-sensor, read-sensor, try, catch, throw-error)
                templateBlock = this.blockPalette.querySelector(`[data-type="${data.type}"]`);
            }
            
            if (!templateBlock) {
                UIUtils.log(`[WORKFLOW] Template block not found for type: ${data.type}`, 'error');
                return;
            }
            
            // Extract block data
            const blockData = BlockSystem.extractBlockData(templateBlock);
            if (!blockData) return;
            
            // Create block instance
            this.blockIdCounter++;
            const newBlock = BlockSystem.createBlockInstance(blockData, this.blockIdCounter);
            
            if (!newBlock) return;
            
            // Add event type if event block
            if (data.type === 'event' && data.eventType) {
                newBlock.eventType = data.eventType;
            }
            
            // Set position (snapped to grid)
            newBlock.x = this.snapToGrid(x);
            newBlock.y = this.snapToGrid(y);
            
            // Add to blocks map
            this.blocks.set(newBlock.id, newBlock);
            
            // If event block, create a new workflow
            if (newBlock.type === 'event') {
                this.workflowIdCounter++;
                const workflow = {
                    id: this.workflowIdCounter,
                    rootBlockId: newBlock.id,
                    blocks: new Set([newBlock.id])
                };
                this.workflows.set(workflow.id, workflow);
                newBlock.workflowId = workflow.id;
            }
            
            // Render the block
            this.renderBlock(newBlock);
            
            // Initialize connections after rendering
            setTimeout(() => {
                BlockConnector.updateConnections();
                // Update canvas size after adding block
                this.updateCanvasSize();
            }, 10);
            
            StorageManager.autoSave();
            
            UIUtils.log(`[WORKFLOW] Added ${newBlock.type.toUpperCase()} block ${newBlock.id}`);
        } catch (error) {
            UIUtils.log(`[ERROR] Failed to drop block: ${error}`, 'error');
        }
    },
    
    /**
     * Render a block on the canvas
     */
    renderBlock(blockData) {
        const blockEl = BlockRenderer.createScratchBlock(blockData);
        if (!blockEl) return;
        
        // Snap position to grid when rendering
        blockData.x = this.snapToGrid(blockData.x);
        blockData.y = this.snapToGrid(blockData.y);
        
        blockEl.style.left = blockData.x + 'px';
        blockEl.style.top = blockData.y + 'px';
        
        this.canvas.appendChild(blockEl);
        
        // Cache the element
        this.blockElementCache.set(blockData.id, blockEl);
        
        // Make block draggable
        this.makeBlockDraggable(blockEl, blockData);
    },
    
    /**
     * Make a block draggable and interactive
     */
    makeBlockDraggable(blockEl, blockData) {
        let isDragging = false;
        let dragStart = { x: 0, y: 0 };
        let startPos = { x: 0, y: 0 };
        
        blockEl.addEventListener('mousedown', (e) => {
            if (e.target.classList.contains('block-connector') || 
                e.target.classList.contains('workflow-trigger-btn') ||
                e.target.classList.contains('remove-btn') ||
                e.target.tagName === 'INPUT' || 
                e.target.tagName === 'SELECT') {
                return;
            }
            
            isDragging = true;
            const rect = blockEl.getBoundingClientRect();
            const canvasRect = this.canvas.getBoundingClientRect();
            
            dragStart = {
                x: e.clientX - rect.left,
                y: e.clientY - rect.top
            };
            
            startPos = {
                x: rect.left - canvasRect.left + this.canvas.scrollLeft,
                y: rect.top - canvasRect.top + this.canvas.scrollTop
            };
            
            blockEl.style.cursor = 'grabbing';
            this.selectBlock(blockData.id);
            e.preventDefault();
        });
        
        window.addEventListener('mousemove', (e) => {
            if (!isDragging) return;
            
            // Throttle drag updates for better performance
            const now = performance.now();
            const shouldUpdate = now - this.lastDragUpdate >= this.dragUpdateThrottle;
            
            const canvasRect = this.canvas.getBoundingClientRect();
            let newX = e.clientX - canvasRect.left + this.canvas.scrollLeft - dragStart.x;
            let newY = e.clientY - canvasRect.top + this.canvas.scrollTop - dragStart.y;
            
            // Snap to grid while dragging
            newX = this.snapToGrid(newX);
            newY = this.snapToGrid(newY);
            
            // Always update position immediately for smooth dragging
            blockEl.style.left = newX + 'px';
            blockEl.style.top = newY + 'px';
            
            // Update block data
            blockData.x = newX;
            blockData.y = newY;
            
            // Invalidate position cache for this block
            this.blockPositionCache.delete(blockData.id);
            
            if (shouldUpdate) {
                this.lastDragUpdate = now;
                
                // Check for snapping
                BlockConnector.checkSnapping(blockEl, blockData);
                
                // Schedule throttled updates instead of calling directly
                this.scheduleConnectionsUpdate();
                this.scheduleCanvasSizeUpdate();
            }
        });
        
        window.addEventListener('mouseup', () => {
            if (isDragging) {
                isDragging = false;
                blockEl.style.cursor = 'move';
                
                // Clear preview
                BlockConnector.clearPreviewConnection();
                BlockConnector.previewTarget = null;
                
                // Remove snapping class from all blocks
                document.querySelectorAll('.scratch-block').forEach(el => {
                    el.classList.remove('snapping');
                });
                
                // Snap blocks if close
                BlockConnector.snapBlocks(blockEl, blockData);
                
                // Update connections and trigger links
                BlockConnector.updateConnections();
                
                StorageManager.autoSave();
            }
        });
        
        // Add right-click context menu to create workflow
        blockEl.addEventListener('contextmenu', (e) => {
            if (blockData.type !== 'event') {
                e.preventDefault();
                const createWorkflow = UIUtils.confirm(`Create a workflow from block ${blockData.id}? This will make it trigger when completed.`);
                if (createWorkflow) {
                    this.createWorkflowFromBlock(blockData.id);
                }
            }
        });
    },
    
    /**
     * Select a block
     */
    selectBlock(blockId) {
        if (this.selectedBlockId === blockId) return;
        
        this.deselectBlock();
        
        this.selectedBlockId = blockId;
        const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
        if (blockEl) {
            blockEl.classList.add('selected');
        }
    },
    
    /**
     * Deselect current block
     */
    deselectBlock() {
        if (this.selectedBlockId) {
            const blockEl = document.querySelector(`[data-block-id="${this.selectedBlockId}"]`);
            if (blockEl) {
                blockEl.classList.remove('selected');
            }
            this.selectedBlockId = null;
        }
    },
    
    /**
     * Remove a block
     */
    removeBlock(blockId) {
        const block = this.blocks.get(blockId);
        if (!block) return;
        
        // Remove workflows triggered by this block
        if (block.triggersWorkflows) {
            const workflowsToRemove = [...block.triggersWorkflows];
            workflowsToRemove.forEach(workflowId => {
                const workflow = this.workflows.get(workflowId);
                if (workflow) {
                    // Remove all blocks in the workflow (starting from the event block)
                    const eventBlockId = workflow.rootBlockId;
                    if (eventBlockId && eventBlockId !== blockId) {
                        // Remove all blocks in the workflow chain (avoid recursive call)
                        const eventBlock = this.blocks.get(eventBlockId);
                        if (eventBlock) {
                            // Remove connected blocks
                            const nextBlocks = BlockConnector.getNextBlocks(eventBlockId);
                            nextBlocks.forEach(nextId => {
                                // Remove chain without removing the block itself (to avoid recursion)
                                this.removeBlockChainOnly(nextId);
                            });
                            
                            // Remove connections from event block
                            BlockConnector.removeBlockConnections(eventBlockId);
                            
                            // Remove the event block
                            this.blocks.delete(eventBlockId);
                            const eventBlockEl = document.querySelector(`[data-block-id="${eventBlockId}"]`);
                            if (eventBlockEl) {
                                eventBlockEl.remove();
                            }
                        }
                    }
                    this.workflows.delete(workflowId);
                }
            });
            // Clear triggersWorkflows from the block
            block.triggersWorkflows = [];
        }
        
        // If this is a workflow-complete event block, clean up the trigger relationship
        if (block.type === 'event' && block.eventType === 'workflow-complete' && block.triggeredBy) {
            const triggeringBlock = this.blocks.get(block.triggeredBy);
            if (triggeringBlock && triggeringBlock.triggersWorkflows) {
                // Remove this workflow from the triggering block's list
                const index = triggeringBlock.triggersWorkflows.indexOf(block.workflowId);
                if (index !== -1) {
                    triggeringBlock.triggersWorkflows.splice(index, 1);
                }
                // Update indicators
                this.updateBlockTriggerIndicators(block.triggeredBy);
            }
        }
        
        // Remove from workflow if in one
        if (block.workflowId) {
            const workflow = this.workflows.get(block.workflowId);
            if (workflow) {
                // If this is the root block, remove the entire workflow
                if (workflow.rootBlockId === blockId) {
                    // Remove all blocks in the workflow
                    const blocksToRemove = [...workflow.blocks];
                    blocksToRemove.forEach(bid => {
                        if (bid !== blockId) {
                            // Don't call removeBlock to avoid recursion - just remove directly
                            const b = this.blocks.get(bid);
                            if (b) {
                                // Remove connected blocks
                                const nextBlocks = BlockConnector.getNextBlocks(bid);
                                nextBlocks.forEach(nextId => {
                                    this.removeBlockChainOnly(nextId);
                                });
                                
                                // Remove connections
                                BlockConnector.removeBlockConnections(bid);
                                
                                // Remove block
                                this.blocks.delete(bid);
                                const bEl = document.querySelector(`[data-block-id="${bid}"]`);
                                if (bEl) {
                                    bEl.remove();
                                }
                            }
                        }
                    });
                    this.workflows.delete(block.workflowId);
                } else {
                    // Remove all connected blocks too
                    this.removeBlockChain(blockId);
                }
            }
        }
        
        // Remove connections
        BlockConnector.removeBlockConnections(blockId);
        
        // Remove from blocks map
        this.blocks.delete(blockId);
        
        // Clear caches for this block
        this.clearBlockCache(blockId);
        
        // Remove from DOM
        const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
        if (blockEl) {
            blockEl.remove();
        }
        
        // Update trigger links and indicators
        BlockConnector.updateConnections();
        
        // Update trigger indicators for any blocks that might have been triggering this one
        // (in case this was an event block that was being triggered)
        WorkflowManager.blocks.forEach((b, bid) => {
            if (b.triggersWorkflows) {
                // Check if any of the workflows this block triggers are now invalid
                b.triggersWorkflows = b.triggersWorkflows.filter(wid => {
                    const w = this.workflows.get(wid);
                    return w !== undefined;
                });
                // Update indicators
                this.updateBlockTriggerIndicators(bid);
            }
        });
        
        StorageManager.autoSave();
        UIUtils.log(`[WORKFLOW] Removed block ${blockId}`);
    },
    
    /**
     * Remove a block and all blocks connected below it
     */
    removeBlockChain(blockId) {
        const block = this.blocks.get(blockId);
        if (!block) return;
        
        // Remove connected blocks below
        const nextBlocks = BlockConnector.getNextBlocks(blockId);
        nextBlocks.forEach(nextId => {
            this.removeBlockChain(nextId);
        });
        
        // Remove connections
        BlockConnector.removeBlockConnections(blockId);
        
        // Remove this block
        this.blocks.delete(blockId);
        const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
        if (blockEl) {
            blockEl.remove();
        }
    },
    
    /**
     * Remove a block chain without removing the block itself (for cleanup)
     */
    removeBlockChainOnly(blockId) {
        const block = this.blocks.get(blockId);
        if (!block) return;
        
        // Remove connected blocks below
        const nextBlocks = BlockConnector.getNextBlocks(blockId);
        nextBlocks.forEach(nextId => {
            this.removeBlockChainOnly(nextId);
        });
        
        // Remove connections
        BlockConnector.removeBlockConnections(blockId);
        
        // Remove this block from blocks map and DOM (but don't call removeBlock to avoid recursion)
        this.blocks.delete(blockId);
        const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
        if (blockEl) {
            blockEl.remove();
        }
    },
    
    /**
     * Render all blocks
     */
    renderAll() {
        // Save the toolbar before clearing
        const toolbar = document.getElementById('workspaceToolbar');
        const toolbarParent = toolbar ? toolbar.parentElement : null;
        
        // Clear caches before re-rendering
        this.clearBlockCache();
        
        // Clear canvas but preserve toolbar
        this.canvas.innerHTML = '';
        
        // Re-append toolbar if it existed
        if (toolbar && toolbarParent === this.canvas) {
            this.canvas.appendChild(toolbar);
        }
        
        this.blocks.forEach(blockData => {
            this.renderBlock(blockData);
        });
        
        // Initialize connections from saved data
        BlockConnector.initializeConnections();
        
        // Update trigger indicators for all blocks that trigger workflows
        this.blocks.forEach((block, blockId) => {
            if (block.triggersWorkflows && block.triggersWorkflows.length > 0) {
                this.updateBlockTriggerIndicators(blockId);
            }
        });
        
        // Update trigger links and canvas size
        setTimeout(() => {
            BlockConnector.updateConnections();
            this.updateCanvasSize();
        }, 10);
    },
    
    /**
     * Clear workspace
     */
    clearWorkspace() {
        if (UIUtils.confirm('Clear workspace? This cannot be undone.')) {
            // Save the toolbar before clearing
            const toolbar = document.getElementById('workspaceToolbar');
            const toolbarParent = toolbar ? toolbar.parentElement : null;
            
            this.blocks.clear();
            this.workflows.clear();
            this.blockIdCounter = 0;
            this.workflowIdCounter = 0;
            this.selectedBlockId = null;
            
            // Clear caches
            this.clearBlockCache();
            
            this.canvas.innerHTML = '';
            
            // Re-append toolbar if it existed
            if (toolbar && toolbarParent === this.canvas) {
                this.canvas.appendChild(toolbar);
            }
            
            BlockConnector.clear();
            StorageManager.autoSave();
            StorageManager.updateWorkspaceUI();
            UIUtils.log('[WORKFLOW] Workspace cleared');
        }
    },
    
    /**
     * Get all workflows that can start (have event blocks)
     */
    getStartableWorkflows() {
        const startable = [];
        this.workflows.forEach((workflow, workflowId) => {
            const rootBlock = this.blocks.get(workflow.rootBlockId);
            if (rootBlock && rootBlock.type === 'event') {
                if (rootBlock.eventType === 'green-flag') {
                    startable.push(workflowId);
                }
            }
        });
        return startable;
    },
    
    /**
     * Get workflow triggered by another block completion
     */
    getWorkflowsTriggeredBy(blockId) {
        const triggered = [];
        this.workflows.forEach((workflow, workflowId) => {
            const rootBlock = this.blocks.get(workflow.rootBlockId);
            if (rootBlock && rootBlock.type === 'event' && 
                rootBlock.eventType === 'workflow-complete' &&
                rootBlock.triggeredBy === blockId) {
                triggered.push(workflowId);
            }
        });
        return triggered;
    },
    
    /**
     * Create a new empty workflow with an event block
     * @param {string} eventType - Type of event ('green-flag' or 'workflow-complete')
     */
    createNewWorkflow(eventType = 'green-flag') {
            // Create event block for the workflow
        this.blockIdCounter++;
        const eventBlock = {
            id: this.blockIdCounter,
            type: 'event',
            eventType: eventType,
            connections: { prev: null, next: null },
            x: this.snapToGrid(50 + Math.random() * 100),
            y: this.snapToGrid(50 + Math.random() * 100)
        };
        
        // Create workflow
        this.workflowIdCounter++;
        const workflow = {
            id: this.workflowIdCounter,
            rootBlockId: eventBlock.id,
            blocks: new Set([eventBlock.id]),
            triggeredBy: null // Will be set if triggered by another workflow
        };
        
        // Add event block to blocks map
        this.blocks.set(eventBlock.id, eventBlock);
        eventBlock.workflowId = workflow.id;
        
        // Add workflow
        this.workflows.set(workflow.id, workflow);
        
        // Render the event block
        this.renderBlock(eventBlock);
        
        StorageManager.autoSave();
        UIUtils.log(`[WORKFLOW] Created new workflow ${workflow.id} with ${eventType} event`);
        return workflow.id;
    },
    
    /**
     * Create a workflow starting from a block (for blocks that can trigger workflows)
     * Creates a workflow-complete event block that triggers when the specified block completes
     * @param {number} blockId - Block ID that should trigger the workflow
     */
    createWorkflowFromBlock(blockId) {
        const block = this.blocks.get(blockId);
        if (!block) return null;
        
            // Create a workflow-complete event block
        this.blockIdCounter++;
        const eventBlock = {
            id: this.blockIdCounter,
            type: 'event',
            eventType: 'workflow-complete',
            connections: { prev: null, next: null },
            triggeredBy: blockId, // This workflow is triggered when blockId completes
            x: this.snapToGrid(block.x + 200), // Position to the right of the triggering block (snapped to grid)
            y: this.snapToGrid(block.y)
        };
        
        // Create new workflow with the event block as root
        this.workflowIdCounter++;
        const workflow = {
            id: this.workflowIdCounter,
            rootBlockId: eventBlock.id,
            blocks: new Set([eventBlock.id]),
            triggeredBy: blockId // Mark that this workflow is triggered by a block
        };
        this.workflows.set(workflow.id, workflow);
        eventBlock.workflowId = workflow.id;
        
        // Add event block to blocks map
        this.blocks.set(eventBlock.id, eventBlock);
        
        // Store trigger relationship in the triggering block
        if (!block.triggersWorkflows) {
            block.triggersWorkflows = [];
        }
        block.triggersWorkflows.push(workflow.id);
        
        // Render the event block
        this.renderBlock(eventBlock);
        
        // Update the triggering block's visual indicators
        this.updateBlockTriggerIndicators(blockId);
        
        // Update trigger links
        setTimeout(() => {
            BlockConnector.updateConnections();
        }, 10);
        
        StorageManager.autoSave();
        UIUtils.log(`[WORKFLOW] Created workflow ${workflow.id} triggered by block ${blockId} completion`);
        return workflow.id;
    },
    
    /**
     * Update visual indicators for a block that triggers workflows
     */
    updateBlockTriggerIndicators(blockId) {
        const block = this.blocks.get(blockId);
        if (!block) return;
        
        const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
        if (!blockEl) return;
        
        // Update workflow trigger button
        const workflowBtn = blockEl.querySelector('.workflow-trigger-btn');
        if (workflowBtn) {
            const triggersWorkflows = block.triggersWorkflows && block.triggersWorkflows.length > 0;
            if (triggersWorkflows) {
                workflowBtn.classList.add('active');
                workflowBtn.title = `Triggers ${block.triggersWorkflows.length} workflow(s) - Click to create another`;
            } else {
                workflowBtn.classList.remove('active');
                workflowBtn.title = 'Create workflow from this block';
            }
        }
        
        // Update or create trigger indicator
        let indicator = blockEl.querySelector('.workflow-trigger-indicator');
        const triggersWorkflows = block.triggersWorkflows && block.triggersWorkflows.length > 0;
        
        if (triggersWorkflows && !indicator) {
            // Create indicator
            indicator = document.createElement('div');
            indicator.className = 'workflow-trigger-indicator';
            indicator.title = `This block triggers ${block.triggersWorkflows.length} workflow(s)`;
            blockEl.appendChild(indicator);
        } else if (!triggersWorkflows && indicator) {
            // Remove indicator
            indicator.remove();
        } else if (triggersWorkflows && indicator) {
            // Update indicator tooltip
            indicator.title = `This block triggers ${block.triggersWorkflows.length} workflow(s)`;
        }
    },
    
    /**
     * Link a workflow to be triggered by another workflow
     * @param {number} fromWorkflowId - Workflow that triggers
     * @param {number} toWorkflowId - Workflow to be triggered
     */
    linkWorkflows(fromWorkflowId, toWorkflowId) {
        const fromWorkflow = this.workflows.get(fromWorkflowId);
        const toWorkflow = this.workflows.get(toWorkflowId);
        
        if (!fromWorkflow || !toWorkflow) {
            UIUtils.log('[WORKFLOW] Invalid workflow IDs for linking', 'error');
            return false;
        }
        
        // Get the last block in the from workflow
        const fromRootBlock = this.blocks.get(fromWorkflow.rootBlockId);
        if (!fromRootBlock) return false;
        
        // Find the last block in the chain
        let lastBlockId = fromRootBlock.id;
        const chain = BlockConnector.getBlockChain(fromRootBlock.id);
        if (chain.length > 0) {
            lastBlockId = chain[chain.length - 1];
        }
        
        // Get the root block of the to workflow (should be a workflow-complete event)
        const toRootBlock = this.blocks.get(toWorkflow.rootBlockId);
        if (!toRootBlock) return false;
        
        // Set the trigger relationship
        toRootBlock.triggeredBy = lastBlockId;
        toWorkflow.triggeredBy = lastBlockId;
        
        if (!toRootBlock.triggeredByWorkflows) {
            toRootBlock.triggeredByWorkflows = [];
        }
        toRootBlock.triggeredByWorkflows.push(fromWorkflowId);
        
        StorageManager.autoSave();
        UIUtils.log(`[WORKFLOW] Linked workflow ${fromWorkflowId} -> ${toWorkflowId}`);
        return true;
    },
    
    /**
     * Get workflows that are triggered by a specific block
     */
    getWorkflowsTriggeredByBlock(blockId) {
        const workflows = [];
        this.workflows.forEach((workflow, workflowId) => {
            if (workflow.triggeredBy === blockId) {
                workflows.push(workflowId);
            }
        });
        return workflows;
    },
    
    /**
     * Clean up workspace layout - snap blocks to nearest row and column
     */
    cleanUpLayout() {
        if (this.blocks.size === 0) {
            UIUtils.log('[WORKFLOW] No blocks to organize', 'warning');
            return;
        }
        
        const CLUSTERING_THRESHOLD = 100; // Distance threshold for grouping positions
        
        // Collect all current block positions
        const xPositions = [];
        const yPositions = [];
        
        this.blocks.forEach(block => {
            xPositions.push(block.x);
            yPositions.push(block.y);
        });
        
        // Find column positions by clustering X coordinates
        const columns = this.clusterPositions(xPositions, CLUSTERING_THRESHOLD);
        
        // Find row positions by clustering Y coordinates
        const rows = this.clusterPositions(yPositions, CLUSTERING_THRESHOLD);
        
        // Snap each block to nearest column and row
        this.blocks.forEach((block, blockId) => {
            // Find nearest column
            const nearestColumn = this.findNearestPosition(block.x, columns);
            
            // Find nearest row
            const nearestRow = this.findNearestPosition(block.y, rows);
            
            // Update block position
            block.x = this.snapToGrid(nearestColumn);
            block.y = this.snapToGrid(nearestRow);
            
            // Update DOM element position
            const blockEl = document.querySelector(`[data-block-id="${blockId}"]`);
            if (blockEl) {
                blockEl.style.left = block.x + 'px';
                blockEl.style.top = block.y + 'px';
            }
        });
        
        // Update all connections to reflect new positions
        BlockConnector.updateConnections();
        
        // Update canvas size to fit all blocks
        this.updateCanvasSize();
        
        // Auto-save
        StorageManager.autoSave();
        
        UIUtils.log('[WORKFLOW] Blocks snapped to nearest rows and columns', 'success');
    },
    
    /**
     * Cluster positions into groups based on proximity
     * @param {number[]} positions - Array of position values
     * @param {number} threshold - Distance threshold for clustering
     * @returns {number[]} - Array of cluster center positions
     */
    clusterPositions(positions, threshold) {
        if (positions.length === 0) return [];
        
        // Sort positions
        const sorted = [...positions].sort((a, b) => a - b);
        
        // Group nearby positions
        const clusters = [];
        let currentCluster = [sorted[0]];
        
        for (let i = 1; i < sorted.length; i++) {
            if (sorted[i] - sorted[i - 1] <= threshold) {
                // Add to current cluster
                currentCluster.push(sorted[i]);
            } else {
                // Start new cluster
                clusters.push(currentCluster);
                currentCluster = [sorted[i]];
            }
        }
        clusters.push(currentCluster);
        
        // Calculate center of each cluster
        const centers = clusters.map(cluster => {
            const sum = cluster.reduce((a, b) => a + b, 0);
            return Math.round(sum / cluster.length);
        });
        
        return centers;
    },
    
    /**
     * Find nearest position from a list
     * @param {number} value - Current position value
     * @param {number[]} positions - Array of target positions
     * @returns {number} - Nearest position
     */
    findNearestPosition(value, positions) {
        if (positions.length === 0) return value;
        
        let nearest = positions[0];
        let minDistance = Math.abs(value - positions[0]);
        
        for (let i = 1; i < positions.length; i++) {
            const distance = Math.abs(value - positions[i]);
            if (distance < minDistance) {
                minDistance = distance;
                nearest = positions[i];
            }
        }
        
        return nearest;
    },
    
    /**
     * Initialize from storage
     */
    initialize() {
        this.blocks.clear();
        this.workflows.clear();
        this.blockIdCounter = 0;
        this.workflowIdCounter = 0;
        StorageManager.loadFromStorage();
        this.renderAll();
        // Update workspace UI after loading
        StorageManager.updateWorkspaceUI();
    }
};

