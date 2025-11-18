/**
 * Storage Manager - Handles saving and loading configurations with multiple workspaces
 */
const StorageManager = {
    currentWorkspace: null, // Current workspace name
    autoSaveTimer: null, // Timer for debounced auto-save
    autoSaveDelay: 500, // Delay in ms for auto-save debouncing
    
    /**
     * Get all saved workspaces
     * @returns {Object} - Map of workspace names to metadata
     */
    getAllWorkspaces() {
        try {
            const workspacesData = localStorage.getItem(Config.WORKSPACES_KEY);
            if (!workspacesData) return {};
            return JSON.parse(workspacesData);
        } catch (error) {
            console.error('Failed to get workspaces:', error);
            return {};
        }
    },
    
    /**
     * Get current workspace name
     * @returns {string|null} - Current workspace name
     */
    getCurrentWorkspace() {
        if (!this.currentWorkspace) {
            const saved = localStorage.getItem(Config.CURRENT_WORKSPACE_KEY);
            this.currentWorkspace = saved || null;
        }
        return this.currentWorkspace;
    },
    
    /**
     * Set current workspace name
     * @param {string} name - Workspace name
     */
    setCurrentWorkspace(name) {
        this.currentWorkspace = name;
        if (name) {
            localStorage.setItem(Config.CURRENT_WORKSPACE_KEY, name);
        } else {
            localStorage.removeItem(Config.CURRENT_WORKSPACE_KEY);
        }
    },
    
    /**
     * Build configuration object from current state
     * @returns {Object} - Configuration object
     */
    buildConfig() {
        // Convert Maps to arrays for JSON serialization
        const blocks = Array.from(WorkflowManager.blocks.entries()).map(([id, block]) => {
            const blockData = { ...block };
            if (!blockData.id) {
                blockData.id = id;
            }
            return blockData;
        });
        
        const workflows = Array.from(WorkflowManager.workflows.entries()).map(([id, workflow]) => ({
            id,
            ...workflow,
            blocks: Array.from(workflow.blocks || [])
        }));
        
        const connections = Array.from(BlockConnector.connections.entries()).map(([id, conn]) => ({
            id,
            prev: conn.prev || null,
            next: Array.isArray(conn.next) ? conn.next : (conn.next ? [conn.next] : null)
        }));
        
        // Export custom paths
        const customPaths = BlockConnector.exportCustomPaths();
        
        return {
            blocks: blocks,
            workflows: workflows,
            connections: connections,
            customPaths: customPaths,
            blockIdCounter: WorkflowManager.blockIdCounter,
            workflowIdCounter: WorkflowManager.workflowIdCounter,
            motorSpeeds: MotorSpeedManager.speeds,
            version: Config.VERSION,
            timestamp: new Date().toISOString()
        };
    },
    
    /**
     * Save current workspace
     * @param {string} workspaceName - Workspace name (optional, uses current if not provided)
     * @returns {boolean} - Success status
     */
    save(workspaceName = null) {
        try {
            const name = workspaceName || this.getCurrentWorkspace() || 'default';
            const config = this.buildConfig();
            
            // Get existing workspaces
            const workspaces = this.getAllWorkspaces();
            
            // Update workspace
            workspaces[name] = {
                name: name,
                timestamp: new Date().toISOString(),
                version: Config.VERSION
            };
            
            // Save workspace data
            localStorage.setItem(`${Config.WORKSPACES_KEY}_${name}`, JSON.stringify(config));
            
            // Save workspace list
            localStorage.setItem(Config.WORKSPACES_KEY, JSON.stringify(workspaces));
            
            // Set as current if not already
            if (!this.getCurrentWorkspace()) {
                this.setCurrentWorkspace(name);
            }
            
            MotorSpeedManager.save();
            UIUtils.log(`[SAVE] Workspace "${name}" saved`, 'success');
            this.updateWorkspaceUI();
            return true;
        } catch (error) {
            UIUtils.log('[ERROR] Failed to save workspace: ' + error, 'error');
            return false;
        }
    },
    
    /**
     * Save workspace with a new name (Save As)
     * @param {string} workspaceName - New workspace name
     * @returns {boolean} - Success status
     */
    saveAs(workspaceName) {
        if (!workspaceName || workspaceName.trim() === '') {
            UIUtils.log('[ERROR] Workspace name cannot be empty', 'error');
            return false;
        }
        
        const name = workspaceName.trim();
        const workspaces = this.getAllWorkspaces();
        
        if (workspaces[name] && !UIUtils.confirm(`Workspace "${name}" already exists. Overwrite?`)) {
            return false;
        }
        
        this.setCurrentWorkspace(name);
        return this.save(name);
    },
    
    /**
     * Load workspace
     * @param {string} workspaceName - Workspace name to load
     * @param {boolean} confirmFirst - Whether to ask for confirmation first
     * @returns {boolean} - Success status
     */
    load(workspaceName = null, confirmFirst = true) {
        try {
            const name = workspaceName || this.getCurrentWorkspace();
            
            if (!name) {
                UIUtils.log('[LOAD] No workspace selected', 'error');
                return false;
            }
            
            const saved = localStorage.getItem(`${Config.WORKSPACES_KEY}_${name}`);
            if (!saved) {
                UIUtils.log(`[LOAD] Workspace "${name}" not found`, 'error');
                return false;
            }
            
            const config = JSON.parse(saved);
            
            if (confirmFirst && !UIUtils.confirm(`Load workspace "${name}"? This will replace the current setup.`)) {
                return false;
            }
            
            // Stop any active execution
            ExecutionEngine.stop();
            
            // Clear current state
            WorkflowManager.blocks.clear();
            WorkflowManager.workflows.clear();
            BlockConnector.connections.clear();
            
            // Load blocks first
            let maxBlockId = 0;
            if (config.blocks && Array.isArray(config.blocks)) {
                config.blocks.forEach(block => {
                    const blockId = parseInt(block.id) || 0;
                    if (blockId === 0) {
                        UIUtils.log('[LOAD] Skipping block with invalid ID', 'warning');
                        return;
                    }
                    
                    const blockData = { ...block };
                    blockData.id = blockId;
                    
                    if (!blockData.connections) {
                        blockData.connections = { prev: null, next: null };
                    }
                    
                    if (blockData.connections.next !== null && blockData.connections.next !== undefined) {
                        if (!Array.isArray(blockData.connections.next)) {
                            blockData.connections.next = [blockData.connections.next];
                        }
                    }
                    
                    WorkflowManager.blocks.set(blockId, blockData);
                    if (blockId > maxBlockId) {
                        maxBlockId = blockId;
                    }
                });
            }
            
            // Load workflows
            let maxWorkflowId = 0;
            if (config.workflows && Array.isArray(config.workflows)) {
                config.workflows.forEach(workflow => {
                    const workflowId = parseInt(workflow.id) || 0;
                    if (workflowId === 0) {
                        UIUtils.log('[LOAD] Skipping workflow with invalid ID', 'warning');
                        return;
                    }
                    
                    const blockIds = (workflow.blocks || []).map(id => parseInt(id)).filter(id => {
                        if (!id || !WorkflowManager.blocks.has(id)) {
                            UIUtils.log(`[LOAD] Workflow ${workflowId} references non-existent block ${id}`, 'warning');
                            return false;
                        }
                        return true;
                    });
                    
                    WorkflowManager.workflows.set(workflowId, {
                        ...workflow,
                        id: workflowId,
                        blocks: new Set(blockIds)
                    });
                    
                    if (workflowId > maxWorkflowId) {
                        maxWorkflowId = workflowId;
                    }
                });
            }
            
            // Load connections
            if (config.connections && Array.isArray(config.connections)) {
                config.connections.forEach(conn => {
                    const blockId = parseInt(conn.id) || 0;
                    if (blockId === 0 || !WorkflowManager.blocks.has(blockId)) {
                        UIUtils.log(`[LOAD] Skipping connection for non-existent block ${blockId}`, 'warning');
                        return;
                    }
                    
                    let next = conn.next || null;
                    if (next !== null && next !== undefined) {
                        if (!Array.isArray(next)) {
                            next = [next];
                        }
                        next = next.map(id => parseInt(id)).filter(id => {
                            if (!id || !WorkflowManager.blocks.has(id)) {
                                UIUtils.log(`[LOAD] Connection from block ${blockId} references non-existent block ${id}`, 'warning');
                                return false;
                            }
                            return true;
                        });
                        if (next.length === 0) {
                            next = null;
                        }
                    }
                    
                    let prev = conn.prev || null;
                    if (prev !== null && prev !== undefined) {
                        prev = parseInt(prev);
                        if (!prev || !WorkflowManager.blocks.has(prev)) {
                            UIUtils.log(`[LOAD] Connection from block ${blockId} references non-existent prev block ${prev}`, 'warning');
                            prev = null;
                        }
                    }
                    
                    BlockConnector.connections.set(blockId, {
                        prev: prev,
                        next: next
                    });
                    
                    const block = WorkflowManager.blocks.get(blockId);
                    if (block) {
                        if (!block.connections) block.connections = {};
                        block.connections.prev = prev;
                        block.connections.next = next;
                    }
                });
            }
            
            // Validate and fix workflow references in blocks
            WorkflowManager.blocks.forEach((block, blockId) => {
                if (block.workflowId) {
                    const workflowId = parseInt(block.workflowId);
                    if (!WorkflowManager.workflows.has(workflowId)) {
                        UIUtils.log(`[LOAD] Block ${blockId} references non-existent workflow ${workflowId}`, 'warning');
                        block.workflowId = undefined;
                    } else {
                        const workflow = WorkflowManager.workflows.get(workflowId);
                        if (workflow && !workflow.blocks.has(blockId)) {
                            workflow.blocks.add(blockId);
                        }
                    }
                }
                
                if (block.triggeredBy) {
                    const triggeredById = parseInt(block.triggeredBy);
                    if (!WorkflowManager.blocks.has(triggeredById)) {
                        UIUtils.log(`[LOAD] Block ${blockId} references non-existent triggeredBy block ${triggeredById}`, 'warning');
                        block.triggeredBy = undefined;
                    }
                }
                
                if (block.triggersWorkflows && Array.isArray(block.triggersWorkflows)) {
                    block.triggersWorkflows = block.triggersWorkflows.map(wid => parseInt(wid)).filter(wid => {
                        if (!wid || !WorkflowManager.workflows.has(wid)) {
                            UIUtils.log(`[LOAD] Block ${blockId} references non-existent workflow ${wid}`, 'warning');
                            return false;
                        }
                        return true;
                    });
                }
            });
            
            // Reassign counters
            WorkflowManager.blockIdCounter = Math.max(maxBlockId + 1, config.blockIdCounter || 0);
            WorkflowManager.workflowIdCounter = Math.max(maxWorkflowId + 1, config.workflowIdCounter || 0);
            
            // Load motor speeds if available
            if (config.motorSpeeds) {
                MotorSpeedManager.speeds = { ...MotorSpeedManager.speeds, ...config.motorSpeeds };
                MotorSpeedManager.updateUI(1);
                MotorSpeedManager.updateUI(2);
            }
            
            // Load custom paths if available
            if (config.customPaths) {
                BlockConnector.importCustomPaths(config.customPaths);
            }
            
            // Set as current workspace
            this.setCurrentWorkspace(name);
            
            // Re-render everything
            WorkflowManager.renderAll();
            
            UIUtils.log(`[LOAD] Workspace "${name}" loaded`, 'success');
            this.updateWorkspaceUI();
            return true;
        } catch (error) {
            UIUtils.log('[ERROR] Failed to load workspace: ' + error, 'error');
            console.error('Load error:', error);
            return false;
        }
    },
    
    /**
     * Delete a workspace
     * @param {string} workspaceName - Workspace name to delete
     * @returns {boolean} - Success status
     */
    deleteWorkspace(workspaceName) {
        if (!workspaceName) return false;
        
        if (!UIUtils.confirm(`Delete workspace "${workspaceName}"? This cannot be undone.`)) {
            return false;
        }
        
        try {
            const workspaces = this.getAllWorkspaces();
            if (!workspaces[workspaceName]) {
                UIUtils.log(`[DELETE] Workspace "${workspaceName}" not found`, 'error');
                return false;
            }
            
            // Remove from list
            delete workspaces[workspaceName];
            localStorage.setItem(Config.WORKSPACES_KEY, JSON.stringify(workspaces));
            
            // Remove workspace data
            localStorage.removeItem(`${Config.WORKSPACES_KEY}_${workspaceName}`);
            
            // If it was the current workspace, clear current
            if (this.getCurrentWorkspace() === workspaceName) {
                this.setCurrentWorkspace(null);
            }
            
            UIUtils.log(`[DELETE] Workspace "${workspaceName}" deleted`, 'success');
            this.updateWorkspaceUI();
            return true;
        } catch (error) {
            UIUtils.log('[ERROR] Failed to delete workspace: ' + error, 'error');
            return false;
        }
    },
    
    /**
     * Auto-load from storage on page load
     */
    loadFromStorage() {
        try {
            const currentName = this.getCurrentWorkspace();
            if (currentName) {
                // Try to load current workspace
                const saved = localStorage.getItem(`${Config.WORKSPACES_KEY}_${currentName}`);
                if (saved) {
                    // Load without confirmation on page load
                    this.load(currentName, false);
                    return;
                }
            }
            
            // Fallback: try to load default workspace
            const defaultSaved = localStorage.getItem(`${Config.WORKSPACES_KEY}_default`);
            if (defaultSaved) {
                this.setCurrentWorkspace('default');
                this.load('default', false);
            } else {
                // No workspace found, start fresh
                WorkflowManager.blocks.clear();
                WorkflowManager.workflows.clear();
                BlockConnector.connections.clear();
                WorkflowManager.renderAll();
            }
        } catch (error) {
            UIUtils.log('[ERROR] Failed to auto-load workspace: ' + error, 'error');
            console.error('Auto-load error:', error);
        }
    },
    
    /**
     * Auto-save current workspace (debounced for performance)
     */
    autoSave() {
        if (!Config.AUTO_SAVE_ENABLED) return;
        
        const currentName = this.getCurrentWorkspace();
        if (!currentName) return; // Don't auto-save if no workspace is set
        
        // Clear existing timer
        if (this.autoSaveTimer) {
            clearTimeout(this.autoSaveTimer);
        }
        
        // Debounce auto-save to avoid excessive saves during drag operations
        this.autoSaveTimer = setTimeout(() => {
            try {
                const config = this.buildConfig();
                localStorage.setItem(`${Config.WORKSPACES_KEY}_${currentName}`, JSON.stringify(config));
                MotorSpeedManager.save();
            } catch (error) {
                console.error('Auto-save error:', error);
            }
            this.autoSaveTimer = null;
        }, this.autoSaveDelay);
    },
    
    /**
     * Update workspace UI
     */
    updateWorkspaceUI() {
        const currentName = this.getCurrentWorkspace();
        const workspaceNameEl = document.getElementById('currentWorkspaceName');
        if (workspaceNameEl) {
            workspaceNameEl.textContent = currentName || 'None';
        }
        
        // Update workspace list in dropdown
        this.updateWorkspaceList();
    },
    
    /**
     * Update workspace list dropdown
     */
    updateWorkspaceList() {
        const workspaceListEl = document.getElementById('workspaceList');
        if (!workspaceListEl) return;
        
        const workspaces = this.getAllWorkspaces();
        const currentName = this.getCurrentWorkspace();
        
        workspaceListEl.innerHTML = '';
        
        if (Object.keys(workspaces).length === 0) {
            const option = document.createElement('option');
            option.value = '';
            option.textContent = 'No workspaces';
            workspaceListEl.appendChild(option);
            return;
        }
        
        // Sort workspaces by name
        const sortedNames = Object.keys(workspaces).sort();
        
        sortedNames.forEach(name => {
            const option = document.createElement('option');
            option.value = name;
            option.textContent = name;
            if (name === currentName) {
                option.selected = true;
            }
            workspaceListEl.appendChild(option);
        });
    }
};
