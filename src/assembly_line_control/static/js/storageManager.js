/**
 * Storage Manager - Handles saving and loading projects to/from server
 * Projects are stored server-side for persistence across devices/sessions
 */
const StorageManager = {
    currentProject: null, // Current project metadata
    autoSaveTimer: null, // Timer for debounced auto-save
    autoSaveDelay: 1500, // Delay in ms for auto-save debouncing (longer reduces jank during editing)
    projectsCache: [], // Cache of project list
    isLoading: false, // Prevent concurrent operations
    
    /**
     * Fetch all projects from server
     * @returns {Promise<Array>} - List of project metadata
     */
    async fetchProjects() {
        try {
            const response = await fetch('/api/projects');
            if (!response.ok) {
                throw new Error(`HTTP error: ${response.status}`);
            }
            const data = await response.json();
            this.projectsCache = data.projects || [];
            return this.projectsCache;
        } catch (error) {
            console.error('Failed to fetch projects:', error);
            UIUtils.log('[ERROR] Failed to fetch projects: ' + error.message, 'error');
            return [];
        }
    },
    
    /**
     * Get all saved projects (cached version)
     * @returns {Array} - List of project metadata
     */
    getAllProjects() {
        return this.projectsCache;
    },
    
    /**
     * Get current project info
     * @returns {Object|null} - Current project metadata
     */
    getCurrentProject() {
        return this.currentProject;
    },
    
    /**
     * Set current project
     * @param {Object} project - Project metadata {id, name, description}
     */
    setCurrentProject(project) {
        this.currentProject = project;
        // Also save to localStorage for quick reload
        if (project) {
            localStorage.setItem(Config.CURRENT_WORKSPACE_KEY, JSON.stringify(project));
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
            version: Config.VERSION
        };
    },
    
    /**
     * Save current project to server
     * @param {string} projectName - Project name (optional, uses current if not provided)
     * @param {string} description - Project description
     * @returns {Promise<boolean>} - Success status
     */
    async save(projectName = null, description = null) {
        if (this.isLoading) {
            UIUtils.log('[SAVE] Operation in progress, please wait...', 'warning');
            return false;
        }
        
        try {
            this.isLoading = true;
            const name = projectName || (this.currentProject?.name) || 'Untitled Project';
            const desc = description !== null ? description : (this.currentProject?.description || '');
            
            const config = this.buildConfig();
            config.name = name;
            config.description = desc;
            
            const response = await fetch('/api/projects', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(config)
            });
            
            const result = await response.json();
            
            if (!response.ok) {
                throw new Error(result.error || 'Failed to save project');
            }
            
            // Update current project
            this.setCurrentProject({
                id: result.id,
                name: result.name,
                description: desc
            });
            
            // Save motor speeds locally
            MotorSpeedManager.save();
            
            UIUtils.log(`[SAVE] Project "${name}" saved to server`, 'success');
            this.updateProjectUI();
            
            // Refresh projects cache
            await this.fetchProjects();
            
            return true;
        } catch (error) {
            UIUtils.log('[ERROR] Failed to save project: ' + error.message, 'error');
            console.error('Save error:', error);
            return false;
        } finally {
            this.isLoading = false;
        }
    },
    
    /**
     * Apply a configuration object to the workspace (used by load and undo)
     * @param {Object} config - Configuration object from buildConfig or loaded JSON
     * @param {Object} options - Options: { skipProjectUpdate: true } to preserve current project (for undo)
     */
    applyConfig(config, options = {}) {
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
                    return;
                }
                
                const blockIds = (workflow.blocks || []).map(id => parseInt(id)).filter(id => {
                    return id && WorkflowManager.blocks.has(id);
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
                    return;
                }
                
                let next = conn.next || null;
                if (next !== null && next !== undefined) {
                    if (!Array.isArray(next)) {
                        next = [next];
                    }
                    next = next.map(id => parseInt(id)).filter(id => id && WorkflowManager.blocks.has(id));
                    if (next.length === 0) {
                        next = null;
                    }
                }
                
                let prev = conn.prev || null;
                if (prev !== null && prev !== undefined) {
                    prev = parseInt(prev);
                    if (!prev || !WorkflowManager.blocks.has(prev)) {
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
        
        // Validate workflow references
        WorkflowManager.blocks.forEach((block, blockId) => {
            if (block.workflowId) {
                const workflowId = parseInt(block.workflowId);
                if (!WorkflowManager.workflows.has(workflowId)) {
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
                    block.triggeredBy = undefined;
                }
            }
            
            if (block.triggersWorkflows && Array.isArray(block.triggersWorkflows)) {
                block.triggersWorkflows = block.triggersWorkflows.map(wid => parseInt(wid)).filter(wid => {
                    return wid && WorkflowManager.workflows.has(wid);
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
        
        if (!options.skipProjectUpdate) {
            this.setCurrentProject({
                id: config.id || null,
                name: config.name || 'Untitled Project',
                description: config.description || ''
            });
        }
        
        // Re-render everything
        WorkflowManager.renderAll();
        
        if (!options.skipProjectUpdate) {
            this.updateProjectUI();
        }
    },
    
    /**
     * Apply a configuration without rendering (for remote/headless execution).
     * Same as applyConfig but skips renderAll() and UI updates so execution works without a canvas.
     * @param {Object} config - Configuration object from buildConfig or loaded JSON
     */
    applyConfigHeadless(config) {
        if (typeof ExecutionEngine !== 'undefined') {
            ExecutionEngine.stop();
        }
        
        WorkflowManager.blocks.clear();
        WorkflowManager.workflows.clear();
        BlockConnector.connections.clear();
        
        let maxBlockId = 0;
        if (config.blocks && Array.isArray(config.blocks)) {
            config.blocks.forEach(block => {
                const blockId = parseInt(block.id) || 0;
                if (blockId === 0) return;
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
                if (blockId > maxBlockId) maxBlockId = blockId;
            });
        }
        
        let maxWorkflowId = 0;
        if (config.workflows && Array.isArray(config.workflows)) {
            config.workflows.forEach(workflow => {
                const workflowId = parseInt(workflow.id) || 0;
                if (workflowId === 0) return;
                const blockIds = (workflow.blocks || []).map(id => parseInt(id)).filter(id => id && WorkflowManager.blocks.has(id));
                WorkflowManager.workflows.set(workflowId, {
                    ...workflow,
                    id: workflowId,
                    blocks: new Set(blockIds)
                });
                if (workflowId > maxWorkflowId) maxWorkflowId = workflowId;
            });
        }
        
        if (config.connections && Array.isArray(config.connections)) {
            config.connections.forEach(conn => {
                const blockId = parseInt(conn.id) || 0;
                if (blockId === 0 || !WorkflowManager.blocks.has(blockId)) return;
                let next = conn.next || null;
                if (next !== null && next !== undefined) {
                    if (!Array.isArray(next)) next = [next];
                    next = next.map(id => parseInt(id)).filter(id => id && WorkflowManager.blocks.has(id));
                    if (next.length === 0) next = null;
                }
                let prev = conn.prev || null;
                if (prev !== null && prev !== undefined) {
                    prev = parseInt(prev);
                    if (!prev || !WorkflowManager.blocks.has(prev)) prev = null;
                }
                BlockConnector.connections.set(blockId, { prev: prev, next: next });
                const block = WorkflowManager.blocks.get(blockId);
                if (block) {
                    if (!block.connections) block.connections = {};
                    block.connections.prev = prev;
                    block.connections.next = next;
                }
            });
        }
        
        WorkflowManager.blocks.forEach((block, blockId) => {
            if (block.workflowId) {
                const workflowId = parseInt(block.workflowId);
                if (!WorkflowManager.workflows.has(workflowId)) {
                    block.workflowId = undefined;
                } else {
                    const workflow = WorkflowManager.workflows.get(workflowId);
                    if (workflow && !workflow.blocks.has(blockId)) workflow.blocks.add(blockId);
                }
            }
            if (block.triggeredBy) {
                const triggeredById = parseInt(block.triggeredBy);
                if (!WorkflowManager.blocks.has(triggeredById)) block.triggeredBy = undefined;
            }
            if (block.triggersWorkflows && Array.isArray(block.triggersWorkflows)) {
                block.triggersWorkflows = block.triggersWorkflows.map(wid => parseInt(wid)).filter(wid => wid && WorkflowManager.workflows.has(wid));
            }
        });
        
        WorkflowManager.blockIdCounter = Math.max(maxBlockId + 1, config.blockIdCounter || 0);
        WorkflowManager.workflowIdCounter = Math.max(maxWorkflowId + 1, config.workflowIdCounter || 0);
        
        if (config.motorSpeeds && typeof MotorSpeedManager !== 'undefined') {
            MotorSpeedManager.speeds = { ...MotorSpeedManager.speeds, ...config.motorSpeeds };
        }
        if (config.customPaths && typeof BlockConnector !== 'undefined' && BlockConnector.importCustomPaths) {
            BlockConnector.importCustomPaths(config.customPaths);
        }
    },
    
    /**
     * Load project from server
     * @param {string} projectId - Project ID to load
     * @param {boolean} confirmFirst - Whether to ask for confirmation
     * @returns {Promise<boolean>} - Success status
     */
    async load(projectId, confirmFirst = true) {
        if (this.isLoading) {
            UIUtils.log('[LOAD] Operation in progress, please wait...', 'warning');
            return false;
        }
        
        try {
            this.isLoading = true;
            
            if (confirmFirst && !UIUtils.confirm(`Load project? This will replace the current workspace.`)) {
                return false;
            }
            
            const response = await fetch(`/api/projects/${encodeURIComponent(projectId)}`);
            
            if (!response.ok) {
                const result = await response.json();
                throw new Error(result.error || 'Failed to load project');
            }
            
            const config = await response.json();
            config.id = projectId;
            
            if (typeof UndoManager !== 'undefined') {
                UndoManager.clear();
            }
            this.applyConfig(config);
            
            UIUtils.log(`[LOAD] Project "${config.name || projectId}" loaded`, 'success');
            this.updateProjectUI();
            return true;
        } catch (error) {
            UIUtils.log('[ERROR] Failed to load project: ' + error.message, 'error');
            console.error('Load error:', error);
            return false;
        } finally {
            this.isLoading = false;
        }
    },
    
    /**
     * Delete a project from server
     * @param {string} projectId - Project ID to delete
     * @returns {Promise<boolean>} - Success status
     */
    async deleteProject(projectId) {
        if (!projectId) return false;
        
        if (!UIUtils.confirm(`Delete this project? This cannot be undone.`)) {
            return false;
        }
        
        try {
            const response = await fetch(`/api/projects/${encodeURIComponent(projectId)}`, {
                method: 'DELETE'
            });
            
            const result = await response.json();
            
            if (!response.ok) {
                throw new Error(result.error || 'Failed to delete project');
            }
            
            // If it was the current project, clear current
            if (this.currentProject?.id === projectId) {
                this.setCurrentProject(null);
            }
            
            UIUtils.log(`[DELETE] ${result.message}`, 'success');
            
            // Refresh projects cache
            await this.fetchProjects();
            this.updateProjectUI();
            
            return true;
        } catch (error) {
            UIUtils.log('[ERROR] Failed to delete project: ' + error.message, 'error');
            return false;
        }
    },
    
    /**
     * Create a new empty project
     * @param {string} name - Project name
     * @param {string} description - Project description
     * @returns {Promise<boolean>} - Success status
     */
    async createNew(name = 'Untitled Project', description = '') {
        if (typeof UndoManager !== 'undefined') {
            UndoManager.clear();
        }
        // Clear current state
        WorkflowManager.blocks.clear();
        WorkflowManager.workflows.clear();
        BlockConnector.connections.clear();
        WorkflowManager.blockIdCounter = 0;
        WorkflowManager.workflowIdCounter = 0;
        WorkflowManager.renderAll();
        
        // Set as new project (not saved yet)
        this.setCurrentProject({
            id: null,
            name: name,
            description: description,
            isNew: true
        });
        
        this.updateProjectUI();
        UIUtils.log(`[NEW] Created new project "${name}"`, 'success');
        return true;
    },
    
    /**
     * Auto-load last project or initialize fresh
     */
    async loadFromStorage() {
        try {
            // First fetch projects list
            await this.fetchProjects();
            
            // Try to load last used project from localStorage
            const savedProject = localStorage.getItem(Config.CURRENT_WORKSPACE_KEY);
            if (savedProject) {
                try {
                    const project = JSON.parse(savedProject);
                    if (project.id) {
                        // Try to load this project
                        const loaded = await this.load(project.id, false);
                        if (loaded) return;
                    }
                } catch (e) {
                    console.warn('Could not parse saved project:', e);
                }
            }
            
            // Fallback: load most recent project if available
            if (this.projectsCache.length > 0) {
                const mostRecent = this.projectsCache[0];
                await this.load(mostRecent.id, false);
                return;
            }
            
            // No projects found, start fresh
            this.createNew();
        } catch (error) {
            console.error('Auto-load error:', error);
            this.createNew();
        }
    },
    
    /**
     * Auto-save current project (debounced)
     */
    autoSave() {
        if (!Config.AUTO_SAVE_ENABLED) return;
        
        // Only auto-save if we have a saved project
        if (!this.currentProject?.id) return;
        
        // Clear existing timer
        if (this.autoSaveTimer) {
            clearTimeout(this.autoSaveTimer);
        }
        
        // Debounce auto-save
        this.autoSaveTimer = setTimeout(async () => {
            try {
                await this.save();
            } catch (error) {
                console.error('Auto-save error:', error);
            }
            this.autoSaveTimer = null;
        }, this.autoSaveDelay);
    },
    
    /**
     * Update project UI elements
     */
    updateProjectUI() {
        const projectNameEl = document.getElementById('currentProjectName');
        if (projectNameEl) {
            const name = this.currentProject?.name || 'No Project';
            const isNew = this.currentProject?.isNew;
            projectNameEl.textContent = name + (isNew ? ' (unsaved)' : '');
        }
    },
    
    // Legacy compatibility methods
    getCurrentWorkspace() {
        return this.currentProject?.name || null;
    },
    
    setCurrentWorkspace(name) {
        if (name) {
            this.currentProject = { ...this.currentProject, name };
        } else {
            this.currentProject = null;
        }
    },
    
    updateWorkspaceUI() {
        this.updateProjectUI();
    },
    
    updateWorkspaceList() {
        // No-op for compatibility
    },
    
    getAllWorkspaces() {
        // Convert projects to workspace format for compatibility
        const workspaces = {};
        this.projectsCache.forEach(p => {
            workspaces[p.name] = {
                name: p.name,
                timestamp: p.timestamp,
                version: p.version
            };
        });
        return workspaces;
    }
};
