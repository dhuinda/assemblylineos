/**
 * Main Application - Sets up and coordinates everything
 */
const App = {
    /**
     * Get everything started up
     */
    async init() {
        // Set up motor speed management
        MotorSpeedManager.init();
        
        // Connect to ROS
        ROSBridge.init();
        
        // Update live motor speed display from ROS status (e.g. from potentiometer/topic)
        setInterval(() => {
            const el1 = document.getElementById('motor1LiveSpeed');
            const el2 = document.getElementById('motor2LiveSpeed');
            const speedNum = (st) => {
                if (!st || st.speed == null) return NaN;
                const v = typeof st.speed === 'number' ? st.speed : parseFloat(st.speed);
                return isNaN(v) ? NaN : v;
            };
            if (el1) {
                const st = typeof ROSBridge !== 'undefined' ? ROSBridge.getMotorStatus(1) : null;
                const v = speedNum(st);
                el1.textContent = !isNaN(v) ? `Live: ${Math.round(v)} sps` : 'Live: -- sps';
            }
            if (el2) {
                const st = typeof ROSBridge !== 'undefined' ? ROSBridge.getMotorStatus(2) : null;
                const v = speedNum(st);
                el2.textContent = !isNaN(v) ? `Live: ${Math.round(v)} sps` : 'Live: -- sps';
            }
        }, 1000);
        
        // Set up the workflow system (Pixi GPU renderer by default)
        await WorkflowManager.init();
        
        // Load workspace from storage (current workspace or default)
        await StorageManager.loadFromStorage();
        
        // Make sure the pause overlay is hidden when we start
        UIUtils.showPauseOverlay(false);
        
        // Update the workspace UI
        StorageManager.updateProjectUI();
        
        // Set up event handlers
        this.attachEventHandlers();
        
        // Sync execution state across all instances (main + remote)
        window.onExecutionStateUpdate = (state) => {
            const el = document.getElementById('executionSyncStatus');
            const playBtn = document.getElementById('playBtn');
            const stopBtn = document.getElementById('stopBtn');
            const myId = ROSBridge.getClientId();
            if (state.running) {
                if (state.clientId === myId) {
                    if (el) el.textContent = 'Running (this device)';
                    if (el) el.className = 'text-xs text-green-400';
                } else {
                    if (el) el.textContent = 'Running (another device)';
                    if (el) el.className = 'text-xs text-yellow-400';
                }
                if (playBtn) playBtn.disabled = true;
                if (stopBtn) stopBtn.disabled = false;
                // Activate playback panel on desktop when playback is running (this or another device)
                if (typeof switchBottomPanel === 'function') {
                    switchBottomPanel('playback');
                }
            } else {
                if (el) el.textContent = '';
                if (playBtn) playBtn.disabled = false;
                if (stopBtn) stopBtn.disabled = false;
                if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.updateActiveBlocksPanelFromSync) {
                    ExecutionEngine.updateActiveBlocksPanelFromSync({ blockIds: [], totalElapsed: 0, blockElapsed: {} });
                }
            }
        };
        if (ROSBridge.executionSyncState && ROSBridge.executionSyncState.running) {
            window.onExecutionStateUpdate(ROSBridge.executionSyncState);
        }

        if (typeof ControlCenter !== 'undefined') {
            ControlCenter.init();
        }
        
        // When active blocks state is received from executor (another device), update playback panel
        window.onActiveBlocksUpdate = (data) => {
            if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.updateActiveBlocksPanelFromSync) {
                ExecutionEngine.updateActiveBlocksPanelFromSync(data);
            }
        };
        
        UIUtils.log('[SYSTEM] Application initialized', 'success');
    },
    
    /**
     * Set up global event handlers
     */
    attachEventHandlers() {
        // Keyboard shortcuts
        document.addEventListener('keydown', (e) => {
            // Ctrl/Cmd + S to save
            if ((e.ctrlKey || e.metaKey) && e.key === 's') {
                e.preventDefault();
                quickSaveProject();
            }
            // Ctrl/Cmd + O to open projects
            if ((e.ctrlKey || e.metaKey) && e.key === 'o') {
                e.preventDefault();
                ProjectDialog.open();
            }
            // Escape to close dialogs (skip while capturing a wait-key binding on a block)
            if (e.key === 'Escape') {
                if (typeof BlockRenderer !== 'undefined' && BlockRenderer.isWaitKeyCapturing()) {
                    return;
                }
                ProjectDialog.close();
            }
            // Ctrl/Cmd + Z to undo
            if ((e.ctrlKey || e.metaKey) && e.key === 'z') {
                e.preventDefault();
                if (typeof UndoManager !== 'undefined' && UndoManager.undo()) {
                    // Handled
                }
            }
            // Configurable E-Stop / Start hotkeys (default Space / Enter).
            // Read fresh on each keydown so changes apply without a reload.
            // Skip while a Settings key-capture or wait-key block key-capture is in progress
            // so binding new keys doesn't also trigger E-Stop / Start / wait-key consume.
            const waitKeyCapturing = typeof BlockRenderer !== 'undefined' && BlockRenderer.isWaitKeyCapturing();
            if ((typeof SettingsManager === 'undefined' || !SettingsManager.captureField) && !waitKeyCapturing) {
                const active = document.activeElement;
                const isInput = active && (
                    active.tagName === 'INPUT' ||
                    active.tagName === 'TEXTAREA' ||
                    active.tagName === 'SELECT' ||
                    active.isContentEditable
                );

                const ctrls = (typeof SettingsManager !== 'undefined' && SettingsManager.getConfig)
                    ? SettingsManager.getConfig() : null;
                const eStopCode = (ctrls && ctrls.controls && ctrls.controls.eStop) || 'Space';
                const startCode = (ctrls && ctrls.controls && ctrls.controls.start) || 'Enter';

                if (!isInput) {
                    if (e.code === eStopCode) {
                        e.preventDefault();
                        if (typeof emergencyStop === 'function') {
                            emergencyStop();
                        }
                    } else if (e.code === startCode) {
                        e.preventDefault();
                        if (typeof startExecution === 'function') {
                            startExecution();
                        }
                    } else if (typeof ExecutionEngine !== 'undefined' &&
                        ExecutionEngine.isExecuting &&
                        !ExecutionEngine.isPaused &&
                        ExecutionEngine.tryConsumeWaitKey(e.code)) {
                        e.preventDefault();
                    }
                }
            }
        });
    }
};

/**
 * Project Dialog Manager
 */
const ProjectDialog = {
    isOpen: false,
    currentTab: 'browse',
    
    /**
     * Open the project dialog
     */
    async open() {
        const modal = document.getElementById('projectModal');
        if (modal) {
            modal.classList.add('visible');
            this.isOpen = true;
            this.switchTab('browse');
            await this.refreshProjectList();
            this.updateSavePanel();
        }
    },
    
    /**
     * Close the project dialog
     */
    close() {
        const modal = document.getElementById('projectModal');
        if (modal) {
            modal.classList.remove('visible');
            this.isOpen = false;
            this.clearError();
        }
    },
    
    /**
     * Switch between tabs
     */
    switchTab(tabName) {
        this.currentTab = tabName;
        
        // Hide all panels
        document.querySelectorAll('.project-panel').forEach(panel => {
            panel.classList.add('hidden');
        });
        
        // Remove active from all tabs
        document.querySelectorAll('.project-tab').forEach(tab => {
            tab.classList.remove('active');
        });
        
        // Show selected panel
        const panel = document.getElementById(`projectPanel-${tabName}`);
        if (panel) {
            panel.classList.remove('hidden');
        }
        
        // Activate tab
        const tab = document.getElementById(`projectTab-${tabName}`);
        if (tab) {
            tab.classList.add('active');
        }
        
        // Update content based on tab
        if (tabName === 'save') {
            this.updateSavePanel();
        }
        
        this.clearError();
    },
    
    /**
     * Refresh the project list
     */
    async refreshProjectList() {
        const listEl = document.getElementById('projectList');
        if (!listEl) return;
        
        // Show loading
        listEl.innerHTML = `
            <div class="project-loading">
                <div class="project-loading-spinner"></div>
                <span>Loading projects...</span>
            </div>
        `;
        
        try {
            const projects = await StorageManager.fetchProjects();
            this.renderProjectList(projects);
        } catch (error) {
            listEl.innerHTML = `
                <div class="project-empty">
                    <div class="project-empty-text">Failed to load projects</div>
                    <div class="project-empty-hint">${error.message}</div>
                </div>
            `;
        }
    },
    
    /**
     * Render the project list
     */
    renderProjectList(projects) {
        const listEl = document.getElementById('projectList');
        if (!listEl) return;
        
        if (!projects || projects.length === 0) {
            listEl.innerHTML = `
                <div class="project-empty">
                    <div class="project-empty-text">No projects yet</div>
                    <div class="project-empty-hint">Create your first project to get started</div>
                </div>
            `;
            return;
        }
        
        const currentProject = StorageManager.getCurrentProject();
        
        listEl.innerHTML = projects.map(project => {
            const isCurrent = currentProject && currentProject.id === project.id;
            const date = project.timestamp ? new Date(project.timestamp).toLocaleDateString() : 'Unknown';
            const time = project.timestamp ? new Date(project.timestamp).toLocaleTimeString() : '';
            
            return `
                <div class="project-item ${isCurrent ? 'current' : ''}" data-project-id="${project.id}">
                    <div class="project-item-info">
                        <div class="project-item-name">${this.escapeHtml(project.name)}</div>
                        ${project.description ? `<div class="project-item-desc">${this.escapeHtml(project.description)}</div>` : ''}
                        <div class="project-item-meta">
                            <span>${date} ${time}</span>
                            <span>${project.blockCount || 0} blocks</span>
                            <span>${project.workflowCount || 0} workflows</span>
                        </div>
                    </div>
                    <div class="project-item-actions">
                        <button class="project-item-btn load" onclick="ProjectDialog.loadProject('${project.id}')">
                            ${isCurrent ? 'CURRENT' : 'LOAD'}
                        </button>
                        <button class="project-item-btn delete" onclick="ProjectDialog.deleteProject('${project.id}', event)">
                            DELETE
                        </button>
                    </div>
                </div>
            `;
        }).join('');
    },
    
    /**
     * Filter projects by search term
     */
    filterProjects() {
        const searchInput = document.getElementById('projectSearchInput');
        const searchTerm = (searchInput?.value || '').toLowerCase();
        
        const projects = StorageManager.getAllProjects();
        const filtered = projects.filter(p => {
            return p.name.toLowerCase().includes(searchTerm) ||
                   (p.description || '').toLowerCase().includes(searchTerm);
        });
        
        this.renderProjectList(filtered);
    },
    
    /**
     * Update the save panel with current project info
     */
    updateSavePanel() {
        const currentProject = StorageManager.getCurrentProject();
        
        const currentNameEl = document.getElementById('saveCurrentName');
        const nameInput = document.getElementById('saveProjectName');
        const descInput = document.getElementById('saveProjectDescription');
        
        if (currentNameEl) {
            currentNameEl.textContent = currentProject?.name || 'No project loaded';
        }
        
        if (nameInput) {
            nameInput.value = currentProject?.name || '';
        }
        
        if (descInput) {
            descInput.value = currentProject?.description || '';
        }
    },
    
    /**
     * Create a new project
     */
    async createNewProject() {
        const nameInput = document.getElementById('newProjectName');
        const descInput = document.getElementById('newProjectDescription');
        
        const name = nameInput?.value?.trim();
        const description = descInput?.value?.trim() || '';
        
        if (!name) {
            this.showError('Please enter a project name');
            nameInput?.focus();
            return;
        }
        
        try {
            // Create new empty project
            await StorageManager.createNew(name, description);
            
            // Close dialog
            this.close();
            
            // Clear inputs
            if (nameInput) nameInput.value = '';
            if (descInput) descInput.value = '';
            
            UIUtils.log(`[PROJECT] Created new project "${name}"`, 'success');
        } catch (error) {
            this.showError(error.message);
        }
    },
    
    /**
     * Save the current project
     */
    async saveProject() {
        const nameInput = document.getElementById('saveProjectName');
        const descInput = document.getElementById('saveProjectDescription');
        
        const name = nameInput?.value?.trim();
        const description = descInput?.value?.trim() || '';
        
        if (!name) {
            this.showError('Please enter a project name');
            nameInput?.focus();
            return;
        }
        
        try {
            const success = await StorageManager.save(name, description);
            
            if (success) {
                await this.refreshProjectList();
                this.updateSavePanel();
                UIUtils.log(`[PROJECT] Saved project "${name}"`, 'success');
            }
        } catch (error) {
            this.showError(error.message);
        }
    },
    
    /**
     * Load a project
     */
    async loadProject(projectId) {
        try {
            const success = await StorageManager.load(projectId, false);
            
            if (success) {
                this.close();
            }
        } catch (error) {
            this.showError(error.message);
        }
    },
    
    /**
     * Delete a project
     */
    async deleteProject(projectId, event) {
        event?.stopPropagation();
        
        try {
            const success = await StorageManager.deleteProject(projectId);
            
            if (success) {
                await this.refreshProjectList();
            }
        } catch (error) {
            this.showError(error.message);
        }
    },
    
    /**
     * Show error message
     */
    showError(message) {
        const errorEl = document.getElementById('projectError');
        if (errorEl) {
            errorEl.textContent = message;
        }
    },
    
    /**
     * Clear error message
     */
    clearError() {
        const errorEl = document.getElementById('projectError');
        if (errorEl) {
            errorEl.textContent = '';
        }
    },
    
    /**
     * Escape HTML to prevent XSS
     */
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
};

// ============================================
// Global Functions (called from HTML)
// ============================================

function startExecution() {
    if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.isExecuting) {
        if (typeof emergencyStop === 'function') {
            emergencyStop();
        }
    }
    if (!ROSBridge.isConnected) {
        UIUtils.log('[ERROR] Not connected to ROS Bridge', 'error');
        return;
    }
    const project = StorageManager.getCurrentProject();
    const projectId = (project && project.id) ? project.id : null;
    ROSBridge.publishExecutionState(true, projectId);
    ROSBridge.startExecutionStateHeartbeat(projectId);
    ExecutionEngine.start().then(() => {
        ROSBridge.stopExecutionStateHeartbeat();
        ROSBridge.publishExecutionState(false, projectId);
    }).catch(() => {
        ROSBridge.stopExecutionStateHeartbeat();
        ROSBridge.publishExecutionState(false, projectId);
    });
}

function stopExecution() {
    if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.isExecuting) {
        ROSBridge.stopExecutionStateHeartbeat();
        ExecutionEngine.stop();
        const project = StorageManager.getCurrentProject();
        const projectId = (project && project.id) ? project.id : null;
        ROSBridge.publishExecutionState(false, projectId);
    } else {
        ROSBridge.publishStopRequest();
    }
}

/**
 * Quick save the current project
 */
async function quickSaveProject() {
    const currentProject = StorageManager.getCurrentProject();
    
    if (!currentProject?.id && !currentProject?.name) {
        // No project, open the save dialog
        ProjectDialog.open();
        ProjectDialog.switchTab('save');
        return;
    }
    
    // Save to current project
    await StorageManager.save();
}

/**
 * Open project dialog to browse/create projects
 */
function openProjectDialog() {
    ProjectDialog.open();
}

// Legacy compatibility functions
function saveCurrentWorkspace() {
    quickSaveProject();
}

function saveWorkspaceAs() {
    ProjectDialog.open();
    ProjectDialog.switchTab('save');
}

function loadConfiguration() {
    ProjectDialog.open();
    ProjectDialog.switchTab('browse');
}

function loadWorkspaceFromSelect() {
    // Legacy - no longer used
}

function deleteCurrentWorkspace() {
    const currentProject = StorageManager.getCurrentProject();
    if (currentProject?.id) {
        StorageManager.deleteProject(currentProject.id);
    } else {
        UIUtils.log('[DELETE] No project to delete', 'error');
    }
}

function newWorkspace() {
    ProjectDialog.open();
    ProjectDialog.switchTab('new');
}

function saveConfiguration() {
    quickSaveProject();
}

function clearWorkspace() {
    WorkflowManager.clearWorkspace();
}

function createNewWorkflow() {
    // Ask the user what kind of workflow they want to create
    const choice = prompt('Create new workflow:\n1 - Green Flag (starts on execution)\n2 - Workflow Complete (triggers when another workflow completes)\n\nEnter 1 or 2:');
    if (choice === '1') {
        WorkflowManager.createNewWorkflow('green-flag');
    } else if (choice === '2') {
        WorkflowManager.createNewWorkflow('workflow-complete');
    } else {
        UIUtils.log('[WORKFLOW] Workflow creation cancelled', 'info');
    }
}

function resumeSequence() {
    ExecutionEngine.resume();
}

// ── Low-Power Mode ────────────────────────────────────────────────
// Auto-enable on ARM/aarch64 devices (Raspberry Pi) or when saved
// by the user.  Can also be toggled from Settings via the global
// helper below.
(function initLowPowerMode() {
    const STORAGE_KEY = 'assemblyLineLowPower';
    const RASPI_KEY = 'assemblyLineRaspiMode';
    const params = new URLSearchParams(window.location.search || '');
    const raspiMode = params.get('raspi') === '1' ||
        window.ASSEMBLYLINE_RASPI_MODE === true ||
        localStorage.getItem(RASPI_KEY) === '1';
    const isARM = /aarch64|armv[78]|arm64/i.test(navigator.userAgent || '');
    const lowSpecDevice = (navigator.hardwareConcurrency && navigator.hardwareConcurrency <= 4) ||
        (navigator.deviceMemory && navigator.deviceMemory <= 4);
    const saved = localStorage.getItem(STORAGE_KEY);
    if (raspiMode) {
        window.ASSEMBLYLINE_RASPI_MODE = true;
        localStorage.setItem(RASPI_KEY, '1');
    }
    // Pi kiosk mode always gets the lean visual path. Non-kiosk browsers can still override.
    const enabled = raspiMode || (saved !== null ? saved === '1' : (isARM || lowSpecDevice));
    if (enabled) {
        document.documentElement.classList.add('low-power-mode');
        document.body.classList.add('low-power-mode');
        localStorage.setItem(STORAGE_KEY, '1');
    }
    if (raspiMode) {
        document.documentElement.classList.add('raspi-mode');
        document.body.classList.add('raspi-mode');
    }
})();

/**
 * Toggle or explicitly set the low-power visual mode.
 * @param {boolean|undefined} force - true = on, false = off, undefined = toggle
 */
function setLowPowerMode(force) {
    const body = document.body;
    const raspiMode = window.ASSEMBLYLINE_RASPI_MODE === true || localStorage.getItem('assemblyLineRaspiMode') === '1';
    if (raspiMode) {
        document.documentElement.classList.add('low-power-mode', 'raspi-mode');
        body.classList.add('low-power-mode', 'raspi-mode');
        localStorage.setItem('assemblyLineLowPower', '1');
        if (typeof UIUtils !== 'undefined') {
            UIUtils.log('[APP] Raspberry Pi performance mode is locked on', 'info');
        }
        return;
    }
    const active = body.classList.contains('low-power-mode');
    const next = force !== undefined ? force : !active;
    document.documentElement.classList.toggle('low-power-mode', next);
    body.classList.toggle('low-power-mode', next);
    localStorage.setItem('assemblyLineLowPower', next ? '1' : '0');
    if (typeof UIUtils !== 'undefined') {
        UIUtils.log(`[APP] Low-power mode ${next ? 'enabled' : 'disabled'}`, 'info');
    }
}

/** Sync the ECO button's visual active/inactive state. */
function updateLowPowerBtn() {
    const btn = document.getElementById('lowPowerBtn');
    if (!btn) return;
    const raspiMode = window.ASSEMBLYLINE_RASPI_MODE === true || localStorage.getItem('assemblyLineRaspiMode') === '1';
    const on = raspiMode || document.body.classList.contains('low-power-mode');
    btn.classList.toggle('btn-warning', on);
    btn.classList.toggle('btn-secondary', !on);
    btn.textContent = raspiMode ? 'PI MODE' : 'ECO';
    btn.title = raspiMode
        ? 'Raspberry Pi performance mode is locked on for this kiosk'
        : 'Toggle low-power mode (reduces animations and repaint cost)';
}

// Start everything up once the page is loaded
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => App.init());
} else {
    App.init();
}
