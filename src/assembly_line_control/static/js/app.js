/**
 * Main Application - Sets up and coordinates everything
 */
const App = {
    /**
     * Get everything started up
     */
    init() {
        // Set up motor speed management
        MotorSpeedManager.init();
        
        // Connect to ROS
        ROSBridge.init();
        
        // Set up the workflow system
        WorkflowManager.init();
        
        // Load any saved workflows
        WorkflowManager.initialize();
        
        // Load workspace from storage (current workspace or default)
        StorageManager.loadFromStorage();
        
        // Make sure the pause overlay is hidden when we start
        UIUtils.showPauseOverlay(false);
        
        // Update the workspace UI
        StorageManager.updateWorkspaceUI();
        
        // Set up event handlers
        this.attachEventHandlers();
        
        UIUtils.log('[SYSTEM] Application initialized', 'success');
    },
    
    /**
     * Set up global event handlers
     */
    attachEventHandlers() {
        // The resume button is handled directly in the overlay HTML
        // The overlay gets created dynamically when we pause
    }
};

// These functions are called directly from the HTML buttons
function startExecution() {
    ExecutionEngine.start();
}

function stopExecution() {
    ExecutionEngine.stop();
}

function saveCurrentWorkspace() {
    const currentName = StorageManager.getCurrentWorkspace();
    if (!currentName) {
        saveWorkspaceAs();
    } else {
        StorageManager.save();
    }
}

function saveWorkspaceAs() {
    const name = prompt('Enter workspace name:');
    if (name && name.trim() !== '') {
        StorageManager.saveAs(name.trim());
    }
}

function loadConfiguration() {
    const workspaces = StorageManager.getAllWorkspaces();
    const names = Object.keys(workspaces).sort();
    
    if (names.length === 0) {
        UIUtils.log('[LOAD] No workspaces available', 'error');
        return;
    }
    
    let message = 'Select workspace to load:\n\n';
    names.forEach((name, index) => {
        message += `${index + 1}. ${name}\n`;
    });
    
    const choice = prompt(message + '\nEnter number or name:');
    if (!choice) return;
    
    // First try to interpret it as a number
    const num = parseInt(choice);
    if (!isNaN(num) && num >= 1 && num <= names.length) {
        StorageManager.load(names[num - 1]);
    } else {
        // If that didn't work, try it as a name
        if (names.includes(choice)) {
            StorageManager.load(choice);
        } else {
            UIUtils.log('[LOAD] Invalid workspace name', 'error');
        }
    }
}

function loadWorkspaceFromSelect() {
    const select = document.getElementById('workspaceList');
    if (select && select.value) {
        StorageManager.load(select.value, true);
    }
}

function deleteCurrentWorkspace() {
    const currentName = StorageManager.getCurrentWorkspace();
    if (!currentName) {
        UIUtils.log('[DELETE] No workspace selected', 'error');
        return;
    }
    
    if (StorageManager.deleteWorkspace(currentName)) {
        // Clear workspace after deletion
        WorkflowManager.blocks.clear();
        WorkflowManager.workflows.clear();
        BlockConnector.connections.clear();
        WorkflowManager.blockIdCounter = 0;
        WorkflowManager.workflowIdCounter = 0;
        WorkflowManager.renderAll();
    }
}

function newWorkspace() {
    if (!UIUtils.confirm('Create a new workspace? Current unsaved changes will be lost.')) {
        return;
    }
    
    // Clear current workspace
    StorageManager.setCurrentWorkspace(null);
    WorkflowManager.blocks.clear();
    WorkflowManager.workflows.clear();
    BlockConnector.connections.clear();
    WorkflowManager.blockIdCounter = 0;
    WorkflowManager.workflowIdCounter = 0;
    WorkflowManager.renderAll();
    
    StorageManager.updateWorkspaceUI();
    UIUtils.log('[WORKSPACE] New workspace created', 'success');
}

// Old name for saveCurrentWorkspace - kept for backward compatibility
function saveConfiguration() {
    saveCurrentWorkspace();
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

// Start everything up once the page is loaded
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => App.init());
} else {
    App.init();
}
