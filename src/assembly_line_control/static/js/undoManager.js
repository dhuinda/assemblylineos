/**
 * Undo Manager - Handles Ctrl-Z undo for workspace mutations
 * Snapshots state before mutations and restores on undo
 */
const UndoManager = {
    undoStack: [],
    maxStackSize: 50,
    
    /**
     * Push current state to undo stack (call before any mutation)
     */
    pushState() {
        try {
            const config = StorageManager.buildConfig();
            if (!config || !config.blocks) return;
            
            this.undoStack.push(config);
            
            if (this.undoStack.length > this.maxStackSize) {
                this.undoStack.shift();
            }
        } catch (error) {
            console.warn('[UNDO] Failed to push state:', error);
        }
    },
    
    /**
     * Restore previous state (Ctrl-Z)
     * @returns {boolean} - Whether undo was performed
     */
    undo() {
        if (this.undoStack.length === 0) {
            UIUtils.log('[UNDO] Nothing to undo', 'info');
            return false;
        }
        
        try {
            const config = this.undoStack.pop();
            StorageManager.applyConfig(config, { skipProjectUpdate: true });
            StorageManager.updateProjectUI();
            StorageManager.autoSave();
            UIUtils.log('[UNDO] Restored previous state', 'success');
            return true;
        } catch (error) {
            UIUtils.log('[UNDO] Failed to undo: ' + error.message, 'error');
            console.error('Undo error:', error);
            return false;
        }
    },
    
    /**
     * Clear undo stack (e.g. when loading a new project)
     */
    clear() {
        this.undoStack = [];
    }
};
