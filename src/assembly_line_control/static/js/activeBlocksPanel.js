/**
 * Active Blocks Panel - Displays currently active blocks during playback
 */
const ActiveBlocksPanel = {
    /**
     * Update the active blocks display
     * @param {number} time - Current playback time
     */
    update(time) {
        const activePanel = document.getElementById('activeBlocksPanel');
        if (!activePanel) return;
        
        UIUtils.updateCurrentTime(time);
        
        // Find active blocks at this time
        PlaybackEngine.activeBlocks.clear();
        
        // Group events by block
        const blockEvents = new Map();
        PlaybackEngine.executionTimeline.forEach(event => {
            const key = `${event.lane}-${event.blockId}`;
            if (!blockEvents.has(key)) {
                blockEvents.set(key, []);
            }
            blockEvents.get(key).push(event);
        });
        
        // Check each block to see if it's active at this time
        blockEvents.forEach((events, blockKey) => {
            // Sort events by time
            events.sort((a, b) => a.time - b.time);
            
            // Find the most recent start event before or at this time
            let latestStart = null;
            for (let i = events.length - 1; i >= 0; i--) {
                if (events[i].type === 'start' && events[i].time <= time) {
                    latestStart = events[i];
                    break;
                }
            }
            
            if (latestStart) {
                // Check if there's an end event after the start but before current time
                const endEvent = events.find(e => 
                    e.type === 'end' && 
                    e.time > latestStart.time && 
                    e.time <= time
                );
                
                // Block is active if there's no end event before current time
                if (!endEvent) {
                    PlaybackEngine.activeBlocks.add(blockKey);
                }
            }
        });
        
        if (PlaybackEngine.activeBlocks.size === 0) {
            activePanel.innerHTML = '<p class="text-xs text-gray-500 text-center py-4">No blocks active</p>';
            return;
        }
        
        activePanel.innerHTML = '';
        // Note: This panel is updated by ExecutionEngine.updateActiveBlocksPanel() 
        // which uses WorkflowManager, not PlaybackEngine
        // This function is kept for backward compatibility but may not be used
    }
};

