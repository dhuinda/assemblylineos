/**
 * Playback Engine - Handles visual playback of sequences
 */
const PlaybackEngine = {
    isPlaying: false,
    playbackStartTime: 0,
    playbackAnimationFrame: null,
    currentPlaybackTime: 0,
    executionTimeline: [],
    activeBlocks: new Set(),
    
    /**
     * Toggle playback
     */
    toggle() {
        if (this.isPlaying) {
            this.pause();
        } else {
            this.start();
        }
    },
    
    /**
     * Start playback
     */
    start() {
        if (this.isPlaying) return;
        
        // Build timeline
        this.buildExecutionTimeline();
        
        // Check if we have any blocks at all
        const totalBlocks = WorkflowManager.blocks.size;
        if (totalBlocks === 0) {
            UIUtils.log('[ERROR] No blocks in workspace. Add blocks before playing.', 'error');
            return;
        }
        
        if (this.executionTimeline.length === 0) {
            UIUtils.log('[ERROR] No execution timeline found. Make sure workflows have blocks.', 'error');
            return;
        }
        
        this.isPlaying = true;
        this.playbackStartTime = performance.now() / 1000 - this.currentPlaybackTime;
        UIUtils.updatePlaybackButton(true);
        
        this.update();
        UIUtils.log('[PLAYBACK] Started');
    },
    
    /**
     * Pause playback
     */
    pause() {
        if (!this.isPlaying) return;
        
        this.isPlaying = false;
        if (this.playbackAnimationFrame) {
            cancelAnimationFrame(this.playbackAnimationFrame);
            this.playbackAnimationFrame = null;
        }
        UIUtils.updatePlaybackButton(false);
        
        UIUtils.log('[PLAYBACK] Paused');
    },
    
    /**
     * Stop playback
     */
    stop() {
        this.isPlaying = false;
        this.currentPlaybackTime = 0;
        if (this.playbackAnimationFrame) {
            cancelAnimationFrame(this.playbackAnimationFrame);
            this.playbackAnimationFrame = null;
        }
        UIUtils.updatePlaybackButton(false);
        ActiveBlocksPanel.update(0);
        this.updateBlockVisualStates(0);
        UIUtils.log('[PLAYBACK] Stopped');
    },
    
    /**
     * Update playback animation
     */
    update() {
        if (!this.isPlaying) return;
        
        const now = performance.now() / 1000;
        this.currentPlaybackTime = now - this.playbackStartTime;
        
        // Check if we've reached the end - find the maximum endTime
        let maxTime = 0;
        this.executionTimeline.forEach(event => {
            if (event.time > maxTime) {
                maxTime = event.time;
            }
        });
        
        if (this.currentPlaybackTime >= maxTime) {
            this.stop();
            return;
        }
        
        ActiveBlocksPanel.update(this.currentPlaybackTime);
        this.updateBlockVisualStates(this.currentPlaybackTime);
        
        this.playbackAnimationFrame = requestAnimationFrame(() => this.update());
    },
    
    /**
     * Build execution timeline for playback
     */
    buildExecutionTimeline() {
        this.executionTimeline = [];
        const executionOrder = ExecutionEngine.buildExecutionOrder();
        
        executionOrder.forEach(orderItem => {
            this.executionTimeline.push({
                time: orderItem.startTime,
                type: 'start',
                lane: orderItem.lane,
                blockId: orderItem.block
            });
            
            // Always add end event, even for instant blocks (they get 0.01s duration)
            if (orderItem.endTime !== undefined && orderItem.endTime !== null) {
                this.executionTimeline.push({
                    time: orderItem.endTime,
                    type: 'end',
                    lane: orderItem.lane,
                    blockId: orderItem.block
                });
            } else {
                // Fallback: if no endTime, use startTime + small duration
                this.executionTimeline.push({
                    time: orderItem.startTime + 0.01,
                    type: 'end',
                    lane: orderItem.lane,
                    blockId: orderItem.block
                });
            }
        });
        
        // Sort by time
        this.executionTimeline.sort((a, b) => a.time - b.time);
    },
    
    /**
     * Update block visual states
     * @param {number} time - Current playback time
     */
    updateBlockVisualStates(time) {
        // Remove executing state from all blocks
        document.querySelectorAll('.sequence-block.executing').forEach(block => {
            block.classList.remove('executing');
        });
        document.querySelectorAll('.lane.executing').forEach(lane => {
            lane.classList.remove('executing');
        });
        
        // Add executing state to active blocks
        this.activeBlocks.forEach(blockKey => {
            const [laneIndex, blockId] = blockKey.split('-').map(Number);
            const blockEl = document.querySelector(`[data-block-id="${blockId}"][data-lane-index="${laneIndex}"]`);
            if (blockEl) {
                blockEl.classList.add('executing');
                const laneEl = blockEl.closest('.lane');
                if (laneEl) {
                    laneEl.classList.add('executing');
                }
            }
        });
    }
};

