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

    _isPixiMode() {
        return typeof WorkflowManager !== 'undefined' && WorkflowManager.isPixiMode();
    },

    _blockIdFromKey(blockKey) {
        const idx = blockKey.lastIndexOf('-');
        if (idx < 0) return null;
        const id = Number(blockKey.slice(idx + 1));
        return Number.isFinite(id) ? id : null;
    },

    _activeBlockIds() {
        const ids = new Set();
        this.activeBlocks.forEach(key => {
            const id = this._blockIdFromKey(key);
            if (id != null) ids.add(id);
        });
        return ids;
    },

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

        this.buildExecutionTimeline();

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
        this.activeBlocks.clear();
        if (this._isPixiMode()) {
            PixiWorkspaceRenderer.clearAllExecuting();
        } else {
            this.updateBlockVisualStates(0);
        }
        UIUtils.log('[PLAYBACK] Stopped');
    },

    /**
     * Update playback animation
     */
    update() {
        if (!this.isPlaying) return;

        const now = performance.now() / 1000;
        this.currentPlaybackTime = now - this.playbackStartTime;

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

            if (orderItem.endTime !== undefined && orderItem.endTime !== null) {
                this.executionTimeline.push({
                    time: orderItem.endTime,
                    type: 'end',
                    lane: orderItem.lane,
                    blockId: orderItem.block
                });
            } else {
                this.executionTimeline.push({
                    time: orderItem.startTime + 0.01,
                    type: 'end',
                    lane: orderItem.lane,
                    blockId: orderItem.block
                });
            }
        });

        this.executionTimeline.sort((a, b) => a.time - b.time);
    },

    /**
     * Update block visual states
     * @param {number} time - Current playback time
     */
    updateBlockVisualStates(time) {
        if (this._isPixiMode()) {
            PixiWorkspaceRenderer.setExecutingBlocks(this._activeBlockIds());
            return;
        }

        document.querySelectorAll('.sequence-block.executing').forEach(block => {
            block.classList.remove('executing');
        });
        document.querySelectorAll('.lane.executing').forEach(lane => {
            lane.classList.remove('executing');
        });

        this.activeBlocks.forEach(blockKey => {
            const idx = blockKey.lastIndexOf('-');
            if (idx < 0) return;
            const laneIndex = Number(blockKey.slice(0, idx));
            const blockId = Number(blockKey.slice(idx + 1));
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
