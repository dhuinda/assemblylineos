/**
 * UI Utilities - Logging, status updates, and UI helpers
 */
const UIUtils = {
    /**
     * Log a message to the system log
     * @param {string} message - The message to log
     * @param {string} type - Log type: 'info', 'error', 'success', 'warning'
     */
    log(message, type = 'info') {
        const logArea = document.getElementById('logArea');
        if (!logArea) {
            console.log(`[${type.toUpperCase()}] ${message}`);
            return;
        }
        
        const logEntry = document.createElement('p');
        
        let color = 'text-gray-400';
        if (type === 'error') color = 'text-red-400';
        if (type === 'success') color = 'text-green-400';
        if (type === 'warning') color = 'text-yellow-400';
        
        logEntry.className = color;
        logEntry.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
        logArea.appendChild(logEntry);
        logArea.scrollTop = logArea.scrollHeight;
    },
    
    /**
     * Update ROS connection status display
     * @param {boolean} connected - Whether ROS is connected
     */
    updateRosStatus(connected) {
        const statusEl = document.getElementById('rosStatus');
        if (!statusEl) return;
        
        if (connected) {
            statusEl.textContent = '● CONNECTED';
            statusEl.className = 'text-sm status-connected font-medium';
        } else {
            statusEl.textContent = '● DISCONNECTED';
            statusEl.className = 'text-sm status-disconnected font-medium';
        }
    },
    
    /**
     * Update Arduino connection status display
     * @param {boolean} connected - Whether Arduino is connected
     * @param {string} port - The port Arduino is connected on (if connected)
     */
    updateArduinoStatus(connected, port = null) {
        const statusEl = document.getElementById('arduinoStatus');
        if (!statusEl) return;
        
        if (connected) {
            const portInfo = port ? ` (${port.split('/').pop()})` : '';
            statusEl.textContent = `● ARDUINO${portInfo}`;
            statusEl.className = 'text-xs status-connected font-medium';
            statusEl.title = `Connected to Arduino on ${port || 'unknown port'}`;
        } else {
            statusEl.textContent = '● ARDUINO OFFLINE';
            statusEl.className = 'text-xs status-disconnected font-medium';
            statusEl.title = 'Arduino not connected - check USB connection';
        }
    },
    
    /**
     * Show a confirmation dialog
     * @param {string} message - The confirmation message
     * @returns {boolean} - User's choice
     */
    confirm(message) {
        return window.confirm(message);
    },
    
    /**
     * Show an alert dialog
     * @param {string} message - The alert message
     */
    alert(message) {
        window.alert(message);
    },
    
    /**
     * Update playback button state
     * @param {boolean} isPlaying - Whether playback is active
     */
    updatePlaybackButton(isPlaying) {
        const playBtn = document.getElementById('playBtn');
        if (playBtn) {
            playBtn.textContent = isPlaying ? '⏸ PAUSE' : '▶ PLAY';
        }
    },
    
    /**
     * Show/hide resume button (legacy - kept for backward compatibility)
     * @param {boolean} visible - Whether to show the button
     */
    showResumeButton(visible) {
        // This is now handled by showPauseOverlay
        this.showPauseOverlay(visible);
    },
    
    /**
     * Show/hide pause overlay with centered resume button
     * Overlay only covers the workspace canvas (designer), not the entire screen
     * @param {boolean} visible - Whether to show the overlay
     */
    showPauseOverlay(visible) {
        // Find the workspace canvas element
        const workspaceCanvas = document.getElementById('workspaceCanvas');
        if (!workspaceCanvas) {
            console.warn('[UIUtils] Workspace canvas not found');
            return;
        }
        
        // Ensure the canvas is positioned relatively so the absolute overlay works
        if (getComputedStyle(workspaceCanvas).position === 'static') {
            workspaceCanvas.style.position = 'relative';
        }
        
        let overlay = document.getElementById('pauseOverlay');
        
        if (visible) {
            // Create overlay if it doesn't exist
            if (!overlay) {
                overlay = document.createElement('div');
                overlay.id = 'pauseOverlay';
                overlay.className = 'pause-overlay';
                overlay.innerHTML = `
                    <div class="pause-overlay-content">
                        <div class="pause-overlay-title">PAUSED</div>
                        <button id="pauseOverlayResumeBtn" class="pause-overlay-resume-btn">
                            ▶ RESUME
                        </button>
                    </div>
                `;
                // Append directly to workspace canvas so it only covers the designer area
                workspaceCanvas.appendChild(overlay);
            } else {
                // If overlay exists but is in the wrong place, move it to the workspace canvas
                if (overlay.parentElement !== workspaceCanvas) {
                    workspaceCanvas.appendChild(overlay);
                }
            }
            
            // Always ensure the resume button handler is attached (in case overlay was reused)
            const resumeBtn = overlay.querySelector('#pauseOverlayResumeBtn');
            if (resumeBtn) {
                // Remove any existing onclick handlers
                resumeBtn.onclick = null;
                // Clone and replace to remove all event listeners
                const newBtn = resumeBtn.cloneNode(true);
                resumeBtn.parentNode.replaceChild(newBtn, resumeBtn);
                
                // Attach reliable event handler
                newBtn.addEventListener('click', function(e) {
                    e.preventDefault();
                    e.stopPropagation();
                    // Directly call ExecutionEngine.resume() to ensure it always works
                    if (typeof ExecutionEngine !== 'undefined' && ExecutionEngine.resume) {
                        ExecutionEngine.resume();
                    } else {
                        // Fallback to global function
                        if (typeof resumeSequence === 'function') {
                            resumeSequence();
                        }
                    }
                }, { once: false });
            }
            
            overlay.style.display = 'flex';
        } else {
            // Hide overlay if it exists
            if (overlay) {
                overlay.style.display = 'none';
            }
        }
        
        // Also hide the old resume button in the header
        const resumeBtn = document.getElementById('resumeBtn');
        if (resumeBtn) {
            resumeBtn.style.display = 'none';
            resumeBtn.classList.remove('visible');
        }
    },
    
    /**
     * Update current time display
     * @param {number} time - Time in seconds
     */
    updateCurrentTime(time) {
        const currentTimeEl = document.getElementById('currentTime');
        if (currentTimeEl) {
            currentTimeEl.textContent = time.toFixed(2);
        }
    },
    
    /**
     * Add visual error state to a block element
     * @param {HTMLElement} blockEl - The block element
     */
    markBlockError(blockEl) {
        if (blockEl) {
            blockEl.classList.add('error');
            blockEl.classList.remove('warning');
        }
    },
    
    /**
     * Add visual warning state to a block element
     * @param {HTMLElement} blockEl - The block element
     */
    markBlockWarning(blockEl) {
        if (blockEl) {
            blockEl.classList.add('warning');
            blockEl.classList.remove('error');
        }
    },
    
    /**
     * Clear visual error/warning states from a block element
     * @param {HTMLElement} blockEl - The block element
     */
    clearBlockStates(blockEl) {
        if (blockEl) {
            blockEl.classList.remove('error', 'warning');
        }
    }
};

