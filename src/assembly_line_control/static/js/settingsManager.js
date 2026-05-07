/**
 * Settings Manager - Handles Arduino pin configuration
 * 
 * Manages the settings dialog for configuring motor pins, relay pins,
 * and custom pins. Configuration is persisted on the server and sent
 * to the Arduino when serial connection is established.
 */
const SettingsManager = {
    // Default pin configuration
    defaultConfig: {
        motors: [
            { id: 1, step_pin: 2, dir_pin: 3, invert_direction: false },
            { id: 2, step_pin: 5, dir_pin: 6, invert_direction: false }
        ],
        relays: [
            { id: 1, pin: 54 },  // A0 on Arduino Giga
            { id: 2, pin: 55 },  // A1
            { id: 3, pin: 56 },  // A2
            { id: 4, pin: 57 }   // A3
        ],
        custom: [],
        controls: { eStop: 'Space', start: 'Enter' }
    },
    
    // Current configuration
    config: null,
    
    // Custom pin counter for unique IDs
    customPinCounter: 0,

    // Active key-capture field name when remapping a control, or null
    captureField: null,
    _captureKeyHandler: null,
    
    /**
     * Initialize the settings manager
     */
    init() {
        this.config = JSON.parse(JSON.stringify(this.defaultConfig));
        this.loadSettings();
    },
    
    /**
     * Open the settings dialog
     */
    openDialog() {
        const modal = document.getElementById('settingsModal');
        if (modal) {
            // Load current settings first
            this.loadSettings().then(() => {
                this.populateForm();
                this.updateArduinoStatus();
                modal.classList.add('visible');
            }).catch(() => {
                // Use defaults if load fails
                this.config = JSON.parse(JSON.stringify(this.defaultConfig));
                this.populateForm();
                this.updateArduinoStatus();
                modal.classList.add('visible');
            });
        }
    },
    
    /**
     * Close the settings dialog
     */
    closeDialog() {
        const modal = document.getElementById('settingsModal');
        if (modal) {
            modal.classList.remove('visible');
            this.clearError();
            this.stopCapture();
        }
    },
    
    /**
     * Populate the form with current configuration
     */
    populateForm() {
        // Motor pins
        for (let i = 1; i <= 2; i++) {
            const motor = this.config.motors.find(m => m.id === i);
            if (motor) {
                const stepInput = document.getElementById(`motor${i}StepPin`);
                const dirInput = document.getElementById(`motor${i}DirPin`);
                const invertInput = document.getElementById(`motor${i}InvertDir`);
                if (stepInput) stepInput.value = motor.step_pin;
                if (dirInput) dirInput.value = motor.dir_pin;
                if (invertInput) invertInput.checked = motor.invert_direction === true;
            }
        }
        
        // Relay pins
        for (let i = 1; i <= 4; i++) {
            const relay = this.config.relays.find(r => r.id === i);
            if (relay) {
                const pinInput = document.getElementById(`relay${i}Pin`);
                if (pinInput) pinInput.value = relay.pin;
            }
        }
        
        // Custom pins
        const container = document.getElementById('customPinsContainer');
        if (container) {
            container.innerHTML = '';
            this.customPinCounter = 0;
            this.config.custom.forEach(pin => {
                this.addCustomPinRow(pin.name, pin.pin, pin.mode);
            });
        }

        // Controls (keyboard hotkeys)
        this.stopCapture();
        const controls = this.config.controls || this.defaultConfig.controls;
        this.applyControlToButton('eStop', controls.eStop || this.defaultConfig.controls.eStop);
        this.applyControlToButton('start', controls.start || this.defaultConfig.controls.start);
    },

    /**
     * Update a control capture button's keycode and visible label
     */
    applyControlToButton(field, code) {
        const btn = document.getElementById(field === 'eStop' ? 'controlEStopCapture' : 'controlStartCapture');
        if (!btn) return;
        btn.setAttribute('data-keycode', code);
        btn.textContent = this.keyCodeToLabel(code);
    },
    
    /**
     * Add a custom pin row to the form
     */
    addCustomPin() {
        this.addCustomPinRow('', '', 'input');
    },
    
    /**
     * Add a custom pin row with optional default values
     */
    addCustomPinRow(name = '', pin = '', mode = 'input') {
        const container = document.getElementById('customPinsContainer');
        if (!container) return;
        
        const rowId = `customPin_${this.customPinCounter++}`;
        const row = document.createElement('div');
        row.className = 'custom-pin-row';
        row.id = rowId;
        row.innerHTML = `
            <div class="custom-pin-name">
                <input type="text" class="settings-input" placeholder="Pin name" value="${this.escapeHtml(name)}" data-field="name">
            </div>
            <div class="custom-pin-number">
                <input type="number" class="settings-input" placeholder="Pin #" min="0" max="99" value="${pin}" data-field="pin">
            </div>
            <div class="custom-pin-mode">
                <select data-field="mode">
                    <option value="input" ${mode === 'input' ? 'selected' : ''}>Input</option>
                    <option value="output" ${mode === 'output' ? 'selected' : ''}>Output</option>
                    <option value="input_pullup" ${mode === 'input_pullup' ? 'selected' : ''}>Input Pullup</option>
                </select>
            </div>
            <button class="custom-pin-remove" onclick="SettingsManager.removeCustomPin('${rowId}')" title="Remove">&times;</button>
        `;
        container.appendChild(row);
    },
    
    /**
     * Remove a custom pin row
     */
    removeCustomPin(rowId) {
        const row = document.getElementById(rowId);
        if (row) {
            row.remove();
        }
    },
    
    /**
     * Collect form data into configuration object
     */
    collectFormData() {
        const config = {
            motors: [],
            relays: [],
            custom: [],
            controls: { ...this.defaultConfig.controls }
        };
        
        // Motor pins
        for (let i = 1; i <= 2; i++) {
            const stepInput = document.getElementById(`motor${i}StepPin`);
            const dirInput = document.getElementById(`motor${i}DirPin`);
            const invertInput = document.getElementById(`motor${i}InvertDir`);
            config.motors.push({
                id: i,
                step_pin: parseInt(stepInput?.value) || 0,
                dir_pin: parseInt(dirInput?.value) || 0,
                invert_direction: invertInput?.checked === true
            });
        }
        
        // Relay pins
        for (let i = 1; i <= 4; i++) {
            const pinInput = document.getElementById(`relay${i}Pin`);
            config.relays.push({
                id: i,
                pin: parseInt(pinInput?.value) || 0
            });
        }
        
        // Custom pins
        const container = document.getElementById('customPinsContainer');
        if (container) {
            const rows = container.querySelectorAll('.custom-pin-row');
            rows.forEach(row => {
                const nameInput = row.querySelector('[data-field="name"]');
                const pinInput = row.querySelector('[data-field="pin"]');
                const modeSelect = row.querySelector('[data-field="mode"]');
                
                const name = nameInput?.value?.trim() || '';
                const pin = parseInt(pinInput?.value) || 0;
                const mode = modeSelect?.value || 'input';
                
                // Only add if name and pin are provided
                if (name && pin > 0) {
                    config.custom.push({ name, pin, mode });
                }
            });
        }

        // Controls (keyboard hotkeys)
        const eStopBtn = document.getElementById('controlEStopCapture');
        const startBtn = document.getElementById('controlStartCapture');
        if (eStopBtn) {
            const code = eStopBtn.getAttribute('data-keycode');
            if (code) config.controls.eStop = code;
        }
        if (startBtn) {
            const code = startBtn.getAttribute('data-keycode');
            if (code) config.controls.start = code;
        }

        return config;
    },
    
    /**
     * Validate the configuration
     * @returns {string|null} Error message or null if valid
     */
    validateConfig(config) {
        const usedPins = new Map();
        
        // Check motor pins
        for (const motor of config.motors) {
            if (motor.step_pin < 0 || motor.step_pin > 99) {
                return `Motor ${motor.id} STEP pin must be between 0 and 99`;
            }
            if (motor.dir_pin < 0 || motor.dir_pin > 99) {
                return `Motor ${motor.id} DIR pin must be between 0 and 99`;
            }
            
            // Check for duplicates
            const stepKey = `pin_${motor.step_pin}`;
            const dirKey = `pin_${motor.dir_pin}`;
            
            if (usedPins.has(stepKey)) {
                return `Pin ${motor.step_pin} is used multiple times (Motor ${motor.id} STEP and ${usedPins.get(stepKey)})`;
            }
            usedPins.set(stepKey, `Motor ${motor.id} STEP`);
            
            if (usedPins.has(dirKey)) {
                return `Pin ${motor.dir_pin} is used multiple times (Motor ${motor.id} DIR and ${usedPins.get(dirKey)})`;
            }
            usedPins.set(dirKey, `Motor ${motor.id} DIR`);
        }
        
        // Check relay pins
        for (const relay of config.relays) {
            if (relay.pin < 0 || relay.pin > 99) {
                return `Relay ${relay.id} pin must be between 0 and 99`;
            }
            
            const key = `pin_${relay.pin}`;
            if (usedPins.has(key)) {
                return `Pin ${relay.pin} is used multiple times (Relay ${relay.id} and ${usedPins.get(key)})`;
            }
            usedPins.set(key, `Relay ${relay.id}`);
        }
        
        // Check custom pins
        const customNames = new Set();
        for (const custom of config.custom) {
            if (!custom.name || custom.name.trim() === '') {
                return 'Custom pin name cannot be empty';
            }
            if (customNames.has(custom.name.toLowerCase())) {
                return `Duplicate custom pin name: ${custom.name}`;
            }
            customNames.add(custom.name.toLowerCase());
            
            if (custom.pin < 0 || custom.pin > 99) {
                return `Custom pin "${custom.name}" must be between 0 and 99`;
            }
            
            const key = `pin_${custom.pin}`;
            if (usedPins.has(key)) {
                return `Pin ${custom.pin} is used multiple times (${custom.name} and ${usedPins.get(key)})`;
            }
            usedPins.set(key, custom.name);
        }

        // Controls must have non-empty key codes (no pin uniqueness check)
        if (config.controls) {
            if (typeof config.controls.eStop !== 'string' || config.controls.eStop.trim() === '') {
                return 'E-Stop hotkey is not set';
            }
            if (typeof config.controls.start !== 'string' || config.controls.start.trim() === '') {
                return 'Start hotkey is not set';
            }
        }

        return null;
    },
    
    /**
     * Show error message
     */
    showError(message) {
        const errorDiv = document.getElementById('settingsError');
        if (errorDiv) {
            errorDiv.textContent = message;
        }
    },
    
    /**
     * Clear error message
     */
    clearError() {
        const errorDiv = document.getElementById('settingsError');
        if (errorDiv) {
            errorDiv.textContent = '';
        }
    },
    
    /**
     * Load settings from the server
     */
    async loadSettings() {
        try {
            const response = await fetch('/api/settings');
            if (response.ok) {
                const data = await response.json();
                if (data && data.motors) {
                    this.config = data;
                    console.log('[Settings] Loaded configuration:', this.config);
                }
            } else if (response.status === 404) {
                // No settings saved yet, use defaults
                console.log('[Settings] No saved configuration, using defaults');
                this.config = JSON.parse(JSON.stringify(this.defaultConfig));
            } else {
                throw new Error(`Server returned ${response.status}`);
            }
        } catch (error) {
            console.error('[Settings] Error loading settings:', error);
            this.config = JSON.parse(JSON.stringify(this.defaultConfig));
        }

        // Backfill controls if missing (older configs without keybindings)
        if (!this.config) {
            this.config = JSON.parse(JSON.stringify(this.defaultConfig));
        }
        if (!this.config.controls || typeof this.config.controls !== 'object') {
            this.config.controls = { ...this.defaultConfig.controls };
        } else {
            if (!this.config.controls.eStop) {
                this.config.controls.eStop = this.defaultConfig.controls.eStop;
            }
            if (!this.config.controls.start) {
                this.config.controls.start = this.defaultConfig.controls.start;
            }
        }
    },
    
    /**
     * Save settings to the server
     */
    async saveSettings() {
        this.clearError();
        
        // Collect and validate form data
        const config = this.collectFormData();
        const error = this.validateConfig(config);
        
        if (error) {
            this.showError(error);
            return;
        }
        
        try {
            const response = await fetch('/api/settings', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                },
                body: JSON.stringify(config)
            });
            
            if (response.ok) {
                this.config = config;
                this.closeDialog();
                
                // Log success
                if (typeof UIUtils !== 'undefined') {
                    UIUtils.log('[SETTINGS] Pin configuration saved successfully', 'success');
                }
                
                console.log('[Settings] Configuration saved:', config);
            } else {
                const data = await response.json();
                throw new Error(data.error || 'Failed to save settings');
            }
        } catch (error) {
            console.error('[Settings] Error saving settings:', error);
            this.showError(error.message || 'Failed to save settings');
        }
    },
    
    /**
     * Escape HTML special characters
     */
    escapeHtml(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    },
    
    /**
     * Get current configuration
     */
    getConfig() {
        return this.config;
    },
    
    /**
     * Update Arduino connection status in the settings dialog
     */
    updateArduinoStatus() {
        const statusEl = document.getElementById('settingsArduinoStatus');
        const reconnectBtn = document.getElementById('reconnectArduinoBtn');
        
        if (!statusEl) return;
        
        // Check ROSBridge connection status and Arduino status
        if (typeof ROSBridge !== 'undefined' && ROSBridge.isConnected) {
            // Check for Arduino status from ROSBridge (if available)
            // For now, we'll show ROS connection status
            statusEl.textContent = '● Connected to ROS';
            statusEl.className = 'text-xs font-medium text-green-400';
            if (reconnectBtn) {
                reconnectBtn.disabled = false;
                reconnectBtn.classList.remove('opacity-50');
            }
        } else {
            statusEl.textContent = '● ROS Disconnected';
            statusEl.className = 'text-xs font-medium text-red-400';
            if (reconnectBtn) {
                reconnectBtn.disabled = true;
                reconnectBtn.classList.add('opacity-50');
            }
        }
    },
    
    /**
     * Begin capturing a key for the given control field ('eStop' or 'start').
     * The next keydown sets the binding; Escape cancels.
     */
    startCapture(field) {
        if (field !== 'eStop' && field !== 'start') return;

        // Cancel any existing capture before starting a new one
        this.stopCapture();

        const btn = document.getElementById(field === 'eStop' ? 'controlEStopCapture' : 'controlStartCapture');
        if (!btn) return;

        this.captureField = field;
        btn.classList.add('capturing');
        btn.textContent = 'Press any key...';

        const handler = (e) => {
            // Ignore lone modifier presses so users can build modifier combos later if desired
            if (e.key === 'Shift' || e.key === 'Control' || e.key === 'Alt' || e.key === 'Meta') {
                return;
            }
            e.preventDefault();
            e.stopPropagation();

            if (e.key === 'Escape') {
                // Cancel capture without changing binding
                this.stopCapture();
                this.applyControlToButton(field, btn.getAttribute('data-keycode') || this.defaultConfig.controls[field]);
                return;
            }

            const code = e.code || e.key;
            this.applyControlToButton(field, code);
            this.stopCapture();
        };

        this._captureKeyHandler = handler;
        // Capture-phase listener so global hotkeys don't fire first
        document.addEventListener('keydown', handler, true);
    },

    /**
     * Stop key capture and clear capture UI state.
     */
    stopCapture() {
        if (this._captureKeyHandler) {
            document.removeEventListener('keydown', this._captureKeyHandler, true);
            this._captureKeyHandler = null;
        }
        this.captureField = null;
        const eStopBtn = document.getElementById('controlEStopCapture');
        const startBtn = document.getElementById('controlStartCapture');
        if (eStopBtn) {
            eStopBtn.classList.remove('capturing');
            const code = eStopBtn.getAttribute('data-keycode');
            if (code) eStopBtn.textContent = this.keyCodeToLabel(code);
        }
        if (startBtn) {
            startBtn.classList.remove('capturing');
            const code = startBtn.getAttribute('data-keycode');
            if (code) startBtn.textContent = this.keyCodeToLabel(code);
        }
    },

    /**
     * Reset a control binding to its default key.
     */
    resetControl(field) {
        if (field !== 'eStop' && field !== 'start') return;
        this.stopCapture();
        this.applyControlToButton(field, this.defaultConfig.controls[field]);
    },

    /**
     * Convert a KeyboardEvent.code into a human-readable label.
     */
    keyCodeToLabel(code) {
        if (!code) return '';
        if (code === 'Space') return 'Space';
        if (code === 'Enter') return 'Enter';
        if (code === 'Escape') return 'Esc';
        if (code === 'Tab') return 'Tab';
        if (code === 'Backspace') return 'Backspace';
        if (code === 'Delete') return 'Delete';
        if (code === 'ArrowUp') return '↑';
        if (code === 'ArrowDown') return '↓';
        if (code === 'ArrowLeft') return '←';
        if (code === 'ArrowRight') return '→';
        if (code.startsWith('Key')) return code.slice(3);
        if (code.startsWith('Digit')) return code.slice(5);
        if (code.startsWith('Numpad')) return 'Num ' + code.slice(6);
        if (code.startsWith('Arrow')) return code.slice(5);
        return code;
    },

    /**
     * Reconnect to Arduino - closes connection and reconnects with new settings
     */
    async reconnectArduino() {
        const reconnectBtn = document.getElementById('reconnectArduinoBtn');
        const statusEl = document.getElementById('settingsArduinoStatus');
        
        // Disable button during reconnection
        if (reconnectBtn) {
            reconnectBtn.disabled = true;
            reconnectBtn.textContent = '↻ Reconnecting...';
            reconnectBtn.classList.add('opacity-50');
        }
        
        if (statusEl) {
            statusEl.textContent = '● Reconnecting...';
            statusEl.className = 'text-xs font-medium text-yellow-400';
        }
        
        try {
            const response = await fetch('/api/arduino/reconnect', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json'
                }
            });
            
            if (response.ok) {
                if (typeof UIUtils !== 'undefined') {
                    UIUtils.log('[SETTINGS] Arduino reconnect request sent', 'success');
                }
                
                // Wait a moment for reconnection to happen
                setTimeout(() => {
                    if (statusEl) {
                        statusEl.textContent = '● Reconnect initiated';
                        statusEl.className = 'text-xs font-medium text-green-400';
                    }
                    if (reconnectBtn) {
                        reconnectBtn.disabled = false;
                        reconnectBtn.textContent = '↻ Reconnect Arduino';
                        reconnectBtn.classList.remove('opacity-50');
                    }
                }, 2000);
                
            } else {
                const data = await response.json();
                throw new Error(data.error || 'Failed to reconnect');
            }
        } catch (error) {
            console.error('[Settings] Error reconnecting Arduino:', error);
            
            if (typeof UIUtils !== 'undefined') {
                UIUtils.log(`[SETTINGS] Arduino reconnect failed: ${error.message}`, 'error');
            }
            
            if (statusEl) {
                statusEl.textContent = '● Reconnect failed';
                statusEl.className = 'text-xs font-medium text-red-400';
            }
            if (reconnectBtn) {
                reconnectBtn.disabled = false;
                reconnectBtn.textContent = '↻ Reconnect Arduino';
                reconnectBtn.classList.remove('opacity-50');
            }
        }
    }
};

// Initialize on DOM load
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => SettingsManager.init());
} else {
    SettingsManager.init();
}
