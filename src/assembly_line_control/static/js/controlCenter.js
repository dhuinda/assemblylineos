/**
 * Control Center — default SYSTEM tab: health, live Arduino-path telemetry, session context.
 */
const ControlCenter = {
    potHistory: [],
    maxPotHistory: 120,
    incidentLog: [],
    maxIncidents: 120,
    serialLines: [],
    maxSerial: 40,
    sensorFlash: new Map(),
    _prevSensorVal: new Map(),
    _uiUtilsLogPatched: false,
    _rafScheduled: false,
    _potDomReady: false,

    init() {
        this.patchUiUtilsLog();
        this.bindIncidentsFilter();
        this.bindPreflight();
        this.bindSelfTest();
        this.bindKeyboard();
        this.bindControlPanelClicks();
        if (typeof window.onArduinoSerialLog !== 'function') {
            window.onArduinoSerialLog = (line) => this.pushSerial(line);
        } else {
            const prev = window.onArduinoSerialLog;
            window.onArduinoSerialLog = (line) => {
                try { prev(line); } catch (e) {}
                this.pushSerial(line);
            };
        }
        const prevExec = window.onExecutionStateUpdate;
        window.onExecutionStateUpdate = (state) => {
            if (typeof prevExec === 'function') prevExec(state);
            this.onExecutionState(state);
        };
        if (ROSBridge.executionSyncState) {
            this.onExecutionState(ROSBridge.executionSyncState);
        }
        this.fetchVersion();
        this.scheduleRefresh();
        this.incidentPush('[SYSTEM] Control Center ready', 'info');
    },

    patchUiUtilsLog() {
        if (this._uiUtilsLogPatched || typeof UIUtils === 'undefined' || !UIUtils.log) return;
        this._uiUtilsLogPatched = true;
        const orig = UIUtils.log.bind(UIUtils);
        UIUtils.log = (message, type = 'info') => {
            orig(message, type);
            this.incidentPush(message, type);
        };
    },

    bindIncidentsFilter() {
        document.querySelectorAll('input[name="ccIncFilter"]').forEach((el) => {
            el.addEventListener('change', () => this.renderIncidentLog());
        });
    },

    bindPreflight() {
        const wrap = document.getElementById('ccPreflight');
        if (!wrap) return;
        const keys = [
            { id: 'pf_saved', label: 'Project saved / backed up' },
            { id: 'pf_sim', label: 'Simulation mode matches intent' },
            { id: 'pf_clear', label: 'Area clear / safety ok' }
        ];
        wrap.innerHTML = '<div class="text-gray-400 font-medium mb-1">Pre-flight (session)</div>';
        keys.forEach((k) => {
            const checked = sessionStorage.getItem(`cc_pf_${k.id}`) === '1';
            const row = document.createElement('label');
            row.className = 'flex items-center gap-2 text-gray-300 cursor-pointer py-0.5';
            row.innerHTML = `<input type="checkbox" data-pf="${k.id}" ${checked ? 'checked' : ''}/> <span>${k.label}</span>`;
            wrap.appendChild(row);
        });
        wrap.querySelectorAll('input[data-pf]').forEach((cb) => {
            cb.addEventListener('change', () => {
                sessionStorage.setItem(`cc_pf_${cb.getAttribute('data-pf')}`, cb.checked ? '1' : '0');
            });
        });
    },

    bindSelfTest() {
        const wrap = document.getElementById('ccSelfTest');
        if (!wrap) return;
        const steps = [
            { id: 'st1', text: 'Confirm E-STOP is accessible' },
            { id: 'st2', text: 'Jog motor 1 ±10 steps from CONTROLS tab; motion matches direction' },
            { id: 'st3', text: 'Toggle one relay; RELAYS section shows state change' },
            { id: 'st4', text: 'If sensors registered: watch value/update in Live sensors' }
        ];
        wrap.innerHTML = '<div class="text-gray-400 font-medium mb-1">Guided checks</div>';
        steps.forEach((s, i) => {
            const done = sessionStorage.getItem(`cc_st_${s.id}`) === '1';
            const row = document.createElement('div');
            row.className = 'flex items-start gap-2 py-1';
            row.innerHTML = `
                <input type="checkbox" id="cc_st_cb_${s.id}" data-stid="${s.id}" ${done ? 'checked' : ''}/>
                <label for="cc_st_cb_${s.id}" class="cursor-pointer text-gray-300">${i + 1}. ${s.text}</label>`;
            wrap.appendChild(row);
        });
        wrap.querySelectorAll('input[data-stid]').forEach((cb) => {
            cb.addEventListener('change', () => {
                sessionStorage.setItem(`cc_st_${cb.getAttribute('data-stid')}`, cb.checked ? '1' : '0');
            });
        });
    },

    bindKeyboard() {
        document.addEventListener('keydown', (e) => {
            if (e.ctrlKey && e.key === '`') {
                e.preventDefault();
                if (typeof switchSidebarTab === 'function') switchSidebarTab('control');
            }
        });
    },

    bindControlPanelClicks() {
        const panel = document.getElementById('sidebarPanel-control');
        if (!panel) return;
        panel.addEventListener('click', (e) => {
            const btn = e.target.closest('.cc-sensor-copy');
            if (btn) {
                const id = btn.getAttribute('data-sid');
                if (id != null) this.copySensorById(id);
            }
        });
    },

    copySensorById(sensorId) {
        let s = ROSBridge.sensorStates.get(sensorId);
        if (!s && sensorId !== '' && !Number.isNaN(Number(sensorId))) {
            s = ROSBridge.sensorStates.get(Number(sensorId));
        }
        if (s) this._copy(JSON.stringify(s, null, 2));
    },

    onRosConnected() {
        this.incidentPush('[ROS] Session re/connected', 'success');
    },

    onRosDisconnected() {
        this.incidentPush('[ROS] Session disconnected', 'error');
    },

    isPaused() {
        const el = document.getElementById('ccPauseLive');
        return el && el.checked;
    },

    scheduleRefresh() {
        if (this._rafScheduled) return;
        this._rafScheduled = true;
        requestAnimationFrame(() => {
            this._rafScheduled = false;
            if (!this.isPaused()) this.refresh();
            this.scheduleRefresh();
        });
    },

    refresh() {
        this.renderSessionStrip();
        this.renderSafetyStrip();
        this.renderBootAndMatrix();
        this.renderLiveMotors();
        this.renderLiveRelays();
        this.renderLivePot();
        this.renderLiveSensors();
        this.renderSerialLog();
    },

    onExecutionState(state) {
        const strip = document.getElementById('headerExecutionStrip');
        if (!strip) return;
        const sim = typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive;
        const parts = [];
        if (state.running) {
            const mine = state.clientId === ROSBridge.getClientId();
            strip.classList.remove('hidden');
            parts.push(`<span class="font-bold text-amber-400">EXECUTION RUNNING</span>`);
            parts.push(`<span class="text-gray-400">${mine ? 'This tab is executor' : 'Another client is executor'}</span>`);
            if (state.clientId) parts.push(`<span class="text-gray-500 font-mono">${state.clientId.slice(0, 8)}…</span>`);
        } else {
            strip.classList.add('hidden');
        }
        if (sim) {
            strip.classList.remove('hidden');
            parts.push(`<span class="text-cyan-400">SIMULATION — no physical motion</span>`);
        }
        strip.innerHTML = parts.join(' <span class="text-gray-600">|</span> ');
    },

    renderSessionStrip() {
        const el = document.getElementById('ccSessionStrip');
        if (!el) return;
        const wsMs = ROSBridge.wsSessionStartMs ? Date.now() - ROSBridge.wsSessionStartMs : 0;
        const wsSec = Math.floor(wsMs / 1000);
        const proj = (typeof StorageManager !== 'undefined' && StorageManager.getCurrentProject())
            ? StorageManager.getCurrentProject()
            : null;
        const sim = typeof SimulationEngine !== 'undefined' && SimulationEngine.isActive;
        const reconnects = ROSBridge.wsReconnectCount || 0;
        el.innerHTML = `
            <div><span class="text-gray-500">Bridge URL</span> <span class="text-gray-200 font-mono">${this._esc(Config.ROS_BRIDGE_URL || '')}</span></div>
            <div><span class="text-gray-500">WS uptime</span> <span class="text-gray-200">${wsSec}s</span>
                <span class="text-gray-500 ml-2">reconnects</span> <span class="text-gray-200">${reconnects}</span></div>
            <div><span class="text-gray-500">Project</span> <span class="text-gray-200">${proj ? this._esc(proj.name || proj.id || '') : '—'}</span></div>
            <div><span class="text-gray-500">Simulation</span> <span class="${sim ? 'text-cyan-400' : 'text-gray-200'}">${sim ? 'ON' : 'OFF'}</span></div>
            <div class="flex flex-wrap gap-1 pt-1">
                <button type="button" class="btn-secondary px-1.5 py-0.5 text-[10px]" onclick="ProjectDialog.open()">Projects</button>
                <button type="button" class="btn-secondary px-1.5 py-0.5 text-[10px]" onclick="SettingsManager.openDialog()">Settings</button>
            </div>`;
    },

    renderSafetyStrip() {
        const el = document.getElementById('ccSafetyStrip');
        if (!el) return;
        const run = ROSBridge.executionSyncState && ROSBridge.executionSyncState.running;
        const other = run && ROSBridge.executionSyncState.clientId !== ROSBridge.getClientId();
        el.innerHTML = `
            <span class="${run ? 'text-amber-400 font-bold' : 'text-gray-500'}">${run ? '● Run active' : '○ Idle'}</span>
            ${other ? '<span class="text-yellow-400">Another device is executor</span>' : ''}
            <button type="button" class="btn-danger px-2 py-0.5 text-xs ml-auto" onclick="ROSBridge.emergencyStop()">E-STOP</button>`;
    },

    renderBootAndMatrix() {
        const boot = document.getElementById('ccBootChecklist');
        const matrix = document.getElementById('ccSubsystemMatrix');
        if (!boot || !matrix) return;

        const rosOk = ROSBridge.isConnected;
        const ard = ROSBridge.arduinoStatus;
        const ardOk = ard && ard.connected;
        const motorOk = !!(ROSBridge.motorStatus[1] || ROSBridge.motorStatus[2]);
        const sensN = ROSBridge.sensorStates ? ROSBridge.sensorStates.size : 0;

        boot.innerHTML = `
            <div class="text-gray-400 font-medium mb-1">Boot checklist</div>
            <ul class="space-y-0.5 text-gray-300">
                <li>${rosOk ? '✓' : '…'} Rosbridge connected</li>
                <li>${ardOk ? '✓' : (ard ? '✗' : '…')} Arduino serial ${ard ? (ardOk ? 'OK' : 'down') : '(no status yet)'}</li>
                <li>${motorOk ? '✓' : '…'} Motor status seen</li>
                <li>${sensN ? `✓ ${sensN} sensor(s)` : '…'} Sensors registered</li>
            </ul>`;

        const row = (name, ok, detail, topic) => {
            const st = ok ? 'ok' : 'bad';
            const hz = topic && ROSBridge.telemetryMeta[topic] ? ROSBridge.telemetryMeta[topic].hz.toFixed(1) : '—';
            const age = topic ? ROSBridge.getTelemetryAgeSec(topic) : null;
            const stale = age != null && age > 3 && ardOk && topic && topic.includes('motor');
            return `<tr class="border-b border-gray-700/50">
                <td class="py-1 pr-2">${this._esc(name)}</td>
                <td class="py-1"><span class="cc-badge cc-badge-${st}">${ok ? 'OK' : '—'}</span></td>
                <td class="py-1 text-gray-500">${detail}</td>
                <td class="py-1 text-gray-500">${topic ? `${hz} Hz` : ''}</td>
                <td class="py-1">${topic ? `<button type="button" class="text-blue-400 hover:underline text-[10px]" onclick="ControlCenter.copyTopic('${topic}')">copy</button>` : ''}</td>
            </tr>`;
        };

        matrix.innerHTML = `
            <div class="text-gray-400 font-medium mb-1">Subsystem matrix</div>
            <table class="w-full text-[10px] border-collapse">
                <thead><tr class="text-gray-500"><th class="text-left">Subsystem</th><th></th><th class="text-left">Detail</th><th>Rate</th><th></th></tr></thead>
                <tbody>
                ${row('Rosbridge', rosOk, rosOk ? 'WS up' : 'down', null)}
                ${row('Arduino', !!ardOk, ard ? `${ard.connected ? 'on' : 'off'} ${this._esc(ard.port || '')}` : '…', '/arduino/status')}
                ${row('Motor 1', !!ROSBridge.motorStatus[1], ROSBridge.motorStatus[1] ? `spd ${ROSBridge.motorStatus[1].speed}` : '', '/motor1/status')}
                ${row('Motor 2', !!ROSBridge.motorStatus[2], ROSBridge.motorStatus[2] ? `spd ${ROSBridge.motorStatus[2].speed}` : '', '/motor2/status')}
                ${row('Potentiometer', ROSBridge.lastPotRaw != null, ROSBridge.lastPotRaw != null ? String(ROSBridge.lastPotRaw.toFixed(1)) : 'no stream', '/potentiometer/raw')}
                ${row('Speed setpoint', ROSBridge.motorSpeedSetpoint != null, ROSBridge.motorSpeedSetpoint != null ? String(ROSBridge.motorSpeedSetpoint.toFixed(2)) : 'no /motor_speed/setpoint', '/motor_speed/setpoint')}
                </tbody>
            </table>`;
    },

    renderLiveMotors() {
        const el = document.getElementById('ccLiveMotors');
        if (!el) return;
        let html = '<div class="text-gray-400 font-medium mb-2">Live motors</div><div class="space-y-2">';
        for (const id of [1, 2]) {
            const st = ROSBridge.getMotorStatus(id);
            const bus = ROSBridge.topicMotorSpeed[id];
            const meta = ROSBridge.telemetryMeta['/motor' + id + '/status'];
            const age = meta ? ROSBridge.getTelemetryAgeSec('/motor' + id + '/status') : null;
            const stale = age != null && age > 0.5;
            let pct = 0;
            if (st && st.steps_total > 0) {
                pct = Math.max(0, Math.min(100, 100 * (1 - (st.steps_remaining || 0) / st.steps_total)));
            }
            html += `<div class="rounded border border-gray-600 p-2">
                <div class="flex justify-between items-center mb-1">
                    <span class="font-medium text-gray-200">Motor ${id}</span>
                    <span class="text-[10px] ${st && st.is_moving ? 'text-green-400' : 'text-gray-500'}">${st && st.is_moving ? 'MOVING' : 'idle'}</span>
                </div>
                <div class="h-1.5 bg-gray-700 rounded overflow-hidden mb-1">
                    <div class="h-full bg-blue-500 transition-all" style="width:${pct}%"></div>
                </div>
                <div class="text-[10px] text-gray-400 space-y-0.5">
                    <div>rem ${st ? st.steps_remaining : '—'} / tot ${st ? st.steps_total : '—'} · rpt spd ${st ? st.speed : '—'} · bus spd ${bus != null ? bus.toFixed(0) : '—'}</div>
                    <div>${stale ? '<span class="text-amber-400">stale telemetry</span>' : 'fresh'} · topic age ${age != null ? age.toFixed(2) + 's' : '—'}</div>
                    <button type="button" class="text-blue-400 text-[10px]" onclick="ControlCenter.copyTopic('/motor${id}/status')">copy JSON</button>
                </div>
            </div>`;
        }
        html += '</div>';
        el.innerHTML = html;
    },

    renderLiveRelays() {
        const el = document.getElementById('ccLiveRelays');
        if (!el) return;
        let html = '<div class="text-gray-400 font-medium mb-2">Live relays</div><div class="grid grid-cols-2 gap-2">';
        for (let r = 1; r <= 4; r++) {
            const rs = ROSBridge.relayStates[r];
            const on = rs && rs.state === 'on';
            const hz = ROSBridge.telemetryMeta[`/relay${r}/status`];
            const since = rs && rs.lastChangeMs ? Math.floor((Date.now() - rs.lastChangeMs) / 1000) : null;
            html += `<div class="rounded border border-gray-600 p-2 ${on ? 'border-amber-500/60' : ''}">
                <div class="flex justify-between"><span>Relay ${r}</span><span class="font-bold ${on ? 'text-amber-400' : 'text-gray-500'}">${on ? 'ON' : 'OFF'}</span></div>
                <div class="text-[10px] text-gray-500">${hz ? hz.hz.toFixed(1) + ' Hz' : ''} ${since != null ? '· since chg ' + since + 's' : ''}</div>
                <button type="button" class="text-blue-400 text-[10px]" onclick="ControlCenter.copyTopic('/relay${r}/status')">copy</button>
            </div>`;
        }
        html += '</div>';
        el.innerHTML = html;
    },

    renderLivePot() {
        const el = document.getElementById('ccLivePot');
        if (!el) return;
        if (!this._potDomReady) {
            el.innerHTML = `
            <div class="text-gray-400 font-medium mb-1">Potentiometer / speed path</div>
            <div class="flex items-center gap-2">
                <div class="flex-1 h-2 bg-gray-700 rounded overflow-hidden">
                    <div id="ccPotBar" class="h-full bg-purple-500" style="width:0%"></div>
                </div>
                <span id="ccPotVal" class="text-gray-200 font-mono w-14">—</span>
            </div>
            <div id="ccPotSetp" class="text-[10px] text-gray-500 mt-1"></div>
            <canvas id="ccPotCanvas" width="280" height="48" class="w-full bg-gray-900/50 rounded mt-1"></canvas>`;
            this._potDomReady = true;
        }
        const v = ROSBridge.lastPotRaw;
        const setp = ROSBridge.motorSpeedSetpoint;
        if (v != null) {
            this.potHistory.push(v);
            if (this.potHistory.length > this.maxPotHistory) this.potHistory.shift();
        }
        const pct = v != null ? Math.max(0, Math.min(100, (v / 1023) * 100)) : 0;
        const bar = document.getElementById('ccPotBar');
        const valEl = document.getElementById('ccPotVal');
        const setpEl = document.getElementById('ccPotSetp');
        if (bar) bar.style.width = pct + '%';
        if (valEl) valEl.textContent = v != null ? v.toFixed(0) : '—';
        if (setpEl) setpEl.textContent = '/motor_speed/setpoint: ' + (setp != null ? setp.toFixed(2) : '—');

        const canvas = document.getElementById('ccPotCanvas');
        if (canvas && this.potHistory.length > 1) {
            const ctx = canvas.getContext('2d');
            const w = canvas.width;
            const h = canvas.height;
            ctx.clearRect(0, 0, w, h);
            ctx.strokeStyle = '#a78bfa';
            ctx.beginPath();
            const arr = this.potHistory;
            for (let i = 0; i < arr.length; i++) {
                const x = (i / (arr.length - 1)) * w;
                const y = h - (arr[i] / 1023) * h;
                if (i === 0) ctx.moveTo(x, y);
                else ctx.lineTo(x, y);
            }
            ctx.stroke();
        }
    },

    renderLiveSensors() {
        const el = document.getElementById('ccLiveSensors');
        if (!el) return;
        if (!ROSBridge.sensorStates || ROSBridge.sensorStates.size === 0) {
            this._prevSensorVal.clear();
            el.innerHTML = '<div class="text-gray-400 font-medium mb-1">Live sensors</div><p class="text-gray-500 text-[10px]">No sensors registered (no sensor/status yet).</p>';
            return;
        }
        ROSBridge.sensorStates.forEach((s, sid) => {
            const cur = JSON.stringify(s.value);
            const prev = this._prevSensorVal.get(sid);
            if (prev !== undefined && prev !== cur) this.sensorFlash.set(sid, 3);
            this._prevSensorVal.set(sid, cur);
        });
        let html = '<div class="text-gray-400 font-medium mb-1">Live sensors</div><table class="w-full text-[10px]">';
        html += '<thead><tr class="text-gray-500"><th class="text-left">ID</th><th>type</th><th>value</th><th>conn</th><th></th></tr></thead><tbody>';
        ROSBridge.sensorStates.forEach((s) => {
            const val = s.type === 'digital' ? (s.value ? 'HIGH' : 'LOW') : String(s.value);
            let flashes = this.sensorFlash.get(s.sensor_id) || 0;
            const flash = flashes > 0;
            if (flashes > 0) this.sensorFlash.set(s.sensor_id, flashes - 1);
            const sidAttr = this._attr(String(s.sensor_id));
            html += `<tr class="${flash ? 'bg-yellow-900/30' : ''} border-b border-gray-700/40">
                <td class="py-0.5">${this._esc(String(s.sensor_id))}</td>
                <td>${this._esc(s.type)}</td>
                <td class="font-mono">${this._esc(val)}</td>
                <td>${s.connected ? 'y' : 'n'}</td>
                <td><button type="button" class="cc-sensor-copy text-blue-400 hover:underline" data-sid="${sidAttr}">copy</button></td>
            </tr>`;
        });
        html += '</tbody></table>';
        el.innerHTML = html;
    },

    _attr(s) {
        return String(s).replace(/&/g, '&amp;').replace(/"/g, '&quot;').replace(/</g, '&lt;');
    },

    renderSerialLog() {
        const el = document.getElementById('ccSerialLog');
        if (!el) return;
        el.innerHTML = '<div class="text-gray-400 font-medium mb-1">Arduino serial (other JSON)</div>' +
            this.serialLines.map((l) => `<div class="truncate">${this._esc(l)}</div>`).join('');
        el.scrollTop = el.scrollHeight;
    },

    pushSerial(line) {
        if (!line) return;
        this.serialLines.push(`[${new Date().toLocaleTimeString()}] ${line}`);
        if (this.serialLines.length > this.maxSerial) this.serialLines.shift();
    },

    copyTopic(topic) {
        const raw = ROSBridge.telemetryLastRaw[topic];
        if (raw) this._copy(raw);
        else this._copy(topic);
    },

    _copy(text) {
        if (navigator.clipboard && navigator.clipboard.writeText) {
            navigator.clipboard.writeText(text).then(() => UIUtils.log('[CONTROL] Copied to clipboard', 'success'));
        } else {
            UIUtils.log('[CONTROL] Copy not available', 'warning');
        }
    },

    _esc(s) {
        const d = document.createElement('div');
        d.textContent = s;
        return d.innerHTML;
    },

    incidentPush(message, type = 'info') {
        this.incidentLog.push({ t: Date.now(), message, type });
        if (this.incidentLog.length > this.maxIncidents) this.incidentLog.shift();
        this.renderIncidentLog();
    },

    renderIncidentLog() {
        const el = document.getElementById('ccIncidentLog');
        if (!el) return;
        const filter = document.querySelector('input[name="ccIncFilter"]:checked');
        const mode = filter ? filter.value : 'all';
        const lines = this.incidentLog.filter((e) => mode !== 'err' || e.type === 'error');
        el.innerHTML = lines.slice(-80).map((e) => {
            const cls = e.type === 'error' ? 'text-red-400' : e.type === 'success' ? 'text-green-400' : e.type === 'warning' ? 'text-yellow-400' : 'text-gray-400';
            return `<div class="${cls}">${new Date(e.t).toLocaleTimeString()} ${this._esc(e.message)}</div>`;
        }).join('');
        el.scrollTop = el.scrollHeight;
    },

    exportIncidentLog() {
        const text = this.incidentLog.map((e) => `[${new Date(e.t).toISOString()}] [${e.type}] ${e.message}`).join('\n');
        const blob = new Blob([text], { type: 'text/plain' });
        const a = document.createElement('a');
        a.href = URL.createObjectURL(blob);
        a.download = `assembly-line-incident-${Date.now()}.txt`;
        a.click();
        URL.revokeObjectURL(a.href);
    },

    reconnectArduino() {
        fetch('/api/arduino/reconnect', { method: 'POST' })
            .then((r) => r.json().catch(() => ({})))
            .then((data) => {
                UIUtils.log('[CONTROL] Arduino reconnect requested', 'success');
                this.incidentPush('[CONTROL] POST /api/arduino/reconnect', 'info');
            })
            .catch(() => UIUtils.log('[CONTROL] Reconnect request failed', 'error'));
    },

    fetchVersion() {
        const foot = document.getElementById('ccVersionFooter');
        fetch('/api/version')
            .then((r) => r.json())
            .then((data) => {
                if (foot) foot.textContent = `Build: ${data.package || '?'} ${data.version || ''} ${data.git || ''}`.trim();
            })
            .catch(() => { if (foot) foot.textContent = ''; });
    },
};
