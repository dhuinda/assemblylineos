/**
 * PixiOverlay - Floating DOM panel for editing block parameters in Pixi mode.
 */
const PixiOverlay = {
    _el: null,
    _headerEl: null,
    _contentEl: null,
    _footerEl: null,
    _currentBlockId: null,
    _mockEl: null,

    _TYPE_LABELS: {
        event: 'Event',
        motor: 'Motor',
        relay: 'Relay',
        delay: 'Delay',
        pause: 'Pause',
        repeat: 'Repeat',
        forever: 'Forever',
        break: 'Break',
        'wait-sensor': 'Wait Sensor',
        'read-sensor': 'Read Sensor',
        'ros-trigger': 'ROS Trigger',
        'motor-speed-from-topic': 'Set Speed',
        'subscribe-motor-speed-topic': 'Subscribe Speed',
        'unsubscribe-motor-speed-topic': 'Unsubscribe Speed',
        try: 'Try',
        catch: 'Catch',
        'throw-error': 'Throw Error',
        'wait-key': 'Wait Key',
        trigger: 'Trigger',
    },

    _build() {
        if (this._el) return;

        const el = document.createElement('div');
        el.id = 'pixi-block-overlay';
        el.setAttribute('role', 'dialog');
        el.setAttribute('aria-label', 'Block parameters');
        el.className = 'pixi-block-overlay-panel';

        const header = document.createElement('div');
        header.className = 'pixi-overlay-header';
        el.appendChild(header);

        const headerMain = document.createElement('div');
        headerMain.className = 'pixi-overlay-header-main';
        header.appendChild(headerMain);

        const closeBtn = document.createElement('button');
        closeBtn.type = 'button';
        closeBtn.className = 'pixi-overlay-close';
        closeBtn.innerHTML = '&times;';
        closeBtn.title = 'Close (Esc)';
        closeBtn.setAttribute('aria-label', 'Close');
        closeBtn.addEventListener('click', () => {
            if (typeof WorkflowManager !== 'undefined') {
                WorkflowManager.deselectBlock();
            } else {
                this.hide();
            }
        });
        header.appendChild(closeBtn);

        const content = document.createElement('div');
        content.className = 'pixi-overlay-content';
        el.appendChild(content);

        const footer = document.createElement('div');
        footer.className = 'pixi-overlay-footer';
        el.appendChild(footer);

        document.body.appendChild(el);
        this._el = el;
        this._headerEl = header;
        this._headerMainEl = headerMain;
        this._contentEl = content;
        this._footerEl = footer;

        el.addEventListener('pointerdown', e => e.stopPropagation());
    },

    _typeLabel(blockData) {
        return this._TYPE_LABELS[blockData.type] ||
            (blockData.type || 'block').replace(/-/g, ' ').replace(/\b\w/g, c => c.toUpperCase());
    },

    _accentClass(blockData) {
        const t = blockData.type || 'default';
        if (t === 'event') return 'pixi-accent-event';
        if (t === 'motor' || t === 'motor-speed-from-topic' ||
            t === 'subscribe-motor-speed-topic' || t === 'unsubscribe-motor-speed-topic') {
            return 'pixi-accent-motor';
        }
        return `pixi-accent-${t}` || 'pixi-accent-default';
    },

    _updateHeader(blockData) {
        if (!this._headerEl || !this._headerMainEl) return;

        this._headerEl.className = `pixi-overlay-header ${this._accentClass(blockData)}`;
        this._headerMainEl.innerHTML = '';

        const typeEl = document.createElement('span');
        typeEl.className = 'pixi-overlay-type';
        typeEl.textContent = this._typeLabel(blockData);

        const idEl = document.createElement('span');
        idEl.className = 'pixi-overlay-id';
        idEl.textContent = `#${blockData.id}`;

        this._headerMainEl.appendChild(typeEl);
        this._headerMainEl.appendChild(idEl);
    },

    show(blockData, renderer) {
        this._build();
        this._currentBlockId = blockData.id;
        this._mockEl = null;

        this._updateHeader(blockData);

        const content = this._contentEl;
        content.innerHTML = '';
        this._footerEl.innerHTML = '';

        if (typeof BlockRenderer === 'undefined') {
            content.textContent = '(BlockRenderer unavailable)';
        } else {
            const mockEl = document.createElement('div');
            mockEl.className = `scratch-block pixi-overlay-block ${this._accentClass(blockData).replace('pixi-accent-', 'block-')}`;
            mockEl.dataset.blockId = String(blockData.id);
            mockEl.dataset.type = blockData.type;
            if (blockData.motor_id !== undefined) mockEl.dataset.motorId = String(blockData.motor_id);
            if (blockData.relay_id !== undefined) mockEl.dataset.relayId = String(blockData.relay_id);
            if (blockData.keyCode) mockEl.dataset.keyCode = blockData.keyCode;

            mockEl.innerHTML = BlockRenderer.generateScratchBlockContent(blockData);
            BlockRenderer.attachParameterListeners(mockEl, blockData);

            const onParamChange = () => {
                const latest = WorkflowManager.blocks.get(blockData.id);
                if (!latest) return;
                BlockSystem.updateBlockFromDOM(latest, mockEl);
                if (typeof PixiWorkspaceRenderer !== 'undefined' && PixiWorkspaceRenderer.enabled) {
                    PixiWorkspaceRenderer.refreshBlockLabels(blockData.id);
                }
                StorageManager.autoSave();
            };

            mockEl.addEventListener('input', onParamChange);
            mockEl.addEventListener('change', onParamChange);

            content.appendChild(mockEl);
            this._mockEl = mockEl;
        }

        const del = document.createElement('button');
        del.type = 'button';
        del.className = 'pixi-overlay-delete';
        del.innerHTML = '<span class="pixi-overlay-delete-icon" aria-hidden="true">&#128465;</span> Delete block';
        del.addEventListener('click', () => {
            if (typeof UIUtils !== 'undefined') {
                if (!UIUtils.confirm(`Delete block ${blockData.id}?`)) return;
            }
            WorkflowManager.removeBlock(blockData.id);
            this.hide();
        });
        this._footerEl.appendChild(del);

        this._el.style.display = 'flex';
        this._el.classList.remove('pixi-overlay-visible');
        requestAnimationFrame(() => {
            this._el.classList.add('pixi-overlay-visible');
        });
        this.reposition(blockData, renderer);

        const focusable = this._el.querySelector('.pixi-overlay-content input, .pixi-overlay-content select');
        if (focusable) focusable.focus({ preventScroll: true });
    },

    reposition(blockData, renderer) {
        if (!this._el || this._currentBlockId !== blockData.id) return;
        const r = renderer || (typeof PixiWorkspaceRenderer !== 'undefined' ? PixiWorkspaceRenderer : null);
        if (!r) return;
        const pos = r.getBlockScreenPos(blockData.id);
        if (!pos) return;

        const vw = window.innerWidth;
        const vh = window.innerHeight;
        const elW = Math.min(340, Math.max(260, this._el.offsetWidth || 300));
        const elH = Math.min(vh * 0.75, this._el.offsetHeight || 400);

        let left = pos.x + pos.w + 14;
        if (left + elW > vw - 12) left = pos.x - elW - 14;
        if (left < 12) left = 12;

        let top = pos.y - 4;
        if (top + elH > vh - 12) top = vh - elH - 12;
        if (top < 12) top = 12;

        this._el.style.left = left + 'px';
        this._el.style.top = top + 'px';
    },

    hide() {
        if (this._el) {
            this._el.classList.remove('pixi-overlay-visible');
            this._el.style.display = 'none';
        }
        this._currentBlockId = null;
        this._mockEl = null;
    },

    isVisible() {
        return !!this._el && this._el.style.display !== 'none';
    },

    currentBlockId() {
        return this._currentBlockId;
    },
};
