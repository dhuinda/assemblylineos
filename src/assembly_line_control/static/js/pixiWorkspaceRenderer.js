/**
 * PixiWorkspaceRenderer - GPU workspace: blocks, connections, previews, triggers.
 *
 * Blocks and connection lines render on the GPU. Parameter editing uses
 * PixiOverlay (floating panel for the selected block). Connection segments
 * have expanded hit areas for hover, click, and right-click disconnect.
 */
const PixiWorkspaceRenderer = {

    TAP_THRESHOLD: 5,

    /** Compact GPU block chrome (params live in PixiOverlay, not on the block). */
    BLOCK_PAD_X: 6,
    BLOCK_PAD_Y: 5,
    BLOCK_MIN_HEIGHT: 40,

    app: null,
    stage: null,
    connectionLayer: null,
    blockLayer: null,
    uiLayer: null,
    hostEl: null,

    blockObjects: new Map(),
    connectionObjects: new Map(),
    _previewGraphics: null,
    _triggerGraphics: null,
    _hoveredConnections: new Set(),

    _scrollEl: null,
    _scrollX: 0,
    _scrollY: 0,
    _contentWidth: 800,
    _contentHeight: 600,

    _dragState: null,
    _panState: null,
    _snappingIds: new Set(),
    _executingIds: new Set(),
    _errorIds: new Set(),
    _warningIds: new Set(),
    _activeConnectionKeys: new Set(),
    _pulsePhase: 0,
    _selectedId: null,
    _connectingHighlight: null,

    enabled: false,
    _initPromise: null,
    _themeCache: null,
    _lastThemeKey: null,
    _themeObserver: null,
    _lastTriggerLinks: null,
    _lastPreviewSegments: null,
    _execGlowLastTick: 0,

    _isLowPower() {
        if (typeof document === 'undefined') return false;
        const root = document.documentElement;
        return root.classList.contains('low-power-mode') || root.classList.contains('raspi-mode');
    },

    _TYPE_CSS_KEY: {
        event: 'event',
        motor: 'motor',
        'motor-speed-from-topic': 'motor',
        'subscribe-motor-speed-topic': 'motor',
        'unsubscribe-motor-speed-topic': 'motor',
        relay: 'relay',
        pause: 'pause',
        delay: 'delay',
        repeat: 'loop',
        forever: 'loop',
        break: 'loop',
        'wait-sensor': 'sensor',
        'read-sensor': 'sensor',
        'ros-trigger': 'sensor',
        'throw-error': 'error-block',
        try: 'error-block',
        catch: 'error-block',
        'wait-key': 'wait-key',
    },

    _cssColor(varName, fallback) {
        if (typeof document === 'undefined') return fallback;
        const raw = getComputedStyle(document.documentElement).getPropertyValue(varName).trim();
        if (!raw) return fallback;
        if (raw.startsWith('#')) {
            let h = raw.slice(1);
            if (h.length === 3) h = h.split('').map(c => c + c).join('');
            const n = parseInt(h, 16);
            return Number.isFinite(n) ? n : fallback;
        }
        const m = raw.match(/rgba?\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)/);
        if (m) {
            return (parseInt(m[1], 10) << 16) | (parseInt(m[2], 10) << 8) | parseInt(m[3], 10);
        }
        return fallback;
    },

    _cssAlpha(varName, fallback) {
        if (typeof document === 'undefined') return fallback;
        const raw = getComputedStyle(document.documentElement).getPropertyValue(varName).trim();
        const n = parseFloat(raw);
        return Number.isFinite(n) ? n : fallback;
    },

    _themeKey() {
        if (typeof document === 'undefined') return 'dark';
        return document.documentElement.getAttribute('data-theme') === 'light' ? 'light' : 'dark';
    },

    _themeColor(name, fallback) {
        const key = this._themeKey();
        if (!this._themeCache || this._themeCache._key !== key) {
            this._themeCache = {
                _key: key,
                blockFillAlpha: this._cssAlpha('--pixi-block-fill-alpha', 0.92),
                labelId: this._cssColor('--pixi-label-id', 0x888888),
                labelDetail: this._cssColor('--pixi-label-detail', 0xbbbbbb),
                connLine: this._cssColor('--pixi-conn-line', 0x3b82f6),
                connActive: this._cssColor('--pixi-conn-active', 0x10b981),
                connHover: this._cssColor('--pixi-conn-hover', 0xef4444),
                connPreview: this._cssColor('--pixi-conn-preview', 0x10b981),
                selected: this._cssColor('--pixi-selected', 0x3b82f6),
                executing: this._cssColor('--pixi-executing', 0x10b981),
                error: this._cssColor('--pixi-error', 0xef4444),
                warning: this._cssColor('--pixi-warning', 0xf59e0b),
                snapping: this._cssColor('--pixi-snapping', 0x10b981),
                connected: this._cssColor('--pixi-connected', 0x10b981),
                connecting: this._cssColor('--pixi-connecting', 0xfbbf24),
                triggerLink: this._cssColor('--pixi-trigger-link', 0x8b5cf6),
                waypointFill: this._cssColor('--pixi-waypoint-fill', 0x8b5cf6),
                waypointStroke: this._cssColor('--pixi-waypoint-stroke', 0xc4b5fd),
                defaultBg: this._cssColor('--pixi-default-bg', 0x1a1a1a),
                defaultBorder: this._cssColor('--pixi-default-border', 0x555555),
            };
        }
        return this._themeCache[name] ?? fallback;
    },

    _colorFor(type) {
        const themeKey = this._themeKey();
        if (!this._blockColorCache || this._blockColorCache._key !== themeKey) {
            this._blockColorCache = { _key: themeKey };
        }
        if (!this._blockColorCache[type]) {
            const cssKey = this._TYPE_CSS_KEY[type] || 'default';
            const suffix = cssKey === 'default' ? 'default' : cssKey;
            this._blockColorCache[type] = {
                bg: this._cssColor(`--pixi-${suffix}-bg`, this._themeColor('defaultBg', 0x1a1a1a)),
                bgA: this._themeColor('blockFillAlpha', 0.92),
                border: this._cssColor(`--pixi-${suffix}-border`, this._themeColor('defaultBorder', 0x555555)),
            };
        }
        return this._blockColorCache[type];
    },

    _bindThemeObserver() {
        if (this._themeObserver || typeof document === 'undefined') return;
        this._lastThemeKey = this._themeKey();
        this._themeObserver = new MutationObserver(() => {
            const key = this._themeKey();
            if (key === this._lastThemeKey) return;
            this._lastThemeKey = key;
            this.refreshTheme();
        });
        this._themeObserver.observe(document.documentElement, {
            attributes: true,
            attributeFilter: ['data-theme'],
        });

        const mq = window.matchMedia('(prefers-color-scheme: light)');
        const onSystemTheme = () => {
            if (localStorage.getItem('assemblyLineTheme')) return;
            document.documentElement.setAttribute('data-theme', mq.matches ? 'light' : '');
        };
        if (typeof mq.addEventListener === 'function') {
            mq.addEventListener('change', onSystemTheme);
        }
    },

    refreshTheme() {
        if (!this.enabled) return;
        this._themeCache = null;
        this._blockColorCache = null;

        this.blockObjects.forEach((_, id) => {
            this.refreshBlockLabels(id);
            this._redrawBodyState(id);
            if (typeof BlockConnector !== 'undefined') {
                this.updateConnectorVisualsForBlock(id);
            }
        });

        this.connectionObjects.forEach((entry, key) => {
            this._drawSegments(
                entry.visible, entry.segments,
                this._hoveredConnections.has(key),
                this._activeConnectionKeys.has(key)
            );
            if (entry.waypoints && entry.waypoints.length > 0) {
                this._drawWaypoints(entry.waypointGfx, entry.waypoints);
            }
        });

        if (this._lastPreviewSegments) {
            this.setPreview(this._lastPreviewSegments);
        }
        if (this._lastTriggerLinks) {
            this.setTriggerLinks(this._lastTriggerLinks);
        }
    },

    _drawWaypoints(g, waypoints) {
        if (!g) return;
        g.clear();
        const fill = this._themeColor('waypointFill', 0x8b5cf6);
        const stroke = this._themeColor('waypointStroke', 0xc4b5fd);
        waypoints.forEach(wp => {
            g.circle(wp.x, wp.y, 5);
            g.fill({ color: fill, alpha: 0.85 });
            g.stroke({ color: stroke, width: 1 });
        });
    },

    isActive() {
        return this.enabled;
    },

    init(scrollEl) {
        if (this._initPromise) return this._initPromise;
        this._initPromise = this._initInternal(scrollEl);
        return this._initPromise;
    },

    async _initInternal(scrollEl) {
        if (!scrollEl) return false;
        if (typeof PIXI === 'undefined') {
            console.warn('[Pixi] PIXI not loaded');
            return false;
        }

        this._scrollEl = scrollEl;

        try {
            const t = document.createElement('canvas');
            const gl = t.getContext('webgl') || t.getContext('experimental-webgl');
            if (!gl) throw new Error('no-webgl');
        } catch (_) {
            console.warn('[Pixi] WebGL unavailable');
            return false;
        }

        this.ensureHost();
        this.syncContentSize();

        const lowPower = this._isLowPower();
        try {
            this.app = new PIXI.Application();
            await this.app.init({
                width: this._contentWidth,
                height: this._contentHeight,
                backgroundAlpha: 0,
                antialias: !lowPower,
                resolution: lowPower ? 1 : Math.min(window.devicePixelRatio || 1, 2),
                autoDensity: true,
            });
        } catch (err) {
            console.warn('[Pixi] Application init failed:', err);
            return false;
        }

        const pixiCanvas = this.app.canvas;
        pixiCanvas.className = 'workspace-pixi-layer';
        pixiCanvas.setAttribute('data-workspace-preserve', 'true');
        pixiCanvas.setAttribute('aria-hidden', 'true');
        this.hostEl.appendChild(pixiCanvas);
        this._mountCanvasStyles();

        this.stage = this.app.stage;
        this.stage.eventMode = 'static';
        this.connectionLayer = new PIXI.Container();
        this.blockLayer = new PIXI.Container();
        this.uiLayer = new PIXI.Container();
        this.stage.addChild(this.connectionLayer);
        this.stage.addChild(this.blockLayer);
        this.stage.addChild(this.uiLayer);

        this._previewGraphics = new PIXI.Graphics();
        this._triggerGraphics = new PIXI.Graphics();
        this.uiLayer.addChild(this._previewGraphics);
        this.uiLayer.addChild(this._triggerGraphics);

        this._bindCanvasEvents(pixiCanvas);
        this._bindScrollSync(scrollEl);
        this._bindEscapeDismiss();
        this._bindExecutionTicker();
        this._bindThemeObserver();

        try {
            new ResizeObserver(() => this.syncContentSize()).observe(scrollEl);
        } catch (_) {
            window.addEventListener('resize', () => this.syncContentSize());
        }

        this.enabled = true;
        if (typeof UIUtils !== 'undefined') {
            UIUtils.log('[Pixi] GPU workspace renderer initialized', 'success');
        }
        return true;
    },

    ensureHost() {
        if (!this._scrollEl) return null;
        let host = this._scrollEl.querySelector('#workspace-pixi-host');
        if (!host) {
            host = document.createElement('div');
            host.id = 'workspace-pixi-host';
            host.setAttribute('data-workspace-preserve', 'true');
            host.className = 'workspace-pixi-host';
            this._scrollEl.appendChild(host);
        }
        this.hostEl = host;
        return host;
    },

    ensureMounted() {
        if (!this.app || !this._scrollEl) return;
        this.ensureHost();
        const canvas = this.app.canvas;
        if (canvas.parentElement !== this.hostEl) {
            this.hostEl.appendChild(canvas);
        }
        this._mountCanvasStyles();
        this.syncContentSize();
    },

    _mountCanvasStyles() {
        if (!this.app || !this.hostEl) return;
        const canvas = this.app.canvas;
        canvas.style.position = 'absolute';
        canvas.style.left = '0';
        canvas.style.top = '0';
        canvas.style.width = this._contentWidth + 'px';
        canvas.style.height = this._contentHeight + 'px';
        canvas.style.zIndex = '6';
        canvas.style.pointerEvents = 'auto';

        this.hostEl.style.position = 'absolute';
        this.hostEl.style.left = '0';
        this.hostEl.style.top = '0';
        this.hostEl.style.width = this._contentWidth + 'px';
        this.hostEl.style.height = this._contentHeight + 'px';
        this.hostEl.style.zIndex = '6';
        this.hostEl.style.pointerEvents = 'none';
        canvas.style.pointerEvents = 'auto';
    },

    syncContentSize() {
        if (!this._scrollEl) return;
        const parent = this._scrollEl;
        const contentWrapper = parent.querySelector('.canvas-content-wrapper');
        const contentWidth = contentWrapper ? parseFloat(contentWrapper.style.width) : 0;
        const contentHeight = contentWrapper ? parseFloat(contentWrapper.style.height) : 0;
        const width = Math.max(contentWidth || 0, parent.clientWidth || 0, 800);
        const height = Math.max(contentHeight || 0, parent.clientHeight || 0, 600);

        this._contentWidth = width;
        this._contentHeight = height;

        if (this.hostEl) {
            this.hostEl.style.width = width + 'px';
            this.hostEl.style.height = height + 'px';
        }

        if (this.app) {
            try { this.app.renderer.resize(width, height); } catch (_) {}
            const canvas = this.app.canvas;
            if (canvas) {
                canvas.style.width = width + 'px';
                canvas.style.height = height + 'px';
            }
        }
    },

    _bindScrollSync(scrollEl) {
        const sync = () => {
            this._scrollX = scrollEl.scrollLeft;
            this._scrollY = scrollEl.scrollTop;
            if (this.stage) {
                this.stage.x = -this._scrollX;
                this.stage.y = -this._scrollY;
            }
            if (typeof PixiOverlay !== 'undefined' && PixiOverlay.isVisible() && this._selectedId != null) {
                const bd = WorkflowManager.blocks.get(this._selectedId);
                if (bd) PixiOverlay.reposition(bd, this);
            }
        };
        scrollEl.addEventListener('scroll', sync, { passive: true });
        sync();
    },

    _bindExecutionTicker() {
        if (!this.app || this._tickerBound) return;
        this._tickerBound = true;
        this.app.ticker.add(() => {
            if (this._executingIds.size === 0) return;
            if (this._isLowPower()) return;
            const now = performance.now();
            if (now - this._execGlowLastTick < 16) return;
            this._execGlowLastTick = now;
            this._pulsePhase += 0.07;
            this._executingIds.forEach(id => this._updateExecGlow(id));
        });
    },

    _updateExecGlow(blockId) {
        const obj = this.blockObjects.get(blockId);
        if (!obj || !obj.execGlow) return;

        if (!this._executingIds.has(blockId)) {
            obj.execGlow.visible = false;
            obj.execGlow.clear();
            return;
        }

        const bd = typeof WorkflowManager !== 'undefined' ? WorkflowManager.blocks.get(blockId) : null;
        if (!bd) return;

        const isEvent = bd.type === 'event';
        const radius = isEvent ? 8 : 6;
        const executingColor = this._themeColor('executing', 0x10b981);
        const lowPower = this._isLowPower();
        const pulse = lowPower ? 1 : (0.65 + 0.35 * Math.sin(this._pulsePhase));
        const glowAlpha = lowPower ? 0.15 : (0.12 + 0.1 * Math.sin(this._pulsePhase));
        const g = obj.execGlow;

        g.clear();
        g.visible = true;
        g.roundRect(-2, -2, obj.w + 4, obj.h + 4, radius + 2);
        g.fill({ color: executingColor, alpha: glowAlpha });
        g.roundRect(0, 0, obj.w, obj.h, radius);
        g.stroke({ color: executingColor, width: lowPower ? 2 : (2 + Math.round(pulse * 2)), alpha: 0.95 });
    },

    _bindEscapeDismiss() {
        window.addEventListener('keydown', (e) => {
            if (e.key !== 'Escape') return;
            if (typeof BlockConnector !== 'undefined' && BlockConnector.connectingFrom) {
                BlockConnector.cancelConnection();
                return;
            }
            if (this._selectedId != null) {
                this._selectBlock(null);
            }
        });
    },

    _bindCanvasEvents(pixiCanvas) {
        pixiCanvas.addEventListener('dragover', (e) => {
            e.preventDefault();
            if (e.dataTransfer) e.dataTransfer.dropEffect = 'copy';
        });

        pixiCanvas.addEventListener('drop', (e) => {
            e.preventDefault();
            e.stopPropagation();
            if (typeof WorkflowManager === 'undefined' || !this._scrollEl) return;
            const rect = this._scrollEl.getBoundingClientRect();
            const x = e.clientX - rect.left + this._scrollEl.scrollLeft;
            const y = e.clientY - rect.top + this._scrollEl.scrollTop;
            WorkflowManager.handleBlockDrop(e, x, y);
        });

        pixiCanvas.addEventListener('pointerdown', (e) => {
            if (e.target !== pixiCanvas) return;
            if (e.button === 0) {
                this._selectBlock(null);
                if (typeof BlockConnector !== 'undefined') BlockConnector.cancelConnection();
            }
            if (e.button === 1 || e.button === 2) {
                this._panState = {
                    startX: e.clientX,
                    startY: e.clientY,
                    scrollLeft: this._scrollEl.scrollLeft,
                    scrollTop: this._scrollEl.scrollTop,
                };
                e.preventDefault();
            }
        });

        window.addEventListener('pointermove', (e) => {
            if (!this._panState || !this._scrollEl) return;
            const dx = e.clientX - this._panState.startX;
            const dy = e.clientY - this._panState.startY;
            this._scrollEl.scrollLeft = this._panState.scrollLeft - dx;
            this._scrollEl.scrollTop = this._panState.scrollTop - dy;
        });

        window.addEventListener('pointerup', () => {
            this._panState = null;
        });

        pixiCanvas.addEventListener('contextmenu', (e) => e.preventDefault());
    },

    _pointerWorld(e) {
        const native = e.nativeEvent || e.originalEvent || e;
        if (native && native.clientX != null && this.app && this._scrollEl) {
            const rect = this.app.canvas.getBoundingClientRect();
            return {
                x: native.clientX - rect.left + this._scrollEl.scrollLeft,
                y: native.clientY - rect.top + this._scrollEl.scrollTop,
            };
        }
        const gx = e.global ? e.global.x : (e.globalX || 0);
        const gy = e.global ? e.global.y : (e.globalY || 0);
        return { x: gx + this._scrollX, y: gy + this._scrollY };
    },

    // -------------------------------------------------------------------------
    // Blocks
    // -------------------------------------------------------------------------

    addBlock(blockData) {
        if (!this.enabled) return;
        this.ensureMounted();
        this._destroyBlockObject(blockData.id);

        const w = this._blockWidth(blockData);
        const h = this._blockHeight(blockData, w);
        const col = this._colorFor(blockData.type);
        const isEvent = blockData.type === 'event';

        const container = new PIXI.Container();
        container.eventMode = 'passive';
        container.sortableChildren = true;
        container.x = blockData.x;
        container.y = blockData.y;

        const body = new PIXI.Graphics();
        this._drawBody(body, w, h, blockData, false, false, false, false, false);
        container.addChild(body);

        const execGlow = new PIXI.Graphics();
        execGlow.visible = false;
        execGlow.zIndex = 2;
        container.addChild(execGlow);

        const labelsLayer = new PIXI.Container();
        container.addChild(labelsLayer);
        const labelRefs = this._createLabels(labelsLayer, blockData, w, h);

        const bodyHit = this._makeBodyHit(blockData, w, h, isEvent);
        container.addChild(bodyHit);

        let topDotVisual = null;
        let topConnectorHit = null;
        if (!isEvent) {
            topDotVisual = this._makeDotVisual(col.border);
            topDotVisual.x = Math.round(w / 2);
            topDotVisual.y = 0;
            container.addChild(topDotVisual);

            topConnectorHit = this._makeConnectorHit(blockData.id, 'top', Math.round(w / 2), 0);
            container.addChild(topConnectorHit);
        }

        const bottomDotVisual = this._makeDotVisual(col.border);
        bottomDotVisual.x = Math.round(w / 2);
        bottomDotVisual.y = h;
        container.addChild(bottomDotVisual);

        const bottomConnectorHit = this._makeConnectorHit(blockData.id, 'bottom', Math.round(w / 2), h);
        container.addChild(bottomConnectorHit);

        this.blockLayer.addChild(container);

        const obj = {
            container, body, execGlow, w, h,
            labelsLayer, labelRefs,
            topDotVisual, bottomDotVisual,
            topConnectorHit, bottomConnectorHit,
            connectorState: { top: { connected: false, connecting: false }, bottom: { connected: false, connecting: false } },
        };
        this.blockObjects.set(blockData.id, obj);

        if (typeof BlockConnector !== 'undefined') {
            this.updateConnectorVisualsForBlock(blockData.id);
        }
    },

    removeBlock(blockId) {
        if (!this.enabled) return;
        this._destroyBlockObject(blockId);
        if (this._selectedId === blockId) {
            this._selectBlock(null);
        }
    },

    updateBlock(blockData) {
        if (!this.enabled) return;
        const wasSelected = this._selectedId === blockData.id;
        const wasExecuting = this._executingIds.has(blockData.id);
        const oldObj = this.blockObjects.get(blockData.id);
        const oldW = oldObj ? oldObj.w : 0;
        const oldH = oldObj ? oldObj.h : 0;
        const newW = this._blockWidth(blockData);
        const newH = this._blockHeight(blockData, newW);

        if (oldObj && oldW === newW && oldH === newH) {
            this.refreshBlockLabels(blockData.id);
            if (wasSelected) this._redrawBodyState(blockData.id);
            if (wasExecuting) this.setBlockExecuting(blockData.id, true);
            return;
        }

        this.addBlock(blockData);
        if (wasSelected) {
            this._selectedId = blockData.id;
            this._redrawBodyState(blockData.id);
        }
        if (wasExecuting) this.setBlockExecuting(blockData.id, true);
    },

    refreshBlockLabels(blockId) {
        if (!this.enabled) return;
        const obj = this.blockObjects.get(blockId);
        const bd = typeof WorkflowManager !== 'undefined' ? WorkflowManager.blocks.get(blockId) : null;
        if (!obj || !bd || !obj.labelRefs) return;

        const col = this._colorFor(bd.type);
        const wrapW = obj.w - this.BLOCK_PAD_X * 2;
        const detail = this._detailFor(bd);
        const newH = this._blockHeight(bd, obj.w);

        if (newH !== obj.h) {
            this.updateBlock(bd);
            return;
        }

        obj.labelRefs.titleText.text = this._titleFor(bd);
        obj.labelRefs.titleText.style.fill = col.border;
        obj.labelRefs.titleText.style.wordWrapWidth = wrapW;

        if (obj.labelRefs.idText) {
            obj.labelRefs.idText.style.fill = this._themeColor('labelId', 0x888888);
        }

        if (obj.labelRefs.detailText) {
            obj.labelRefs.detailText.style.fill = this._themeColor('labelDetail', 0xbbbbbb);
            if (detail) {
                obj.labelRefs.detailText.text = detail;
                obj.labelRefs.detailText.style.wordWrapWidth = wrapW;
                obj.labelRefs.detailText.visible = true;
            } else {
                obj.labelRefs.detailText.visible = false;
            }
        }
    },

    moveBlock(blockId, x, y) {
        if (!this.enabled) return;
        const obj = this.blockObjects.get(blockId);
        if (obj) {
            obj.container.x = x;
            obj.container.y = y;
        }
    },

    _destroyBlockObject(blockId) {
        const obj = this.blockObjects.get(blockId);
        if (!obj) return;
        this.blockLayer.removeChild(obj.container);
        try { obj.container.destroy({ children: true }); } catch (_) {}
        this.blockObjects.delete(blockId);
    },

    _drawBody(g, w, h, blockData, selected, executing, snapping, error, warning) {
        g.clear();
        const col = this._colorFor(blockData.type);
        const isEvent = blockData.type === 'event';
        const radius = isEvent ? 8 : 6;

        let borderColor = col.border;
        let borderWidth = 2;
        const executingColor = this._themeColor('executing', 0x10b981);
        if (error)       { borderColor = this._themeColor('error', 0xef4444); borderWidth = 3; }
        else if (warning) { borderColor = this._themeColor('warning', 0xf59e0b); borderWidth = 3; }
        else if (snapping)  { borderColor = this._themeColor('snapping', 0x10b981); borderWidth = 2; }
        else if (executing) {
            borderColor = executingColor;
            borderWidth = 3;
        }
        if (selected)  {
            borderColor = error ? this._themeColor('error', 0xef4444)
                : (warning ? this._themeColor('warning', 0xf59e0b) : this._themeColor('selected', 0x3b82f6));
            borderWidth = Math.max(borderWidth, 3);
        }

        g.roundRect(0, 0, w, h, radius);
        g.fill({ color: col.bg, alpha: col.bgA });
        g.stroke({ color: borderColor, width: borderWidth });
    },

    _makeDotVisual(color, radius, connected, connecting) {
        const g = new PIXI.Graphics();
        this._drawConnectorDot(g, color, radius || 5, connected, connecting);
        return g;
    },

    _drawConnectorDot(g, color, radius, connected, connecting) {
        g.clear();
        const r = connecting ? radius + 3 : radius;
        const fillColor = connected ? this._themeColor('connected', 0x10b981)
            : (connecting ? this._themeColor('connecting', 0xfbbf24) : color);
        g.circle(0, 0, r);
        g.fill({ color: fillColor, alpha: connecting ? 1 : 0.85 });
        if (connecting) {
            const connCol = this._themeColor('connecting', 0xfbbf24);
            g.circle(0, 0, r + 2);
            g.stroke({ color: connCol, width: 2, alpha: 0.6 });
        }
    },

    _makeConnectorHit(blockId, connectorType, cx, cy) {
        const hitRadius = 14;
        const hit = new PIXI.Graphics();
        hit.circle(0, 0, hitRadius);
        hit.fill({ color: 0xffffff, alpha: 0.001 });
        hit.x = cx;
        hit.y = cy;
        hit.eventMode = 'static';
        hit.cursor = 'pointer';
        hit.zIndex = 10;

        const onConnectorActivate = (e) => {
            e.stopPropagation();
            if (typeof BlockConnector !== 'undefined') {
                BlockConnector.handleConnectorClick(blockId, connectorType);
            }
        };

        hit.on('pointerdown', (e) => {
            e.stopPropagation();
        });
        hit.on('pointertap', onConnectorActivate);

        hit.on('rightclick', (e) => {
            e.stopPropagation();
            if (typeof BlockConnector === 'undefined') return;
            const conn = BlockConnector.connections.get(blockId);
            if (connectorType === 'top' && conn && conn.prev) {
                BlockConnector.disconnectBlocks(conn.prev, blockId);
            } else if (connectorType === 'bottom' && conn && conn.next) {
                const nextArr = Array.isArray(conn.next) ? conn.next : [conn.next];
                nextArr.forEach(nextId => {
                    if (nextId) BlockConnector.disconnectBlocks(blockId, nextId);
                });
            }
        });

        return hit;
    },

    _makeBodyHit(blockData, w, h, isEvent) {
        const hit = new PIXI.Graphics();
        const connInset = 14;
        const y0 = isEvent ? 0 : connInset;
        const y1 = h - connInset;
        const bodyH = y1 - y0;

        if (bodyH >= 10) {
            hit.rect(0, y0, w, bodyH);
        } else {
            const mid = h / 2;
            hit.rect(w * 0.12, mid - 5, w * 0.76, 10);
        }
        hit.fill({ color: 0xffffff, alpha: 0.001 });
        hit.eventMode = 'static';
        hit.cursor = 'move';
        hit.zIndex = 1;

        this._attachBodyInteraction(hit, blockData, blockData.id);
        return hit;
    },

    _blockWidth(blockData) {
        return typeof BlockSystem !== 'undefined' ? BlockSystem.calculateBlockWidth(blockData) : 140;
    },

    _blockHeight(blockData, w) {
        const wrapW = Math.max(w - this.BLOCK_PAD_X * 2, 40);
        let h = this.BLOCK_PAD_Y + 9 + 2 + 12;

        const detail = this._detailFor(blockData);
        if (detail) {
            const charsPerLine = Math.max(6, Math.floor(wrapW / 6.5));
            const lines = Math.min(3, Math.ceil(detail.length / charsPerLine));
            h += 2 + lines * 11;
        }

        h += this.BLOCK_PAD_Y;
        return Math.max(this.BLOCK_MIN_HEIGHT, Math.ceil(h / 4) * 4);
    },

    _ts(size, fillColor, bold, wordWrapWidth) {
        const style = {
            fontSize: size,
            fontFamily: 'ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace',
            fill: fillColor,
            fontWeight: bold ? '700' : '400',
        };
        if (wordWrapWidth != null && wordWrapWidth > 0) {
            style.wordWrap = true;
            style.wordWrapWidth = wordWrapWidth;
            style.breakWords = true;
        }
        return new PIXI.TextStyle(style);
    },

    _createLabels(container, blockData, w, h) {
        const col = this._colorFor(blockData.type);
        const wrapW = w - this.BLOCK_PAD_X * 2;
        let y = this.BLOCK_PAD_Y;

        const idText = new PIXI.Text({
            text: `#${blockData.id}`,
            style: this._ts(8, this._themeColor('labelId', 0x888888), false),
        });
        idText.x = this.BLOCK_PAD_X;
        idText.y = y;
        container.addChild(idText);
        y += 9;

        const titleText = new PIXI.Text({
            text: this._titleFor(blockData),
            style: this._ts(10, col.border, true, wrapW),
        });
        titleText.x = this.BLOCK_PAD_X;
        titleText.y = y;
        container.addChild(titleText);
        y += titleText.height + 2;

        let detailText = null;
        const detail = this._detailFor(blockData);
        if (detail) {
            detailText = new PIXI.Text({
                text: detail,
                style: this._ts(9, this._themeColor('labelDetail', 0xbbbbbb), false, wrapW),
            });
            detailText.x = this.BLOCK_PAD_X;
            detailText.y = y;
            container.addChild(detailText);
        }

        return { idText, titleText, detailText };
    },

    _titleFor(bd) {
        const t = bd.type;
        if (t === 'event') return bd.eventType === 'workflow-complete' ? '⚡ WHEN COMPLETE' : '▶ WHEN START';
        if (t === 'motor') return `MOTOR ${bd.motor_id}`;
        if (t === 'relay') return `RELAY ${bd.relay_id}`;
        if (t === 'delay') return 'DELAY';
        if (t === 'pause') return 'PAUSE';
        if (t === 'repeat') return 'REPEAT';
        if (t === 'forever') return 'FOREVER';
        if (t === 'break') return 'BREAK';
        if (t === 'wait-sensor') return 'WAIT SENSOR';
        if (t === 'read-sensor') return 'READ SENSOR';
        if (t === 'ros-trigger') return 'ROS TRIGGER';
        if (t === 'motor-speed-from-topic') return 'SET SPEED';
        if (t === 'subscribe-motor-speed-topic') return 'SUB SPEED';
        if (t === 'unsubscribe-motor-speed-topic') return 'UNSUB SPEED';
        if (t === 'try') return 'TRY';
        if (t === 'catch') return 'CATCH';
        if (t === 'throw-error') return 'THROW ERROR';
        if (t === 'wait-key') return 'WAIT KEY';
        return t.toUpperCase();
    },

    _detailFor(bd) {
        const t = bd.type;
        if (t === 'motor') {
            const dir = bd.direction === 'backward' ? '←' : '→';
            const steps = Math.abs(bd.steps || 0);
            const hasCustomSpeed = bd.speed !== undefined && bd.speed !== null;
            const speed = hasCustomSpeed ? bd.speed : (typeof MotorSpeedManager !== 'undefined' ? MotorSpeedManager.getSpeed(bd.motor_id) : 100);
            const speedNote = hasCustomSpeed ? `${speed} sps` : `${speed} sps (global)`;
            return `${dir} ${steps} steps · ${speedNote}`;
        }
        if (t === 'relay') return `State: ${(bd.state || 'off').toUpperCase()}`;
        if (t === 'delay') return `Duration: ${bd.duration || 0}s`;
        if (t === 'repeat') return `Repeat: ×${bd.count || 0}`;
        if (t === 'wait-sensor') {
            const parts = [];
            if (bd.sensorId) parts.push(`sensor: ${bd.sensorId}`);
            if (bd.condition) parts.push(bd.condition);
            if (bd.threshold != null) parts.push(`thr: ${bd.threshold}`);
            return parts.join(' · ') || '';
        }
        if (t === 'read-sensor') return bd.sensorId ? `sensor: ${bd.sensorId}` : '';
        if (t === 'ros-trigger') {
            const topic = bd.topic || '';
            const expect = bd.expectedString ? ` · "${bd.expectedString}"` : '';
            return topic + expect;
        }
        if (t === 'motor-speed-from-topic' || t === 'subscribe-motor-speed-topic') {
            return `M${bd.motor_id || 1} · ${bd.topic || ''}`;
        }
        if (t === 'unsubscribe-motor-speed-topic') return `Motor ${bd.motor_id || 1}`;
        if (t === 'wait-key') {
            const kc = (bd.keyCode && String(bd.keyCode).trim()) || 'KeyK';
            const label = (typeof SettingsManager !== 'undefined' && SettingsManager.keyCodeToLabel)
                ? SettingsManager.keyCodeToLabel(kc) : kc.replace('Key', '');
            return `Key: ${label}`;
        }
        if (t === 'throw-error') return bd.errorMessage ? `"${bd.errorMessage}"` : '';
        return '';
    },

    setBlockSelected(blockId, on) {
        if (!this.enabled) return;
        const prev = this._selectedId;
        if (on) {
            this._selectedId = blockId;
        } else if (this._selectedId === blockId) {
            this._selectedId = null;
        }
        if (prev !== null && prev !== this._selectedId) this._redrawBodyState(prev);
        if (blockId !== null && on) this._redrawBodyState(blockId);
    },

    setBlockExecuting(blockId, on) {
        if (!this.enabled) return;
        if (on) this._executingIds.add(blockId); else this._executingIds.delete(blockId);
        this._redrawBodyState(blockId);
        this._updateExecGlow(blockId);
        this._updateActiveConnections(this._executingIds);
    },

    setExecutingBlocks(blockIds) {
        if (!this.enabled) return;
        const next = blockIds instanceof Set ? blockIds : new Set(blockIds);
        const changed = new Set();
        this._executingIds.forEach(id => { if (!next.has(id)) changed.add(id); });
        next.forEach(id => { if (!this._executingIds.has(id)) changed.add(id); });
        this._executingIds = new Set(next);
        changed.forEach(id => {
            this._redrawBodyState(id);
            this._updateExecGlow(id);
        });
        this._updateActiveConnections(next);
    },

    clearAllExecuting() {
        const prev = [...this._executingIds];
        this._executingIds = new Set();
        prev.forEach(id => {
            this._redrawBodyState(id);
            this._updateExecGlow(id);
        });
        this._updateActiveConnections(this._executingIds);
    },

    setBlockError(blockId, on) {
        if (!this.enabled) return;
        if (on) {
            this._errorIds.add(blockId);
            this._warningIds.delete(blockId);
        } else {
            this._errorIds.delete(blockId);
        }
        this._redrawBodyState(blockId);
    },

    setBlockWarning(blockId, on) {
        if (!this.enabled) return;
        if (on) {
            this._warningIds.add(blockId);
            this._errorIds.delete(blockId);
        } else {
            this._warningIds.delete(blockId);
        }
        this._redrawBodyState(blockId);
    },

    clearBlockStates(blockId) {
        if (!this.enabled) return;
        this._errorIds.delete(blockId);
        this._warningIds.delete(blockId);
        this._redrawBodyState(blockId);
    },

    _updateActiveConnections(executingIds) {
        const newActive = new Set();
        if (executingIds.size > 0 && typeof BlockConnector !== 'undefined') {
            BlockConnector.connections.forEach((conn, fromId) => {
                if (!conn.next) return;
                const nextArr = Array.isArray(conn.next) ? conn.next : [conn.next];
                nextArr.forEach(toId => {
                    if (toId && executingIds.has(toId)) {
                        newActive.add(`${fromId}-${toId}`);
                    }
                });
            });
        }

        const touched = new Set([...this._activeConnectionKeys, ...newActive]);
        touched.forEach(key => {
            const was = this._activeConnectionKeys.has(key);
            const now = newActive.has(key);
            if (was === now) return;
            const entry = this.connectionObjects.get(key);
            if (entry) {
                this._drawSegments(
                    entry.visible, entry.segments,
                    this._hoveredConnections.has(key), now
                );
            }
        });
        this._activeConnectionKeys = newActive;
    },

    setBlockSnapping(blockId, on) {
        if (!this.enabled) return;
        if (on) this._snappingIds.add(blockId); else this._snappingIds.delete(blockId);
        this._redrawBodyState(blockId);
    },

    clearAllSnapping() {
        if (!this.enabled) return;
        this._snappingIds.forEach(id => this._redrawBodyState(id));
        this._snappingIds.clear();
    },

    _redrawBodyState(blockId) {
        const obj = this.blockObjects.get(blockId);
        if (!obj) return;
        const bd = WorkflowManager.blocks.get(blockId);
        if (!bd) return;
        this._drawBody(
            obj.body, obj.w, obj.h, bd,
            this._selectedId === blockId,
            this._executingIds.has(blockId),
            this._snappingIds.has(blockId),
            this._errorIds.has(blockId),
            this._warningIds.has(blockId)
        );
    },

    updateConnectorVisualsForBlock(blockId) {
        if (!this.enabled) return;
        const obj = this.blockObjects.get(blockId);
        if (!obj) return;

        const conn = typeof BlockConnector !== 'undefined' ? BlockConnector.connections.get(blockId) : null;
        const connectingFrom = typeof BlockConnector !== 'undefined' ? BlockConnector.connectingFrom : null;

        const topConnected = !!(conn && conn.prev);
        const bottomConnected = !!(conn && conn.next && (Array.isArray(conn.next) ? conn.next.length > 0 : conn.next));
        const topConnecting = !!(connectingFrom && connectingFrom.blockId === blockId && connectingFrom.connectorType === 'top');
        const bottomConnecting = !!(connectingFrom && connectingFrom.blockId === blockId && connectingFrom.connectorType === 'bottom');

        obj.connectorState.top = { connected: topConnected, connecting: topConnecting };
        obj.connectorState.bottom = { connected: bottomConnected, connecting: bottomConnecting };

        const col = this._colorFor((WorkflowManager.blocks.get(blockId) || {}).type);

        if (obj.topDotVisual) {
            this._drawConnectorDot(obj.topDotVisual, col.border, 5, topConnected, topConnecting);
        }
        this._drawConnectorDot(obj.bottomDotVisual, col.border, 5, bottomConnected, bottomConnecting);
    },

    setConnectorConnecting(blockId, connectorType, connecting) {
        if (!this.enabled) return;
        this._connectingHighlight = connecting ? { blockId, connectorType } : null;
        if (blockId != null) {
            this.updateConnectorVisualsForBlock(blockId);
        }
        if (typeof BlockConnector !== 'undefined' && BlockConnector.connectingFrom) {
            this.updateConnectorVisualsForBlock(BlockConnector.connectingFrom.blockId);
        }
    },

    _selectBlock(blockId) {
        if (typeof WorkflowManager !== 'undefined') {
            if (blockId == null) {
                WorkflowManager.deselectBlock();
            } else {
                WorkflowManager.selectBlock(blockId);
            }
            return;
        }

        const prev = this._selectedId;
        this._selectedId = blockId;

        if (prev !== null && prev !== blockId) this._redrawBodyState(prev);
        if (blockId !== null) {
            this._redrawBodyState(blockId);
            if (typeof PixiOverlay !== 'undefined') {
                const bd = WorkflowManager.blocks.get(blockId);
                if (bd) PixiOverlay.show(bd, this);
            }
        } else if (typeof PixiOverlay !== 'undefined') {
            PixiOverlay.hide();
        }
    },

    _attachBodyInteraction(hitTarget, blockData, blockId) {
        const container = () => this.blockObjects.get(blockId)?.container;

        hitTarget.on('pointerdown', (e) => {
            e.stopPropagation();
            const ptr = this._pointerWorld(e);
            this._dragState = {
                blockId,
                offsetX: ptr.x - blockData.x,
                offsetY: ptr.y - blockData.y,
                startX: ptr.x,
                startY: ptr.y,
                didMove: false,
                pushedUndo: false,
            };

            const c = container();
            if (c) {
                this.blockLayer.removeChild(c);
                this.blockLayer.addChild(c);
            }
        });

        hitTarget.on('globalpointermove', (e) => {
            if (!this._dragState || this._dragState.blockId !== blockId) return;

            const ptr = this._pointerWorld(e);
            const dx = ptr.x - this._dragState.startX;
            const dy = ptr.y - this._dragState.startY;

            if (!this._dragState.didMove) {
                if (Math.hypot(dx, dy) < this.TAP_THRESHOLD) return;
                this._dragState.didMove = true;
                if (!this._dragState.pushedUndo && typeof UndoManager !== 'undefined') {
                    UndoManager.pushState();
                    this._dragState.pushedUndo = true;
                }
                if (typeof WorkflowManager !== 'undefined') WorkflowManager.beginBlockDrag(blockId);
            }

            const nx = ptr.x - this._dragState.offsetX;
            const ny = ptr.y - this._dragState.offsetY;
            blockData.x = nx;
            blockData.y = ny;

            const c = container();
            if (c) {
                c.x = nx;
                c.y = ny;
            }

            if (BlockConnector.blockPositionCache) BlockConnector.blockPositionCache.delete(blockId);
            if (BlockConnector.updateSpatialEntry) BlockConnector.updateSpatialEntry(blockId);

            const now = performance.now();
            const throttle = WorkflowManager.dragUpdateThrottle || 50;
            if (now - (WorkflowManager.lastDragUpdate || 0) >= throttle) {
                WorkflowManager.lastDragUpdate = now;
                BlockConnector.checkSnapping(null, blockData);
                WorkflowManager.scheduleConnectionsUpdate(blockId);
            }

            if (typeof PixiOverlay !== 'undefined' && PixiOverlay.isVisible() && PixiOverlay.currentBlockId() === blockId) {
                PixiOverlay.reposition(blockData, this);
            }
        });

        const onUp = (e) => {
            if (!this._dragState || this._dragState.blockId !== blockId) return;

            const wasTap = !this._dragState.didMove;
            this._dragState = null;

            if (wasTap) {
                this._selectBlock(blockId);
                return;
            }

            this.clearAllSnapping();
            if (typeof BlockConnector !== 'undefined') {
                BlockConnector.clearPreviewConnection();
                BlockConnector.previewTarget = null;
                BlockConnector._prevSnappingEls.forEach(el => {
                    if (el && el.classList) el.classList.remove('snapping');
                });
                BlockConnector._prevSnappingEls.clear();
                if (typeof WorkflowManager !== 'undefined') WorkflowManager.endBlockDrag();
                BlockConnector.snapBlocks(null, blockData);
                if (BlockConnector.invalidateSpatialIndex) BlockConnector.invalidateSpatialIndex();
                BlockConnector.updateConnections();
            }

            if (typeof WorkflowManager !== 'undefined') WorkflowManager.scheduleCanvasSizeUpdate();
            if (typeof StorageManager !== 'undefined') StorageManager.autoSave();

            if (typeof PixiOverlay !== 'undefined' && PixiOverlay.isVisible() && PixiOverlay.currentBlockId() === blockId) {
                PixiOverlay.reposition(blockData, this);
            }
        };

        hitTarget.on('pointerup', onUp);
        hitTarget.on('pointerupoutside', onUp);

        hitTarget.on('rightdown', (e) => {
            e.stopPropagation();
            if (blockData.type !== 'event' && typeof UIUtils !== 'undefined') {
                if (UIUtils.confirm(`Create a workflow from block ${blockData.id}?`)) {
                    WorkflowManager.createWorkflowFromBlock(blockData.id);
                }
            }
        });
    },

    getBlockPos(blockId) {
        const obj = this.blockObjects.get(blockId);
        if (obj) {
            return {
                left: obj.container.x, top: obj.container.y, width: obj.w, height: obj.h,
                centerX: obj.container.x + obj.w / 2, centerY: obj.container.y + obj.h / 2,
                bottom: obj.container.y + obj.h, right: obj.container.x + obj.w,
            };
        }
        const block = typeof WorkflowManager !== 'undefined' ? WorkflowManager.blocks.get(blockId) : null;
        if (!block) return null;
        const bw = this._blockWidth(block);
        const bh = this._blockHeight(block, bw);
        return {
            left: block.x, top: block.y, width: bw, height: bh,
            centerX: block.x + bw / 2, centerY: block.y + bh / 2,
            bottom: block.y + bh, right: block.x + bw,
        };
    },

    getBlockScreenPos(blockId) {
        if (!this.enabled || !this._scrollEl) return null;
        const obj = this.blockObjects.get(blockId);
        if (!obj) return null;
        const rect = this._scrollEl.getBoundingClientRect();
        return {
            x: obj.container.x - this._scrollX + rect.left,
            y: obj.container.y - this._scrollY + rect.top,
            w: obj.w,
            h: obj.h,
        };
    },

    // -------------------------------------------------------------------------
    // Interactive connections
    // -------------------------------------------------------------------------

    _bindConnectionHitEvents(hit, key, fromBlockId, toBlockId) {
        const disconnect = () => {
            if (typeof BlockConnector !== 'undefined') {
                BlockConnector.disconnectBlocks(fromBlockId, toBlockId);
            }
        };

        hit.on('pointerover', () => this.setConnectionHover(key, true));
        hit.on('pointerout', () => this.setConnectionHover(key, false));
        hit.on('pointertap', (e) => {
            e.stopPropagation();
            if (typeof BlockConnector === 'undefined') return;
            if (BlockConnector.autoWireEnabled) {
                disconnect();
            } else {
                const ptr = this._pointerWorld(e);
                BlockConnector.addWaypoint(fromBlockId, toBlockId, ptr.x, ptr.y);
            }
        });
        hit.on('rightclick', (e) => {
            e.stopPropagation();
            disconnect();
        });
        hit.on('pointerdown', (e) => {
            const native = e.nativeEvent || e.originalEvent || e;
            if (native && native.button === 2) {
                e.stopPropagation();
                disconnect();
            }
        });
    },

    findConnectionAt(worldX, worldY, thresholdPx) {
        const t = thresholdPx || 8;
        let best = null;
        let bestDist = t;

        this.connectionObjects.forEach((entry, key) => {
            if (!entry.segments) return;
            entry.segments.forEach(seg => {
                const cx = seg.x + seg.width / 2;
                const cy = seg.y + seg.height / 2;
                const dx = Math.max(seg.x - worldX, 0, worldX - (seg.x + seg.width));
                const dy = Math.max(seg.y - worldY, 0, worldY - (seg.y + seg.height));
                const dist = Math.hypot(dx, dy);
                if (dist < bestDist) {
                    bestDist = dist;
                    best = { key, fromBlockId: entry.fromBlockId, toBlockId: entry.toBlockId };
                }
            });
        });
        return best;
    },

    setConnection(key, segments, fromBlockId, toBlockId, waypoints) {
        if (!this.enabled) return;

        let entry = this.connectionObjects.get(key);
        if (!entry) {
            const container = new PIXI.Container();
            container.eventMode = 'passive';
            const visible = new PIXI.Graphics();
            container.addChild(visible);
            this.connectionLayer.addChild(container);
            entry = { container, visible, hitAreas: [], waypointGfx: null, segments: [], fromBlockId, toBlockId };
            this.connectionObjects.set(key, entry);
        }

        entry.fromBlockId = fromBlockId;
        entry.toBlockId = toBlockId;
        entry.segments = segments || [];
        entry.waypoints = waypoints && waypoints.length > 0 ? waypoints : null;

        this._drawSegments(entry.visible, entry.segments, this._hoveredConnections.has(key), this._activeConnectionKeys.has(key));
        this._syncConnectionHitAreas(entry, key, fromBlockId, toBlockId);

        if (entry.waypointGfx) {
            try { entry.waypointGfx.destroy(); } catch (_) {}
            entry.waypointGfx = null;
        }
        if (entry.waypoints && entry.waypoints.length > 0) {
            const wg = new PIXI.Graphics();
            this._drawWaypoints(wg, entry.waypoints);
            entry.container.addChild(wg);
            entry.waypointGfx = wg;
        }
    },

    _syncConnectionHitAreas(entry, key, fromBlockId, toBlockId) {
        const pad = 6;
        const segCount = entry.segments.length;

        while (entry.hitAreas.length > segCount) {
            const extra = entry.hitAreas.pop();
            entry.container.removeChild(extra);
            try { extra.destroy(); } catch (_) {}
        }

        entry.segments.forEach((segment, index) => {
            const isV = segment.width <= segment.height;
            let hx, hy, hw, hh;
            if (isV) {
                hx = segment.x - pad;
                hy = segment.y;
                hw = Math.max(segment.width, 2) + pad * 2;
                hh = segment.height;
            } else {
                hx = segment.x;
                hy = segment.y - pad;
                hw = segment.width;
                hh = Math.max(segment.height, 2) + pad * 2;
            }

            let hit = entry.hitAreas[index];
            if (!hit) {
                hit = new PIXI.Graphics();
                hit.eventMode = 'static';
                hit.cursor = 'pointer';
                this._bindConnectionHitEvents(hit, key, fromBlockId, toBlockId);
                entry.container.addChild(hit);
                entry.hitAreas[index] = hit;
            } else {
                hit.clear();
            }
            hit.rect(hx, hy, hw, hh);
            hit.fill({ color: 0xffffff, alpha: 0.001 });
        });
    },

    _drawSegments(g, segments, hovered, active) {
        g.clear();
        if (!segments || segments.length === 0) return;
        const color = hovered ? this._themeColor('connHover', 0xef4444)
            : (active ? this._themeColor('connActive', 0x10b981) : this._themeColor('connLine', 0x3b82f6));
        const thick = hovered ? 4 : (active ? 3 : 2);
        const alpha = active && !hovered ? 0.95 : 1;
        segments.forEach(seg => {
            const isV = seg.width <= seg.height;
            const x = isV ? seg.x + (seg.width - thick) / 2 : seg.x;
            const y = isV ? seg.y : seg.y + (seg.height - thick) / 2;
            const sw = isV ? thick : seg.width;
            const sh = isV ? seg.height : thick;
            if (sw > 0 && sh > 0) { g.rect(x, y, sw, sh); g.fill({ color, alpha }); }
        });
    },

    removeConnection(key) {
        if (!this.enabled) return;
        const entry = this.connectionObjects.get(key);
        if (entry) {
            this.connectionLayer.removeChild(entry.container);
            try { entry.container.destroy({ children: true }); } catch (_) {}
            this.connectionObjects.delete(key);
            this._hoveredConnections.delete(key);
        }
    },

    setConnectionHover(key, isHovered) {
        if (!this.enabled) return;
        if (isHovered) this._hoveredConnections.add(key); else this._hoveredConnections.delete(key);
        const entry = this.connectionObjects.get(key);
        if (entry && entry.segments) {
            this._drawSegments(entry.visible, entry.segments, isHovered, this._activeConnectionKeys.has(key));
        }
    },

    setPreview(segments) {
        if (!this.enabled || !this._previewGraphics) return;
        this._lastPreviewSegments = segments || null;
        const g = this._previewGraphics;
        g.clear();
        const previewColor = this._themeColor('connPreview', 0x10b981);
        const thick = 3;
        (segments || []).forEach(seg => {
            const isV = seg.width <= seg.height;
            const x = isV ? seg.x + (seg.width - thick) / 2 : seg.x;
            const y = isV ? seg.y : seg.y + (seg.height - thick) / 2;
            const sw = isV ? thick : seg.width;
            const sh = isV ? seg.height : thick;
            if (sw > 0 && sh > 0) { g.rect(x, y, sw, sh); g.fill({ color: previewColor, alpha: 0.9 }); }
        });
    },

    clearPreview() {
        this._lastPreviewSegments = null;
        if (this._previewGraphics) this._previewGraphics.clear();
    },

    setTriggerLinks(links) {
        if (!this.enabled || !this._triggerGraphics) return;
        this._lastTriggerLinks = links || null;
        const g = this._triggerGraphics;
        g.clear();
        if (!links || links.length === 0) return;
        const linkColor = this._themeColor('triggerLink', 0x8b5cf6);
        links.forEach(link => {
            const dx = link.toX - link.fromX;
            const dy = link.toY - link.fromY;
            const len = Math.sqrt(dx * dx + dy * dy);
            if (len < 4) return;
            const nx = dx / len;
            const ny = dy / len;
            const dash = 10;
            const gap = 7;
            const stopLen = len - 12;
            for (let s = 0; s < stopLen; s += dash + gap) {
                const endS = Math.min(s + dash, stopLen);
                g.moveTo(link.fromX + nx * s, link.fromY + ny * s);
                g.lineTo(link.fromX + nx * endS, link.fromY + ny * endS);
            }
            g.stroke({ color: linkColor, width: 2, alpha: 0.7 });

            const ax = link.toX;
            const ay = link.toY;
            const angle = link.angle !== undefined ? link.angle : Math.atan2(dy, dx);
            const size = 9;
            const bx = ax - Math.cos(angle) * size;
            const by = ay - Math.sin(angle) * size;
            const sx = -Math.sin(angle) * size * 0.55;
            const sy = Math.cos(angle) * size * 0.55;
            g.moveTo(ax, ay);
            g.lineTo(bx + sx, by + sy);
            g.lineTo(bx - sx, by - sy);
            g.closePath();
            g.fill({ color: linkColor, alpha: 0.7 });
        });
    },

    clearConnections() {
        if (!this.enabled) return;
        this.connectionObjects.forEach(entry => {
            this.connectionLayer.removeChild(entry.container);
            try { entry.container.destroy({ children: true }); } catch (_) {}
        });
        this.connectionObjects.clear();
        this._hoveredConnections.clear();
        if (this._previewGraphics) this._previewGraphics.clear();
        if (this._triggerGraphics) this._triggerGraphics.clear();
    },

    clear() {
        if (!this.enabled) return;
        this.blockObjects.forEach((_, id) => this._destroyBlockObject(id));
        this.blockObjects.clear();
        this.clearConnections();
        this._selectedId = null;
        this._executingIds.clear();
        this._errorIds.clear();
        this._warningIds.clear();
        this._activeConnectionKeys.clear();
        this._snappingIds.clear();
        this._dragState = null;
        if (typeof PixiOverlay !== 'undefined') PixiOverlay.hide();
    },
};
