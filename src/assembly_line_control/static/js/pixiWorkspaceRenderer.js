/**
 * PixiWorkspaceRenderer - GPU-accelerated workspace renderer using PixiJS v8.
 *
 * Draws block shells, labels, connector affordances, selection/snap/executing
 * highlights, and connection lines via PixiJS. A thin DOM overlay (PixiOverlay)
 * provides parameter editing controls for the selected block only.
 *
 * Enabled when Config.PIXI_WORKSPACE is true (or ?pixi=1 URL param).
 * Falls back gracefully to the existing DOM+WebGL renderer when disabled or
 * when WebGL is unavailable on the device.
 */
const PixiWorkspaceRenderer = {

    app: null,
    stage: null,
    connectionLayer: null,
    blockLayer: null,
    uiLayer: null,

    blockObjects: new Map(),       // blockId -> { container, body, w, h }
    connectionObjects: new Map(),  // connKey -> PIXI.Graphics (with ._segments cache)
    _previewGraphics: null,
    _triggerGraphics: null,
    _hoveredConnections: new Set(),

    _scrollEl: null,
    _scrollX: 0,
    _scrollY: 0,

    _dragState: null,      // { blockId, offsetX, offsetY } or null
    _snappingIds: new Set(),
    _executingIds: new Set(),
    _selectedId: null,

    enabled: false,

    // -------------------------------------------------------------------------
    // Color palette matching CSS block type classes
    // -------------------------------------------------------------------------
    _COLORS: {
        event:                          { bg: 0x0a2218, bgA: 0.92, border: 0x10b981 },
        motor:                          { bg: 0x0a1a2e, bgA: 0.92, border: 0x3b82f6 },
        'motor-speed-from-topic':       { bg: 0x0a1a2e, bgA: 0.92, border: 0x3b82f6 },
        'subscribe-motor-speed-topic':  { bg: 0x0a1a2e, bgA: 0.92, border: 0x3b82f6 },
        'unsubscribe-motor-speed-topic':{ bg: 0x0a1a2e, bgA: 0.92, border: 0x3b82f6 },
        relay:                          { bg: 0x2a1e00, bgA: 0.92, border: 0xf59e0b },
        pause:                          { bg: 0x2a0808, bgA: 0.92, border: 0xef4444 },
        delay:                          { bg: 0x160828, bgA: 0.92, border: 0x8b5cf6 },
        repeat:                         { bg: 0x28081a, bgA: 0.92, border: 0xec4899 },
        forever:                        { bg: 0x28081a, bgA: 0.92, border: 0xec4899 },
        break:                          { bg: 0x28081a, bgA: 0.92, border: 0xec4899 },
        'wait-sensor':                  { bg: 0x001414, bgA: 0.92, border: 0x06b6d4 },
        'read-sensor':                  { bg: 0x001414, bgA: 0.92, border: 0x06b6d4 },
        'ros-trigger':                  { bg: 0x001414, bgA: 0.92, border: 0x06b6d4 },
        'throw-error':                  { bg: 0x1a0e00, bgA: 0.92, border: 0xf97316 },
        try:                            { bg: 0x1a0e00, bgA: 0.92, border: 0xf97316 },
        catch:                          { bg: 0x1a0e00, bgA: 0.92, border: 0xf97316 },
        'wait-key':                     { bg: 0x16082a, bgA: 0.92, border: 0x8b5cf6 },
    },

    _colorFor(type) {
        return this._COLORS[type] || { bg: 0x1a1a1a, bgA: 0.92, border: 0x555555 };
    },

    // -------------------------------------------------------------------------
    // Initialisation (async - Pixi v8 requires await app.init())
    // -------------------------------------------------------------------------

    async init(scrollEl) {
        if (!scrollEl) return false;
        if (typeof PIXI === 'undefined') {
            console.warn('[Pixi] PIXI not loaded, falling back to DOM renderer');
            return false;
        }

        this._scrollEl = scrollEl;

        // Test WebGL availability before committing
        try {
            const t = document.createElement('canvas');
            const gl = t.getContext('webgl') || t.getContext('experimental-webgl');
            if (!gl) throw new Error('no-webgl');
        } catch (_) {
            console.warn('[Pixi] WebGL unavailable, using DOM renderer');
            return false;
        }

        const rect = scrollEl.getBoundingClientRect();
        const w = Math.max(rect.width || 800, 100);
        const h = Math.max(rect.height || 600, 100);

        try {
            this.app = new PIXI.Application();
            await this.app.init({
                width: w,
                height: h,
                backgroundAlpha: 0,
                antialias: false,
                resolution: Math.min(window.devicePixelRatio || 1, 2),
                autoDensity: true,
            });
        } catch (err) {
            console.warn('[Pixi] Application init failed:', err);
            return false;
        }

        // Mount canvas inside the scroll container, above the WebGL connection layer (z-index 4)
        const pixiCanvas = this.app.canvas;
        pixiCanvas.style.cssText = 'position:absolute;left:0;top:0;z-index:6;pointer-events:auto;';
        pixiCanvas.style.width = w + 'px';
        pixiCanvas.style.height = h + 'px';
        pixiCanvas.setAttribute('aria-hidden', 'true');
        scrollEl.appendChild(pixiCanvas);

        // Layer order: connections → blocks → UI (snap/preview overlays)
        this.stage = this.app.stage;
        this.connectionLayer = new PIXI.Container();
        this.blockLayer = new PIXI.Container();
        this.uiLayer = new PIXI.Container();
        this.stage.addChild(this.connectionLayer);
        this.stage.addChild(this.blockLayer);
        this.stage.addChild(this.uiLayer);

        // Persistent overlay graphics objects
        this._previewGraphics = new PIXI.Graphics();
        this._triggerGraphics = new PIXI.Graphics();
        this.uiLayer.addChild(this._previewGraphics);
        this.uiLayer.addChild(this._triggerGraphics);

        // Deselect on empty canvas click
        pixiCanvas.addEventListener('pointerdown', (e) => {
            if (e.target === pixiCanvas) {
                this._selectBlock(null);
            }
        });

        // Scroll sync: shift stage so blocks stay in world coordinates
        scrollEl.addEventListener('scroll', () => {
            this._scrollX = scrollEl.scrollLeft;
            this._scrollY = scrollEl.scrollTop;
            this.stage.x = -this._scrollX;
            this.stage.y = -this._scrollY;
        }, { passive: true });

        // Resize sync
        try {
            new ResizeObserver(() => this._resize()).observe(scrollEl);
        } catch (_) {
            window.addEventListener('resize', () => this._resize());
        }

        this.enabled = true;
        if (typeof UIUtils !== 'undefined') {
            UIUtils.log('[Pixi] PixiJS workspace renderer initialized', 'success');
        }
        return true;
    },

    _resize() {
        if (!this.app || !this._scrollEl) return;
        const rect = this._scrollEl.getBoundingClientRect();
        const w = Math.max(rect.width || 1, 1);
        const h = Math.max(rect.height || 1, 1);
        try { this.app.renderer.resize(w, h); } catch (_) {}
        const c = this.app.canvas;
        c.style.width = w + 'px';
        c.style.height = h + 'px';
    },

    // -------------------------------------------------------------------------
    // Block management
    // -------------------------------------------------------------------------

    addBlock(blockData) {
        if (!this.enabled) return;

        this._destroyBlockObject(blockData.id);

        const w = typeof BlockSystem !== 'undefined' ? BlockSystem.calculateBlockWidth(blockData) : 140;
        const h = typeof BlockSystem !== 'undefined' ? BlockSystem.calculateBlockHeight(blockData) : 100;
        const col = this._colorFor(blockData.type);
        const isEvent = blockData.type === 'event';

        const container = new PIXI.Container();
        container.eventMode = 'static';
        container.cursor = 'move';
        container.x = blockData.x;
        container.y = blockData.y;

        // Background body
        const body = new PIXI.Graphics();
        this._drawBody(body, w, h, blockData, false, false, false);
        container.addChild(body);

        // Connector dots (visual affordance only; click is detected by zone)
        if (!isEvent) {
            const td = this._makeDot(col.border);
            td.x = Math.round(w / 2);
            td.y = 0;
            container.addChild(td);
        }
        const bd = this._makeDot(col.border);
        bd.x = Math.round(w / 2);
        bd.y = h;
        container.addChild(bd);

        // Text labels
        this._addLabels(container, blockData, w, h);

        // Pointer interaction
        this._attachInteraction(container, blockData, w, h);

        this.blockLayer.addChild(container);
        this.blockObjects.set(blockData.id, { container, body, w, h });
    },

    removeBlock(blockId) {
        if (!this.enabled) return;
        this._destroyBlockObject(blockId);
    },

    updateBlock(blockData) {
        if (!this.enabled) return;
        // Preserve selection/exec state across redraw
        const wasSelected = this._selectedId === blockData.id;
        const wasExecuting = this._executingIds.has(blockData.id);
        this.addBlock(blockData);
        if (wasSelected) this._redrawBodyState(blockData.id);
        if (wasExecuting) this.setBlockExecuting(blockData.id, true);
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

    _drawBody(g, w, h, blockData, selected, executing, snapping) {
        g.clear();
        const col = this._colorFor(blockData.type);
        const isEvent = blockData.type === 'event';
        const radius = isEvent ? 8 : 6;

        let borderColor = col.border;
        let borderWidth = 2;
        if (snapping)  { borderColor = 0x10b981; borderWidth = 2; }
        if (executing) { borderColor = 0x10b981; borderWidth = 3; }
        if (selected)  { borderColor = 0x3b82f6; borderWidth = 3; }

        g.roundRect(0, 0, w, h, radius);
        g.fill({ color: col.bg, alpha: col.bgA });
        g.stroke({ color: borderColor, width: borderWidth });
    },

    _makeDot(color) {
        const g = new PIXI.Graphics();
        g.circle(0, 0, 5);
        g.fill({ color, alpha: 0.8 });
        return g;
    },

    _ts(size, fillColor, bold) {
        return new PIXI.TextStyle({
            fontSize: size,
            fontFamily: 'ui-monospace,SFMono-Regular,Menlo,Monaco,Consolas,monospace',
            fill: fillColor,
            fontWeight: bold ? '700' : '400',
        });
    },

    _addLabels(container, blockData, w, h) {
        const col = this._colorFor(blockData.type);

        // Block ID badge
        container.addChild(Object.assign(
            new PIXI.Text({ text: `#${blockData.id}`, style: this._ts(9, 0x888888, false) }),
            { x: 8, y: 8 }
        ));

        // Title (accent colour)
        container.addChild(Object.assign(
            new PIXI.Text({ text: this._titleFor(blockData), style: this._ts(11, col.border, true) }),
            { x: 8, y: 22 }
        ));

        // Detail line
        const detail = this._detailFor(blockData);
        if (detail) {
            container.addChild(Object.assign(
                new PIXI.Text({ text: detail, style: this._ts(10, 0xbbbbbb, false) }),
                { x: 8, y: 40 }
            ));
        }

        // Hint at bottom
        const hint = new PIXI.Text({ text: 'tap to edit', style: this._ts(8, 0x444444, false) });
        hint.x = 8;
        hint.y = Math.max(h - 18, 56);
        hint.name = 'hint';
        container.addChild(hint);
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
        if (t === 'try') return 'TRY'; if (t === 'catch') return 'CATCH';
        if (t === 'throw-error') return 'THROW ERROR';
        if (t === 'wait-key') return 'WAIT KEY';
        return t.toUpperCase();
    },

    _detailFor(bd) {
        const t = bd.type;
        if (t === 'motor') return `${bd.direction === 'backward' ? '← ' : '→ '}${bd.steps || 0} steps`;
        if (t === 'relay') return `${(bd.state || 'off').toUpperCase()}`;
        if (t === 'delay') return `${bd.duration || 0}s`;
        if (t === 'repeat') return `×${bd.count || 0}`;
        if (t === 'wait-sensor') return bd.sensorId ? `id: ${bd.sensorId}` : '';
        if (t === 'ros-trigger') return bd.topic || '';
        if (t === 'motor-speed-from-topic' || t === 'subscribe-motor-speed-topic')
            return `M${bd.motor_id || 1}: ${(bd.topic || '').split('/').pop()}`;
        if (t === 'unsubscribe-motor-speed-topic') return `Motor ${bd.motor_id || 1}`;
        if (t === 'wait-key') return `KEY: ${(bd.keyCode || 'KeyK').replace('Key', '')}`;
        return '';
    },

    // -------------------------------------------------------------------------
    // Visual state
    // -------------------------------------------------------------------------

    setBlockSelected(blockId, on) {
        if (!this.enabled) return;
        const prev = this._selectedId;
        this._selectedId = on ? blockId : (this._selectedId === blockId ? null : this._selectedId);
        if (prev !== null && prev !== blockId) this._redrawBodyState(prev);
        if (blockId !== null) this._redrawBodyState(blockId);
    },

    setBlockExecuting(blockId, on) {
        if (!this.enabled) return;
        if (on) this._executingIds.add(blockId); else this._executingIds.delete(blockId);
        this._redrawBodyState(blockId);
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
            this._snappingIds.has(blockId)
        );
    },

    _selectBlock(blockId) {
        const prev = this._selectedId;
        this._selectedId = blockId;
        if (prev !== null && prev !== blockId) this._redrawBodyState(prev);
        if (blockId !== null) {
            this._redrawBodyState(blockId);
            if (typeof PixiOverlay !== 'undefined') {
                const bd = WorkflowManager.blocks.get(blockId);
                if (bd) PixiOverlay.show(bd, this);
            }
        } else {
            if (typeof PixiOverlay !== 'undefined') PixiOverlay.hide();
        }
    },

    // -------------------------------------------------------------------------
    // Drag interaction
    // -------------------------------------------------------------------------

    _attachInteraction(container, blockData, w, h) {
        container.on('pointerdown', (e) => {
            e.stopPropagation();

            // Connector zone – delegate to BlockConnector
            const local = e.getLocalPosition(container);
            const zone = 16;
            if (local.y <= zone && blockData.type !== 'event') {
                BlockConnector.handleConnectorClick(blockData.id, 'top');
                return;
            }
            if (local.y >= h - zone) {
                BlockConnector.handleConnectorClick(blockData.id, 'bottom');
                return;
            }

            if (typeof UndoManager !== 'undefined') UndoManager.pushState();

            this._dragState = {
                blockId: blockData.id,
                offsetX: e.global.x + this._scrollX - blockData.x,
                offsetY: e.global.y + this._scrollY - blockData.y,
            };

            // Bring to front
            this.blockLayer.removeChild(container);
            this.blockLayer.addChild(container);

            this._selectBlock(blockData.id);
        });

        container.on('pointermove', (e) => {
            if (!this._dragState || this._dragState.blockId !== blockData.id) return;

            const rawX = e.global.x + this._scrollX - this._dragState.offsetX;
            const rawY = e.global.y + this._scrollY - this._dragState.offsetY;
            const nx = typeof WorkflowManager !== 'undefined' ? WorkflowManager.snapToGrid(rawX) : Math.round(rawX / 20) * 20;
            const ny = typeof WorkflowManager !== 'undefined' ? WorkflowManager.snapToGrid(rawY) : Math.round(rawY / 20) * 20;

            blockData.x = nx;
            blockData.y = ny;
            container.x = nx;
            container.y = ny;

            if (BlockConnector.blockPositionCache) BlockConnector.blockPositionCache.delete(blockData.id);
            if (BlockConnector.invalidateSpatialIndex) BlockConnector.invalidateSpatialIndex();

            const now = performance.now();
            const throttle = WorkflowManager.dragUpdateThrottle || 50;
            if (now - (WorkflowManager.lastDragUpdate || 0) >= throttle) {
                WorkflowManager.lastDragUpdate = now;
                BlockConnector.checkSnapping(null, blockData);
                WorkflowManager.scheduleConnectionsUpdate(blockData.id);
            }

            if (typeof PixiOverlay !== 'undefined') PixiOverlay.reposition(blockData, this);
        });

        const onUp = () => {
            if (!this._dragState || this._dragState.blockId !== blockData.id) return;
            this._dragState = null;

            this.clearAllSnapping();

            if (typeof BlockConnector !== 'undefined') {
                BlockConnector.clearPreviewConnection();
                BlockConnector.previewTarget = null;
                BlockConnector._prevSnappingEls.forEach(el => {
                    if (el && el.classList) el.classList.remove('snapping');
                });
                BlockConnector._prevSnappingEls.clear();
                BlockConnector.snapBlocks(null, blockData);
                if (BlockConnector.invalidateSpatialIndex) BlockConnector.invalidateSpatialIndex();
                BlockConnector.updateConnections();
            }

            if (typeof WorkflowManager !== 'undefined') WorkflowManager.scheduleCanvasSizeUpdate();
            if (typeof StorageManager !== 'undefined') StorageManager.autoSave();
        };

        container.on('pointerup', onUp);
        container.on('pointerupoutside', onUp);

        container.on('rightdown', (e) => {
            e.stopPropagation();
            if (blockData.type !== 'event' && typeof UIUtils !== 'undefined') {
                if (UIUtils.confirm(`Create a workflow from block ${blockData.id}?`)) {
                    WorkflowManager.createWorkflowFromBlock(blockData.id);
                }
            }
        });
    },

    // -------------------------------------------------------------------------
    // Connections
    // -------------------------------------------------------------------------

    setConnection(key, segments) {
        if (!this.enabled) return;
        let g = this.connectionObjects.get(key);
        if (!g) {
            g = new PIXI.Graphics();
            this.connectionLayer.addChild(g);
            this.connectionObjects.set(key, g);
        }
        g._segments = segments;
        this._drawSegments(g, segments, this._hoveredConnections.has(key));
    },

    _drawSegments(g, segments, hovered) {
        g.clear();
        if (!segments || segments.length === 0) return;
        const color = hovered ? 0xef4444 : 0x3b82f6;
        const thick = hovered ? 4 : 2;
        segments.forEach(seg => {
            const isV = seg.width <= seg.height;
            const x = isV ? seg.x + (seg.width - thick) / 2 : seg.x;
            const y = isV ? seg.y : seg.y + (seg.height - thick) / 2;
            const sw = isV ? thick : seg.width;
            const sh = isV ? seg.height : thick;
            if (sw > 0 && sh > 0) { g.rect(x, y, sw, sh); g.fill({ color }); }
        });
    },

    removeConnection(key) {
        if (!this.enabled) return;
        const g = this.connectionObjects.get(key);
        if (g) {
            this.connectionLayer.removeChild(g);
            try { g.destroy(); } catch (_) {}
            this.connectionObjects.delete(key);
        }
    },

    setConnectionHover(key, isHovered) {
        if (!this.enabled) return;
        if (isHovered) this._hoveredConnections.add(key); else this._hoveredConnections.delete(key);
        const g = this.connectionObjects.get(key);
        if (g && g._segments) this._drawSegments(g, g._segments, isHovered);
    },

    setPreview(segments) {
        if (!this.enabled || !this._previewGraphics) return;
        const g = this._previewGraphics;
        g.clear();
        const thick = 3;
        (segments || []).forEach(seg => {
            const isV = seg.width <= seg.height;
            const x = isV ? seg.x + (seg.width - thick) / 2 : seg.x;
            const y = isV ? seg.y : seg.y + (seg.height - thick) / 2;
            const sw = isV ? thick : seg.width;
            const sh = isV ? seg.height : thick;
            if (sw > 0 && sh > 0) { g.rect(x, y, sw, sh); g.fill({ color: 0x10b981, alpha: 0.9 }); }
        });
    },

    clearPreview() {
        if (this._previewGraphics) this._previewGraphics.clear();
    },

    setTriggerLinks(links) {
        if (!this.enabled || !this._triggerGraphics) return;
        const g = this._triggerGraphics;
        g.clear();
        if (!links || links.length === 0) return;
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
            g.stroke({ color: 0x8b5cf6, width: 2, alpha: 0.7 });

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
            g.fill({ color: 0x8b5cf6, alpha: 0.7 });
        });
    },

    clearConnections() {
        if (!this.enabled) return;
        this.connectionObjects.forEach(g => {
            this.connectionLayer.removeChild(g);
            try { g.destroy(); } catch (_) {}
        });
        this.connectionObjects.clear();
        this._hoveredConnections.clear();
        if (this._previewGraphics) this._previewGraphics.clear();
        if (this._triggerGraphics) this._triggerGraphics.clear();
    },

    // -------------------------------------------------------------------------
    // Bulk operations
    // -------------------------------------------------------------------------

    clear() {
        if (!this.enabled) return;
        this.blockObjects.forEach((_, id) => this._destroyBlockObject(id));
        this.blockObjects.clear();
        this.clearConnections();
        this._selectedId = null;
        this._executingIds.clear();
        this._snappingIds.clear();
        this._dragState = null;
    },

    // -------------------------------------------------------------------------
    // Coordinate helpers
    // -------------------------------------------------------------------------

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

    // Returns canvas-coordinate position struct (same shape as BlockConnector position objects).
    // Used by BlockConnector spatial index and snap logic when in Pixi mode.
    getBlockPos(blockId) {
        const obj = this.blockObjects.get(blockId);
        if (obj) {
            return {
                left: obj.container.x, top: obj.container.y,
                width: obj.w, height: obj.h,
                centerX: obj.container.x + obj.w / 2,
                centerY: obj.container.y + obj.h / 2,
                bottom: obj.container.y + obj.h,
                right: obj.container.x + obj.w,
            };
        }
        // Fallback: compute from block data if Pixi object not yet created
        const block = typeof WorkflowManager !== 'undefined' ? WorkflowManager.blocks.get(blockId) : null;
        if (!block) return null;
        const bw = typeof BlockSystem !== 'undefined' ? BlockSystem.calculateBlockWidth(block) : 140;
        const bh = typeof BlockSystem !== 'undefined' ? BlockSystem.calculateBlockHeight(block) : 100;
        return {
            left: block.x, top: block.y, width: bw, height: bh,
            centerX: block.x + bw / 2, centerY: block.y + bh / 2,
            bottom: block.y + bh, right: block.x + bw,
        };
    },
};
