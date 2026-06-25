/**
 * WebGL Connection Renderer
 *
 * Draws workspace connection lines in one GPU-backed layer while the existing
 * DOM blocks and transparent hit targets continue handling user interaction.
 */
const WebGLConnectionRenderer = {
    canvas: null,
    gl: null,
    program: null,
    positionBuffer: null,
    colorBuffer: null,
    positionLocation: -1,
    colorLocation: -1,
    resolutionLocation: null,
    connections: new Map(),
    hoveredConnections: new Set(),
    triggerLinks: [],
    previewSegments: [],
    enabled: false,
    needsRender: false,
    lastWidth: 0,
    lastHeight: 0,
    dpr: 1,

    attach(workspaceCanvas) {
        if (!workspaceCanvas) return false;

        if (this.canvas && this.canvas.parentNode !== workspaceCanvas) {
            this.canvas.remove();
            this.canvas = null;
            this.gl = null;
            this.enabled = false;
        }

        if (!this.canvas) {
            this.canvas = document.createElement('canvas');
            this.canvas.className = 'workspace-webgl-layer';
            this.canvas.setAttribute('aria-hidden', 'true');
            workspaceCanvas.insertBefore(this.canvas, workspaceCanvas.firstChild);
        }

        if (!this.gl) {
            this.enabled = this.initGL();
        }

        this.resize();
        this.requestRender();
        return this.enabled;
    },

    initGL() {
        const gl = this.canvas.getContext('webgl', {
            alpha: true,
            antialias: false,
            depth: false,
            stencil: false,
            preserveDrawingBuffer: false
        });

        if (!gl) {
            this.canvas.style.display = 'none';
            return false;
        }

        this.gl = gl;
        const vertexShader = this.createShader(gl.VERTEX_SHADER, `
            attribute vec2 a_position;
            attribute vec4 a_color;
            uniform vec2 u_resolution;
            varying vec4 v_color;

            void main() {
                vec2 zeroToOne = a_position / u_resolution;
                vec2 zeroToTwo = zeroToOne * 2.0;
                vec2 clipSpace = zeroToTwo - 1.0;
                gl_Position = vec4(clipSpace * vec2(1.0, -1.0), 0.0, 1.0);
                v_color = a_color;
            }
        `);
        const fragmentShader = this.createShader(gl.FRAGMENT_SHADER, `
            precision mediump float;
            varying vec4 v_color;

            void main() {
                gl_FragColor = v_color;
            }
        `);

        if (!vertexShader || !fragmentShader) return false;

        this.program = this.createProgram(vertexShader, fragmentShader);
        if (!this.program) return false;

        this.positionLocation = gl.getAttribLocation(this.program, 'a_position');
        this.colorLocation = gl.getAttribLocation(this.program, 'a_color');
        this.resolutionLocation = gl.getUniformLocation(this.program, 'u_resolution');
        this.positionBuffer = gl.createBuffer();
        this.colorBuffer = gl.createBuffer();
        gl.enable(gl.BLEND);
        gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
        return true;
    },

    createShader(type, source) {
        const gl = this.gl || this.canvas.getContext('webgl');
        const shader = gl.createShader(type);
        gl.shaderSource(shader, source);
        gl.compileShader(shader);
        if (!gl.getShaderParameter(shader, gl.COMPILE_STATUS)) {
            console.warn('[WebGLConnectionRenderer] Shader compile failed:', gl.getShaderInfoLog(shader));
            gl.deleteShader(shader);
            return null;
        }
        return shader;
    },

    createProgram(vertexShader, fragmentShader) {
        const gl = this.canvas.getContext('webgl');
        const program = gl.createProgram();
        gl.attachShader(program, vertexShader);
        gl.attachShader(program, fragmentShader);
        gl.linkProgram(program);
        if (!gl.getProgramParameter(program, gl.LINK_STATUS)) {
            console.warn('[WebGLConnectionRenderer] Program link failed:', gl.getProgramInfoLog(program));
            gl.deleteProgram(program);
            return null;
        }
        return program;
    },

    resize() {
        if (!this.canvas || !this.canvas.parentElement) return;

        const parent = this.canvas.parentElement;
        const contentWrapper = parent.querySelector('.canvas-content-wrapper');
        const contentWidth = contentWrapper ? parseFloat(contentWrapper.style.width) : 0;
        const contentHeight = contentWrapper ? parseFloat(contentWrapper.style.height) : 0;
        const width = Math.max(contentWidth || 0, parent.clientWidth, 1);
        const height = Math.max(contentHeight || 0, parent.clientHeight, 1);
        this.dpr = Math.max(1, Math.min(window.devicePixelRatio || 1, 2));

        this.canvas.style.width = width + 'px';
        this.canvas.style.height = height + 'px';

        const bufferWidth = Math.max(1, Math.floor(width * this.dpr));
        const bufferHeight = Math.max(1, Math.floor(height * this.dpr));
        if (this.canvas.width !== bufferWidth || this.canvas.height !== bufferHeight) {
            this.canvas.width = bufferWidth;
            this.canvas.height = bufferHeight;
        }

        if (width !== this.lastWidth || height !== this.lastHeight) {
            this.lastWidth = width;
            this.lastHeight = height;
            this.requestRender();
        }
    },

    setConnection(key, segments) {
        if (!key) return;
        this.connections.set(key, segments || []);
        this.requestRender();
    },

    removeConnection(key) {
        this.connections.delete(key);
        this.hoveredConnections.delete(key);
        this.requestRender();
    },

    setConnectionHover(key, isHovered) {
        if (!key) return;
        if (isHovered) {
            this.hoveredConnections.add(key);
        } else {
            this.hoveredConnections.delete(key);
        }
        this.requestRender();
    },

    setTriggerLinks(links) {
        this.triggerLinks = links || [];
        this.requestRender();
    },

    setPreview(segments) {
        this.previewSegments = segments || [];
        this.requestRender();
    },

    clearPreview() {
        if (this.previewSegments.length === 0) return;
        this.previewSegments = [];
        this.requestRender();
    },

    clear() {
        this.connections.clear();
        this.hoveredConnections.clear();
        this.triggerLinks = [];
        this.previewSegments = [];
        this.requestRender();
    },

    requestRender() {
        if (!this.enabled || this.needsRender) return;
        this.needsRender = true;
        requestAnimationFrame(() => {
            this.needsRender = false;
            this.render();
        });
    },

    render() {
        if (!this.enabled || !this.gl || !this.canvas) return;

        this.resize();

        const positions = [];
        const colors = [];
        const pushRect = (x, y, width, height, color) => {
            if (width <= 0 || height <= 0) return;
            const x2 = x + width;
            const y2 = y + height;
            this.pushTriangle(positions, colors, color, x, y, x2, y, x, y2);
            this.pushTriangle(positions, colors, color, x, y2, x2, y, x2, y2);
        };
        const pushSegment = (segment, color, thickness = 2) => {
            const isVertical = segment.width <= segment.height;
            const x = isVertical ? segment.x + (segment.width - thickness) / 2 : segment.x;
            const y = isVertical ? segment.y : segment.y + (segment.height - thickness) / 2;
            const width = isVertical ? thickness : segment.width;
            const height = isVertical ? segment.height : thickness;
            pushRect(x, y, width, height, color);
        };

        this.connections.forEach((segments, key) => {
            const isHovered = this.hoveredConnections.has(key);
            const color = isHovered ? [0.937, 0.267, 0.267, 1] : [0.231, 0.510, 0.965, 1];
            const thickness = isHovered ? 4 : 2;
            segments.forEach((segment) => pushSegment(segment, color, thickness));
        });

        this.triggerLinks.forEach((link) => {
            this.pushDashedLine(positions, colors, link.fromX, link.fromY, link.toX, link.toY, [0.545, 0.361, 0.965, 0.72]);
            this.pushArrow(positions, colors, link.toX, link.toY, link.angle, [0.545, 0.361, 0.965, 0.72]);
        });

        this.previewSegments.forEach((segment) => {
            pushSegment(segment, [0.063, 0.725, 0.506, 1], 3);
        });

        const gl = this.gl;
        gl.viewport(0, 0, this.canvas.width, this.canvas.height);
        gl.clearColor(0, 0, 0, 0);
        gl.clear(gl.COLOR_BUFFER_BIT);

        if (positions.length === 0) return;

        gl.useProgram(this.program);
        gl.uniform2f(this.resolutionLocation, this.lastWidth, this.lastHeight);

        gl.bindBuffer(gl.ARRAY_BUFFER, this.positionBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(positions), gl.DYNAMIC_DRAW);
        gl.enableVertexAttribArray(this.positionLocation);
        gl.vertexAttribPointer(this.positionLocation, 2, gl.FLOAT, false, 0, 0);

        gl.bindBuffer(gl.ARRAY_BUFFER, this.colorBuffer);
        gl.bufferData(gl.ARRAY_BUFFER, new Float32Array(colors), gl.DYNAMIC_DRAW);
        gl.enableVertexAttribArray(this.colorLocation);
        gl.vertexAttribPointer(this.colorLocation, 4, gl.FLOAT, false, 0, 0);

        gl.drawArrays(gl.TRIANGLES, 0, positions.length / 2);
    },

    pushTriangle(positions, colors, color, x1, y1, x2, y2, x3, y3) {
        positions.push(x1, y1, x2, y2, x3, y3);
        colors.push(...color, ...color, ...color);
    },

    pushDashedLine(positions, colors, fromX, fromY, toX, toY, color) {
        const dx = toX - fromX;
        const dy = toY - fromY;
        const length = Math.sqrt(dx * dx + dy * dy);
        if (length <= 1) return;

        const nx = dx / length;
        const ny = dy / length;
        const px = -ny;
        const py = nx;
        const thickness = 2;
        const dash = 10;
        const gap = 7;

        for (let start = 0; start < length; start += dash + gap) {
            const end = Math.min(start + dash, length);
            const x1 = fromX + nx * start;
            const y1 = fromY + ny * start;
            const x2 = fromX + nx * end;
            const y2 = fromY + ny * end;
            const ax = px * thickness / 2;
            const ay = py * thickness / 2;
            this.pushTriangle(positions, colors, color, x1 - ax, y1 - ay, x2 - ax, y2 - ay, x1 + ax, y1 + ay);
            this.pushTriangle(positions, colors, color, x1 + ax, y1 + ay, x2 - ax, y2 - ay, x2 + ax, y2 + ay);
        }
    },

    pushArrow(positions, colors, tipX, tipY, angle, color) {
        const size = 9;
        const backX = tipX - Math.cos(angle) * size;
        const backY = tipY - Math.sin(angle) * size;
        const sideX = -Math.sin(angle) * size * 0.55;
        const sideY = Math.cos(angle) * size * 0.55;
        this.pushTriangle(
            positions,
            colors,
            color,
            tipX,
            tipY,
            backX + sideX,
            backY + sideY,
            backX - sideX,
            backY - sideY
        );
    }
};
