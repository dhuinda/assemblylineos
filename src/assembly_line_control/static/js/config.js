/**
 * Configuration constants for the Assembly Line Control system
 */
const Config = {
    // Motor configuration
    MOTOR_STEPS_PER_SECOND: 100, // Default: 100 steps = 1 second duration
    MIN_MOTOR_SPEED: 1, // Minimum motor speed in steps per second
    MAX_MOTOR_SPEED: 6500, // Maximum motor speed in steps per second (Arduino limit)

    // ROS Bridge configuration
    // Use the URL provided by the server, or fall back to default
    ROS_BRIDGE_URL: (typeof window !== 'undefined' && window.ROS_BRIDGE_URL)
        ? window.ROS_BRIDGE_URL
        : 'ws://localhost:9090',
    ROS_RECONNECT_DELAY: 3000, // milliseconds

    // Block configuration
    BLOCK_MIN_WIDTH: 120,
    BLOCK_MAX_WIDTH: 300,
    BLOCK_DEFAULT_WIDTH: 140,

    // Validation
    MAX_STEPS: 2147483647, // Maximum 32-bit signed integer (Arduino long limit)
    MIN_STEPS: -2147483647, // Minimum 32-bit signed integer (Arduino long limit)
    MAX_TIME_DELAY: 3600, // 1 hour max
    MIN_TIME_DELAY: 0,

    // Storage
    STORAGE_KEY: 'assemblyLineConfig',
    WORKSPACES_KEY: 'assemblyLineWorkspaces',
    CURRENT_WORKSPACE_KEY: 'assemblyLineCurrentWorkspace',
    VERSION: '3.0.0',

    // Auto-save
    AUTO_SAVE_ENABLED: true,

    // UI
    ANIMATION_DURATION: 150, // milliseconds

    // Pixi workspace renderer (default). Legacy DOM renderer: Settings → Use legacy DOM renderer.
    // URL overrides: ?pixi=0 or ?legacy=1 disables Pixi; ?pixi=1 forces Pixi.
    PIXI_WORKSPACE: (function() {
        if (typeof window === 'undefined') return true;
        const params = new URLSearchParams(window.location.search || '');
        if (params.get('pixi') === '0' || params.get('legacy') === '1') return false;
        if (params.get('pixi') === '1') return true;
        if (localStorage.getItem('assemblyLineLegacyRenderer') === '1') return false;
        // Migrate old opt-in flag (assemblyLinePixiWorkspace=0 meant user disabled pixi)
        if (localStorage.getItem('assemblyLineLegacyRenderer') == null &&
            localStorage.getItem('assemblyLinePixiWorkspace') === '0') {
            return false;
        }
        return true;
    })(),

    /** True when Pixi was requested and the GPU renderer initialized successfully. */
    isPixiWorkspaceActive() {
        return this.PIXI_WORKSPACE &&
            typeof PixiWorkspaceRenderer !== 'undefined' &&
            PixiWorkspaceRenderer.enabled;
    },
};
