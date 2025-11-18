/**
 * Configuration constants for the Assembly Line Control system
 */
const Config = {
    // Motor configuration
    MOTOR_STEPS_PER_SECOND: 100, // 100 steps = 1 second duration
    
    // ROS Bridge configuration
    ROS_BRIDGE_URL: 'ws://localhost:9090',
    ROS_RECONNECT_DELAY: 3000, // milliseconds
    
    // Block configuration
    BLOCK_MIN_WIDTH: 120,
    BLOCK_MAX_WIDTH: 300,
    BLOCK_DEFAULT_WIDTH: 140,
    
    // Validation
    MAX_STEPS: 100000,
    MIN_STEPS: -100000,
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
};

