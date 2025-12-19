// ===== Simulation Setup =====
const steeringWheel = document.getElementById('steeringWheel');
let currentRotation = 0;
let currentTilt = 0;

function updateSteeringWheel(command) {
    let targetRotation = 0;
    let targetTilt = 0;

    switch (command) {
        case 'left':
            targetRotation = -3000; // Increased rotation
            break;
        case 'right':
            targetRotation = 3000; // Increased rotation
            break;
        case 'forward':
            targetTilt = 2000;
            break;
        case 'backward':
            targetTilt = -2000;
            break;
        case 'stop':
            // Targets remain 0, so the wheel returns to center
            break;
    }

    // Smoothly animate towards the target rotation and tilt
    currentRotation += (targetRotation - currentRotation) * 0.2;
    currentTilt += (targetTilt - currentTilt) * 0.2;

    if (steeringWheel) {
        //
        // === THIS IS THE LINE THAT CONTROLS THE ROTATION ===
        // The `rotateZ(${currentRotation}deg)` part handles the left/right turning.
        //
        steeringWheel.style.transform = `rotateX(${currentTilt}deg) rotateZ(${currentRotation}deg)`;
    }
}

function gameLoop() {
    // This function is called on every frame to smoothly return the wheel to center
    updateSteeringWheel('stop');
    requestAnimationFrame(gameLoop);
}

document.addEventListener('DOMContentLoaded', () => {
    if (steeringWheel) {
        gameLoop();
        // Listen for the 'command' event from app.js
        window.addEventListener('command', (e) => {
            updateSteeringWheel(e.detail);
        });
    }
});