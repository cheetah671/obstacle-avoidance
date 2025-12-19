import {
    db, auth,
    ref, onValue, set, push, serverTimestamp,
    signInAnonymously, onAuthStateChanged,
    isDemo
  } from './firebase.js';

  // ===== UI Elements =====
  const modeToggle = document.getElementById('modeToggle');
  const modeLabel = document.getElementById('modeLabel');
  const modeMetric = document.getElementById('modeMetric');
  const connChip = document.getElementById('connChip');
  const distanceEl = document.getElementById('distance');
  const lastCmdEl = document.getElementById('lastCmd');
  const fbStatus = document.getElementById('fbStatus');
  const logsContent = document.getElementById('logsContent');
  const btnUp = document.getElementById('btnUp');
  const btnDown = document.getElementById('btnDown');
  const btnLeft = document.getElementById('btnLeft');
  const btnRight = document.getElementById('btnRight');
  const btnStop = document.getElementById('btnStop');

  // Theme Elements
  const settingsBtn = document.getElementById('settingsBtn');
  const themePanel = document.getElementById('themePanel');
  const themePanelOverlay = document.getElementById('themePanelOverlay');
  const closeThemePanelBtn = document.getElementById('closeThemePanelBtn');
  const themeSwitcherBtns = document.querySelectorAll('.theme-switcher__btn');
  const colorSwatches = document.querySelectorAll('.color-swatch');

  // ===== Robot Status Tracking =====
  let robotConnected = false;
  let lastCommandSent = '';
  const MAX_LOG_ENTRIES = 50;

  // ===== Logging Function =====
  function addLogEntry(message, type = 'info') {
    if (!logsContent) return;
    const now = new Date();
    const time = `${now.getHours().toString().padStart(2, '0')}:${now.getMinutes().toString().padStart(2, '0')}:${now.getSeconds().toString().padStart(2, '0')}`;
    const p = document.createElement('p');
    p.className = `log-entry log-${type}`;
    p.innerHTML = `<span class="log-time">[${time}]</span> <span>${message}</span>`;
    logsContent.appendChild(p);
    
    while (logsContent.children.length > MAX_LOG_ENTRIES) {
      logsContent.removeChild(logsContent.firstChild);
    }
    
    logsContent.scrollTop = logsContent.scrollHeight;
  }

  // ===== Theme Management =====
  function applyTheme(theme, accentColor) {
    document.body.dataset.theme = theme;
    themeSwitcherBtns.forEach(btn => {
      btn.classList.toggle('is-active', btn.dataset.themeSet === theme);
    });
    
    const root = document.documentElement;
    root.style.setProperty('--accent-primary', accentColor);
    const accentGlow = hexToRgbA(accentColor, 0.4);
    root.style.setProperty('--accent-glow', accentGlow);
    
    // Extract RGB values for ripple effect
    const rgb = hexToRgb(accentColor);
    root.style.setProperty('--accent-primary-rgb', `${rgb.r}, ${rgb.g}, ${rgb.b}`);
    
    colorSwatches.forEach(swatch => {
      swatch.classList.toggle('is-active', swatch.dataset.color === accentColor);
    });
  }

  function saveThemePreferences(theme, accentColor) {
    localStorage.setItem('robotDashboardTheme', JSON.stringify({ theme, accentColor }));
  }

  function loadThemePreferences() {
    const prefs = JSON.parse(localStorage.getItem('robotDashboardTheme'));
    const defaultPrefs = { theme: 'dark', accentColor: '#0ea5e9' };
    applyTheme(prefs?.theme || defaultPrefs.theme, prefs?.accentColor || defaultPrefs.accentColor);
  }

  function setupTheming() {
    settingsBtn.addEventListener('click', () => {
      themePanel.classList.add('is-open');
      themePanelOverlay.classList.add('is-open');
    });
    
    const closePanel = () => {
      themePanel.classList.remove('is-open');
      themePanelOverlay.classList.remove('is-open');
    };
    
    closeThemePanelBtn.addEventListener('click', closePanel);
    themePanelOverlay.addEventListener('click', closePanel);
    
    themeSwitcherBtns.forEach(btn => {
      btn.addEventListener('click', () => {
        const theme = btn.dataset.themeSet;
        const currentAccent = document.documentElement.style.getPropertyValue('--accent-primary').trim();
        applyTheme(theme, currentAccent);
        saveThemePreferences(theme, currentAccent);
      });
    });
    
    colorSwatches.forEach(swatch => {
      swatch.addEventListener('click', () => {
        const accentColor = swatch.dataset.color;
        const currentTheme = document.body.dataset.theme;
        applyTheme(currentTheme, accentColor);
        saveThemePreferences(currentTheme, accentColor);
      });
    });
  }

  function hexToRgbA(hex, alpha) {
    let c;
    if (/^#([A-Fa-f0-9]{3}){1,2}$/.test(hex)) {
      c = hex.substring(1).split('');
      if (c.length === 3) c = [c[0], c[0], c[1], c[1], c[2], c[2]];
      c = '0x' + c.join('');
      return `rgba(${[(c>>16)&255, (c>>8)&255, c&255].join(',')},${alpha})`;
    }
    return `rgba(14, 165, 233, ${alpha})`;
  }

  function hexToRgb(hex) {
    const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
    return result ? {
      r: parseInt(result[1], 16),
      g: parseInt(result[2], 16),
      b: parseInt(result[3], 16)
    } : { r: 14, g: 165, b: 233 };
  }

  // ===== Robot Control Functions =====
  function setModeUI(mode) {
    const isAuto = mode === 'auto';
    modeToggle.dataset.on = String(isAuto);
    modeToggle.setAttribute('aria-checked', String(isAuto));
    modeLabel.textContent = isAuto ? 'Auto' : 'Manual';
    modeMetric.textContent = isAuto ? 'Auto' : 'Manual';
    [btnUp, btnDown, btnLeft, btnRight, btnStop].forEach(b => b.disabled = isAuto);
  }

  async function setMode(mode) {
    if (isDemo) {
      addLogEntry(`Mode changed to: ${mode.toUpperCase()}`, 'warn');
      setModeUI(mode);
      return;
    }
    try {
      await set(ref(db, 'control/mode'), mode);
      await push(ref(db, 'events/mode_changes'), { mode, ts: serverTimestamp() });
      addLogEntry(`Mode changed to: ${mode.toUpperCase()}`, 'warn');
    } catch (error) {
      console.error('Mode change failed:', error);
      addLogEntry('Error: Mode change failed.', 'bad');
    }
  }

  async function sendCommand(command) {
    window.dispatchEvent(new CustomEvent('command', { detail: command }));
    lastCmdEl.textContent = command.toUpperCase();
    if (command !== 'stop') {
      addLogEntry(`Sending command: ${command.toUpperCase()}`, 'cmd');
    }
    if (!robotConnected && !isDemo) {
        addLogEntry(`Robot offline. Command "${command}" not sent.`, 'bad');
    }
    if (isDemo) return;
    try {
      await set(ref(db, 'control/command'), command);
      await push(ref(db, 'control/commands'), { command, ts: serverTimestamp(), source: 'website' });
      lastCommandSent = command;
    } catch (error) {
      console.error('Command send failed:', error);
      lastCmdEl.textContent = 'ERROR';
      addLogEntry(`Error: Command "${command}" failed to send.`, 'bad');
    }
  }

  // ===== Firebase Connection & Real-time Data =====
  async function initializeApp() {
    loadThemePreferences();
    setupTheming();
    setupControls();
    
    addLogEntry('System Initialized. Connecting...');
    fbStatus.textContent = 'Firebase: connecting...';
    
    if (isDemo) {
        handleDemoMode();
        return;
    }

    try {
      await signInAnonymously(auth);
    } catch (e) {
      console.error('Firebase auth error:', e);
      addLogEntry('Firebase authentication failed.', 'bad');
      fbStatus.textContent = 'Firebase: Auth Failed';
      return;
    }
    
    onAuthStateChanged(auth, (user) => {
      if (user) {
        fbStatus.textContent = 'Firebase: connected';
        addLogEntry('Firebase connection successful.', 'ok');
        bindRealtime();
        initializeRobotConnection();
      } else {
        fbStatus.textContent = 'Firebase: disconnected';
        addLogEntry('Firebase disconnected.', 'bad');
        robotConnected = false;
      }
    });
  }

  function handleDemoMode() {
    addLogEntry('System running in Demo Mode.', 'warn');
    fbStatus.textContent = 'Firebase: Demo Mode';
    robotConnected = true;
    connChip.textContent = 'Connected';
    connChip.className = 'chip chip--ok';
    setInterval(() => {
        const distance = Math.floor(Math.random() * 100) + 20;
        distanceEl.textContent = distance;
        const parentMetric = distanceEl.closest('.metric__value');
        if (parentMetric) {
          parentMetric.classList.remove('value-change');
          void parentMetric.offsetWidth;
          parentMetric.classList.add('value-change');
        }
    }, 2000);
  }

  function bindRealtime() {
    onValue(ref(db, 'sensors/distance_cm'), (snapshot) => {
      const v = snapshot.val();
      const formattedV = (typeof v === 'number') ? v.toFixed(0) : '—';
      if (distanceEl.textContent !== formattedV) {
        distanceEl.textContent = formattedV;
        const parentMetric = distanceEl.closest('.metric__value');
        if (parentMetric) {
          parentMetric.classList.remove('value-change');
          void parentMetric.offsetWidth;
          parentMetric.classList.add('value-change');
        }
      }
    });
    
    let firstConnection = true;
    onValue(ref(db, 'status/connected'), (snapshot) => {
      const ok = !!snapshot.val();
      if (ok !== robotConnected || firstConnection) {
        robotConnected = ok;
        connChip.textContent = ok ? 'Connected' : 'Disconnected';
        connChip.className = `chip ${ok ? 'chip--ok' : 'chip--bad'}`;
        addLogEntry(`Robot is now ${ok ? 'Online' : 'Offline'}`, ok ? 'ok' : 'bad');
        firstConnection = false;
      }
    });
    
    onValue(ref(db, 'control/mode'), (snapshot) => {
      const mode = snapshot.val() === 'auto' ? 'auto' : 'manual';
      setModeUI(mode);
    });
  }

  async function initializeRobotConnection() {
    try {
      await set(ref(db, 'control/mode'), 'manual');
      await set(ref(db, 'status/website_connected'), true);
      addLogEntry('Dashboard connection established.');
    } catch (error) {
      console.error('Failed to initialize robot connection:', error);
    }
  }

  // ===== Controls Setup =====
  function setupControls() {
    modeToggle.addEventListener('click', () => setMode(modeToggle.dataset.on === 'true' ? 'manual' : 'auto'));
    
    btnUp.addEventListener('mousedown', () => sendCommand('forward'));
    btnDown.addEventListener('mousedown', () => sendCommand('backward'));
    btnLeft.addEventListener('mousedown', () => sendCommand('left'));
    btnRight.addEventListener('mousedown', () => sendCommand('right'));
    btnStop.addEventListener('click', () => sendCommand('stop'));

    const stopMovement = () => sendCommand('stop');
    ['mouseup', 'mouseleave'].forEach(evt => {
        btnUp.addEventListener(evt, stopMovement);
        btnDown.addEventListener(evt, stopMovement);
        btnLeft.addEventListener(evt, stopMovement);
        btnRight.addEventListener(evt, stopMovement);
    });
    
    const keyMap = { ArrowUp: 'forward', KeyW: 'forward', ArrowDown: 'backward', KeyS: 'backward', ArrowLeft: 'left', KeyA: 'left', ArrowRight: 'right', KeyD: 'right', Space: 'stop' };
    
    window.addEventListener('keydown', (e) => {
      const cmd = keyMap[e.code];
      if (cmd && !e.repeat && modeToggle.dataset.on !== 'true') {
        e.preventDefault();
        sendCommand(cmd);
      }
    });
    window.addEventListener('keyup', (e) => {
        const cmd = keyMap[e.code];
        if (cmd && cmd !== 'stop') {
            sendCommand('stop');
        }
    });
    
    addLogEntry('Manual controls are ready.');
  }

  // ===== Initialize Application =====
  initializeApp().catch(error => {
    console.error('App initialization failed:', error);
    addLogEntry('CRITICAL: App failed to start.', 'bad');
  });

  // ===== Connection Monitoring =====
  window.addEventListener('beforeunload', async () => {
    if (isDemo) return;
    try {
      await set(ref(db, 'status/website_connected'), false);
    } catch (error) {
      console.error('Failed to update disconnect status:', error);
    }
  });

  // ===== Custom Color Picker =====
  const colorSpectrum = document.getElementById('colorSpectrum');
  const spectrumCursor = document.getElementById('spectrumCursor');
  const brightnessSlider = document.getElementById('brightnessSlider');
  const rgbR = document.getElementById('rgbR');
  const rgbG = document.getElementById('rgbG');
  const rgbB = document.getElementById('rgbB');
  const hexValue = document.getElementById('hexValue');
  const applyCustomColorBtn = document.getElementById('applyCustomColor');
  const recentColorsContainer = document.getElementById('recentColors');
  const previewSwatch = document.getElementById('previewSwatch');

  let currentHue = 180;
  let currentSaturation = 100;
  let currentBrightness = 50;
  let isPickingColor = false;

  // Draw color spectrum gradient
  function drawColorSpectrum() {
    if (!colorSpectrum) return;
    const ctx = colorSpectrum.getContext('2d');
    const width = colorSpectrum.width;
    const height = colorSpectrum.height;

    // Create horizontal hue gradient
    for (let x = 0; x < width; x++) {
      const hue = (x / width) * 360;
      const gradient = ctx.createLinearGradient(0, 0, 0, height);
      gradient.addColorStop(0, `hsl(${hue}, 100%, 50%)`);
      gradient.addColorStop(0.5, `hsl(${hue}, 100%, 50%)`);
      gradient.addColorStop(1, `hsl(${hue}, 50%, 25%)`);
      
      ctx.fillStyle = gradient;
      ctx.fillRect(x, 0, 1, height);
    }

    // Add saturation overlay
    const saturationGradient = ctx.createLinearGradient(0, 0, 0, height);
    saturationGradient.addColorStop(0, 'rgba(255, 255, 255, 0)');
    saturationGradient.addColorStop(1, 'rgba(128, 128, 128, 0.5)');
    ctx.fillStyle = saturationGradient;
    ctx.fillRect(0, 0, width, height);
  }

  // Convert HSL to RGB
  function hslToRgb(h, s, l) {
    s /= 100;
    l /= 100;
    const k = n => (n + h / 30) % 12;
    const a = s * Math.min(l, 1 - l);
    const f = n => l - a * Math.max(-1, Math.min(k(n) - 3, Math.min(9 - k(n), 1)));
    return [
      Math.round(255 * f(0)),
      Math.round(255 * f(8)),
      Math.round(255 * f(4))
    ];
  }

  // Convert RGB to Hex
  function rgbToHex(r, g, b) {
    return '#' + [r, g, b].map(x => {
      const hex = x.toString(16);
      return hex.length === 1 ? '0' + hex : hex;
    }).join('');
  }

  // Update RGB values from HSL
  function updateRgbFromHsl() {
    const [r, g, b] = hslToRgb(currentHue, currentSaturation, currentBrightness);
    rgbR.value = r;
    rgbG.value = g;
    rgbB.value = b;
    const hex = rgbToHex(r, g, b);
    hexValue.textContent = hex;
    updateBrightnessSliderGradient();
    updateColorPreview(hex);
  }

  // Update HSL from RGB
  function updateHslFromRgb() {
    const r = parseInt(rgbR.value) / 255;
    const g = parseInt(rgbG.value) / 255;
    const b = parseInt(rgbB.value) / 255;

    const max = Math.max(r, g, b);
    const min = Math.min(r, g, b);
    let h, s, l = (max + min) / 2;

    if (max === min) {
      h = s = 0;
    } else {
      const d = max - min;
      s = l > 0.5 ? d / (2 - max - min) : d / (max + min);
      switch (max) {
        case r: h = ((g - b) / d + (g < b ? 6 : 0)) / 6; break;
        case g: h = ((b - r) / d + 2) / 6; break;
        case b: h = ((r - g) / d + 4) / 6; break;
      }
    }

    currentHue = Math.round(h * 360);
    currentSaturation = Math.round(s * 100);
    currentBrightness = Math.round(l * 100);
    brightnessSlider.value = currentBrightness;
    
    const hex = rgbToHex(
      parseInt(rgbR.value),
      parseInt(rgbG.value),
      parseInt(rgbB.value)
    );
    hexValue.textContent = hex;
    updateCursorPosition();
    updateBrightnessSliderGradient();
    updateColorPreview(hex);
  }

  // Update cursor position on spectrum
  function updateCursorPosition() {
    const x = (currentHue / 360) * colorSpectrum.width;
    const y = (1 - currentSaturation / 100) * colorSpectrum.height;
    spectrumCursor.style.left = `${x}px`;
    spectrumCursor.style.top = `${y}px`;
  }

  // Update brightness slider gradient
  function updateBrightnessSliderGradient() {
    const [r, g, b] = hslToRgb(currentHue, currentSaturation, 50);
    const midColor = rgbToHex(r, g, b);
    brightnessSlider.style.background = `linear-gradient(to right, #000 0%, ${midColor} 50%, #fff 100%)`;
  }

  // Handle spectrum click
  function handleSpectrumInteraction(e) {
    const rect = colorSpectrum.getBoundingClientRect();
    const x = Math.max(0, Math.min(e.clientX - rect.left, rect.width));
    const y = Math.max(0, Math.min(e.clientY - rect.top, rect.height));
    
    currentHue = Math.round((x / rect.width) * 360);
    currentSaturation = Math.round((1 - y / rect.height) * 100);
    
    updateRgbFromHsl();
    updateCursorPosition();
  }

  // Setup color picker event listeners
  if (colorSpectrum) {
    drawColorSpectrum();
    updateRgbFromHsl();
    updateCursorPosition();

    colorSpectrum.addEventListener('mousedown', (e) => {
      isPickingColor = true;
      handleSpectrumInteraction(e);
    });

    document.addEventListener('mousemove', (e) => {
      if (isPickingColor) {
        handleSpectrumInteraction(e);
      }
    });

    document.addEventListener('mouseup', () => {
      isPickingColor = false;
    });

    brightnessSlider.addEventListener('input', (e) => {
      currentBrightness = parseInt(e.target.value);
      updateRgbFromHsl();
    });

    // RGB input listeners
    [rgbR, rgbG, rgbB].forEach(input => {
      input.addEventListener('input', () => {
        // Clamp values
        const val = parseInt(input.value);
        if (val < 0) input.value = 0;
        if (val > 255) input.value = 255;
        updateHslFromRgb();
      });
    });

    // Apply custom color button
    applyCustomColorBtn.addEventListener('click', () => {
      const hex = hexValue.textContent;
      const currentTheme = document.body.dataset.theme;
      applyTheme(currentTheme, hex);
      saveThemePreferences(currentTheme, hex);
      addRecentColor(hex);
      addLogEntry(`Custom color applied: ${hex}`, 'ok');
    });
  }

  // Recent colors management
  function loadRecentColors() {
    const colors = JSON.parse(localStorage.getItem('recentColors') || '[]');
    return colors.slice(0, 8); // Keep max 8 recent colors
  }

  function saveRecentColors(colors) {
    localStorage.setItem('recentColors', JSON.stringify(colors));
  }

  function addRecentColor(hex) {
    let colors = loadRecentColors();
    // Remove if already exists
    colors = colors.filter(c => c !== hex);
    // Add to front
    colors.unshift(hex);
    // Keep only 8
    colors = colors.slice(0, 8);
    saveRecentColors(colors);
    renderRecentColors();
  }

  function renderRecentColors() {
    const colors = loadRecentColors();
    
    if (colors.length === 0) {
      recentColorsContainer.innerHTML = '<div class="recent-colors-empty">No recent colors</div>';
      return;
    }

    recentColorsContainer.innerHTML = '';
    colors.forEach(color => {
      const swatch = document.createElement('button');
      swatch.className = 'recent-color-swatch';
      swatch.style.backgroundColor = color;
      swatch.title = color;
      swatch.addEventListener('click', () => {
        const currentTheme = document.body.dataset.theme;
        applyTheme(currentTheme, color);
        saveThemePreferences(currentTheme, color);
        addLogEntry(`Recent color applied: ${color}`, 'ok');
      });
      recentColorsContainer.appendChild(swatch);
    });
  }

  // Update color preview
  function updateColorPreview(hex) {
    if (previewSwatch) {
      previewSwatch.style.backgroundColor = hex;
      previewSwatch.style.boxShadow = `0 0 20px ${hex}80, inset 0 0 10px rgba(255, 255, 255, 0.1)`;
    }
  }

  // Initialize recent colors
  renderRecentColors();

  // ===== Speed Slider Control =====
  const speedSlider = document.getElementById('speedSlider');
  const speedIcon = document.getElementById('speedIcon');
  // Display values shown to user (percentage-like feel)
  const speedDisplayValues = [0, 50, 100, 150, 200, 255];
  // Actual PWM values sent to hardware (min 180 for motor to move, 0 = stop)
  const MIN_PWM = 180;
  const MAX_PWM = 255;
  let isDragging = false;

  // Map display value to actual PWM: 0 -> 0 (stop), non-zero -> 180-255 range
  function mapToActualPWM(displayValue) {
    if (displayValue === 0) return 0; // Stop
    // Map 50-255 display range to 180-255 PWM range
    const minDisplay = 50;
    const maxDisplay = 255;
    const pwm = MIN_PWM + ((displayValue - minDisplay) / (maxDisplay - minDisplay)) * (MAX_PWM - MIN_PWM);
    return Math.round(Math.max(MIN_PWM, Math.min(MAX_PWM, pwm)));
  }

  if (speedSlider && speedIcon) {
    function updateSpeedDisplay() {
      const index = parseInt(speedSlider.value);
      const displaySpeed = speedDisplayValues[index];
      const actualPWM = mapToActualPWM(displaySpeed);
      
      // Update the bubble value to show actual PWM
      speedIcon.setAttribute('data-value', actualPWM);
      
      // Calculate exact thumb position
      // Thumb is 24px wide, so center is at 12px offset
      const thumbWidth = 24;
      const sliderWidth = speedSlider.offsetWidth;
      const availableWidth = sliderWidth - thumbWidth;
      const thumbPosition = (index / 5) * availableWidth - (thumbWidth / 2) + 2.5;
      
      // Position the bubble exactly above the thumb center
      speedIcon.style.left = `${thumbPosition}px`;
      
      // Change color based on speed
      let colorRgb;
      if (actualPWM === 0) {
        colorRgb = '107, 114, 128'; // Gray for stop
      } else if (actualPWM <= 200) {
        colorRgb = '16, 185, 129'; // Green for slow
      } else if (actualPWM <= 230) {
        colorRgb = '245, 158, 11'; // Orange for medium
      } else {
        colorRgb = '225, 29, 72'; // Red for fast
      }
      
      speedIcon.style.background = `linear-gradient(135deg, rgba(${colorRgb}, 0.25) 0%, rgba(${colorRgb}, 0.15) 100%)`;
      speedIcon.style.boxShadow = `0 0 25px rgba(${colorRgb}, 0.6), 0 4px 12px rgba(0, 0, 0, 0.5)`;
      speedIcon.style.borderColor = `rgba(${colorRgb}, 0.4)`;
      speedIcon.style.color = `rgb(${colorRgb})`;
    }

    function showStar() {
      isDragging = true;
      speedIcon.classList.add('active');
      updateSpeedDisplay();
    }

    function hideStar() {
      isDragging = false;
      speedIcon.classList.remove('active');
    }

    // Show star when dragging
    speedSlider.addEventListener('mousedown', showStar);
    speedSlider.addEventListener('touchstart', showStar);
    
    // Update position while dragging
    speedSlider.addEventListener('input', () => {
      if (isDragging) {
        updateSpeedDisplay();
      }
    });
    
    // Hide star when released
    speedSlider.addEventListener('mouseup', hideStar);
    speedSlider.addEventListener('touchend', hideStar);
    document.addEventListener('mouseup', () => {
      if (isDragging) hideStar();
    });
    
    // Initial update (hidden)
    updateSpeedDisplay();
    
    // Recalculate on window resize
    window.addEventListener('resize', () => {
      if (isDragging) updateSpeedDisplay();
    });

    // Send speed to Firebase when changed
    speedSlider.addEventListener('change', async () => {
      const index = parseInt(speedSlider.value);
      const displaySpeed = speedDisplayValues[index];
      const actualPWM = mapToActualPWM(displaySpeed);
      
      if (isDemo) {
        addLogEntry(`Speed set to PWM: ${actualPWM}`, 'cmd');
        return;
      }

      try {
        await set(ref(db, 'control/speed'), actualPWM);
        addLogEntry(`Speed set to PWM: ${actualPWM}`, 'cmd');
      } catch (error) {
        console.error('Speed change failed:', error);
        addLogEntry('Error: Speed change failed.', 'bad');
      }
    });
  }
