import re

# Update index.html
with open('web/index.html', 'r') as f:
    html = f.read()

new_main = """        <main class="dashboard-grid">
            
            <!-- Left Sidebar -->
            <aside class="sidebar sidebar-left">
                <!-- Target Selection -->
                <section class="card glass-panel">
                    <h2>Target Selection</h2>
                    <p class="subtitle">Select the object for the AI to track and follow.</p>
                    <div class="target-grid" id="target-selection">
                        <button class="target-btn active" data-target="person"><span class="icon">👤</span> Person</button>
                        <button class="target-btn" data-target="car"><span class="icon">🚗</span> Car</button>
                        <button class="target-btn" data-target="red_sphere"><span class="icon">🔴</span> Red Sphere</button>
                        <button class="target-btn" data-target="blue_cube"><span class="icon">🟦</span> Blue Cube</button>
                        <button class="target-btn" data-target="green_cone"><span class="icon">🟢</span> Green Cone</button>
                        <button class="target-btn" data-target="yellow_cylinder"><span class="icon">🟡</span> Yellow Cylinder</button>
                    </div>
                </section>

                <!-- Target Behavior -->
                <section class="card glass-panel">
                    <h2>Target Behavior</h2>
                    <p class="subtitle">Set the movement pattern for the target.</p>
                    <div class="behavior-controls">
                        <div class="control-row">
                            <button id="btn-behavior-toggle" class="btn btn-secondary" style="flex: 1; margin-bottom: 5px;">Hold Position (No Physics)</button>
                        </div>
                    </div>
                </section>

                <!-- Flight Controls -->
                <section class="card glass-panel flight-controls">
                    <h2>Flight Controls</h2>
                    <div class="control-row">
                        <button class="btn btn-primary" id="btn-arm">ARM</button>
                        <button class="btn btn-danger" id="btn-disarm">DISARM</button>
                    </div>
                    <div class="control-row" style="margin-top: 0.5rem;">
                        <button class="btn btn-secondary" id="btn-takeoff">TAKEOFF (5m)</button>
                        <button class="btn btn-secondary" id="btn-land">LAND</button>
                    </div>
                    <div class="control-action">
                        <button class="btn btn-action start" id="btn-follow">▶ FOLLOW TARGET</button>
                    </div>
                </section>
                
                <!-- Manual Flight Controls -->
                <section class="card glass-panel manual-controls">
                    <h2>Manual Controls</h2>
                    <div class="manual-layout">
                        <!-- Left: Translation Pad -->
                        <div class="control-pad-container">
                            <span class="pad-label">Directional</span>
                            <div class="dpad">
                                <div class="dpad-cell"></div>
                                <button class="btn btn-manual" id="btn-forward" data-direction="forward" title="Move Forward (W / ArrowUp)">▲</button>
                                <div class="dpad-cell"></div>
                                <button class="btn btn-manual" id="btn-left" data-direction="left" title="Move Left (A / ArrowLeft)">◀</button>
                                <button class="btn btn-manual btn-center" disabled>⌖</button>
                                <button class="btn btn-manual" id="btn-right" data-direction="right" title="Move Right (D / ArrowRight)">▶</button>
                                <div class="dpad-cell"></div>
                                <button class="btn btn-manual" id="btn-backward" data-direction="backward" title="Move Backward (S / ArrowDown)">▼</button>
                                <div class="dpad-cell"></div>
                            </div>
                        </div>
                        
                        <!-- Right: Altitude Pad and RTH -->
                        <div class="vertical-controls-container">
                            <span class="pad-label">Alt / Action</span>
                            <div class="vertical-buttons">
                                <button class="btn btn-manual btn-alt" id="btn-up" data-direction="up" title="Ascend (Space)"><span class="icon">⏶</span> UP</button>
                                <button class="btn btn-manual btn-alt" id="btn-down" data-direction="down" title="Descend (Shift)"><span class="icon">⏷</span> DOWN</button>
                                <button class="btn btn-rth" id="btn-rth" title="Return to Launch"><span class="icon">🏠</span> RTH</button>
                            </div>
                        </div>
                    </div>
                </section>
            </aside>

            <!-- Center Column: Video Feed -->
            <section class="video-section card glass-panel">
                <div class="card-header">
                    <h2>Live Video Feed</h2>
                    <span class="badge" id="mode-badge">UNKNOWN</span>
                </div>
                <div class="video-container">
                    <img id="video-stream" src="/video_feed" alt="Live Drone Camera Feed" />
                    <div class="crosshair"></div>
                </div>
            </section>

            <!-- Right Sidebar -->
            <aside class="sidebar sidebar-right">
                <!-- Detection Info -->
                <section class="card glass-panel detection-info">
                    <h2>Detection Status</h2>
                    <div class="info-row"><span class="label">Target:</span><span class="value highlight" id="det-class">--</span></div>
                    <div class="info-row"><span class="label">Confidence:</span><span class="value" id="det-conf">--</span></div>
                    <div class="info-row"><span class="label">Distance:</span><span class="value" id="det-dist">--</span></div>
                </section>

                <!-- Telemetry -->
                <section class="card glass-panel telemetry-panel">
                    <h2>Telemetry</h2>
                    <div class="stats-grid">
                        <div class="stat-box"><span class="label">Altitude</span><span class="value" id="tel-alt">0.0m</span></div>
                        <div class="stat-box"><span class="label">Speed</span><span class="value" id="tel-speed">0.0m/s</span></div>
                        <div class="stat-box"><span class="label">Battery</span><span class="value" id="tel-batt">100%</span></div>
                    </div>
                </section>

                <!-- SLAM Live Map -->
                <section class="card glass-panel map-panel">
                    <h2>Live 2D Map (SLAM)</h2>
                    <div style="width: 100%; height: 200px; display: flex; align-items: center; justify-content: center; background: #000; border-radius: 8px; overflow: hidden; position: relative;">
                        <img id="slam-map" src="/map_image" alt="SLAM Map" style="width: 100%; height: 100%; object-fit: contain;" />
                    </div>
                    <div class="info-row" style="margin-top: 10px;">
                        <span class="label">Position (VO):</span>
                        <span class="value highlight" id="slam-pose">X: 0.0, Y: 0.0</span>
                    </div>
                </section>

                <!-- Controller Tuning -->
                <section class="card glass-panel tuning-panel">
                    <h2>Controller Tuning</h2>
                    <p class="subtitle">Fine-tune PID & smoothing</p>
                    <div class="tuning-controls">
                        <div class="tuning-group">
                            <h3>Proportional Gains (P)</h3>
                            <div class="control-row-tuning">
                                <label for="range-kp-yaw">Yaw (P): <span id="val-kp-yaw">0.12</span></label>
                                <input type="range" id="range-kp-yaw" min="0.01" max="0.5" step="0.01" value="0.12" class="slider" />
                            </div>
                            <div class="control-row-tuning">
                                <label for="range-kp-fwd">Forward (P): <span id="val-kp-fwd">0.25</span></label>
                                <input type="range" id="range-kp-fwd" min="0.01" max="0.8" step="0.01" value="0.25" class="slider" />
                            </div>
                            <div class="control-row-tuning">
                                <label for="range-kp-alt">Altitude (P): <span id="val-kp-alt">0.25</span></label>
                                <input type="range" id="range-kp-alt" min="0.01" max="0.6" step="0.01" value="0.25" class="slider" />
                            </div>
                        </div>
                        <div class="tuning-group" style="margin-top: 10px;">
                            <h3>Smoothing Factors (&alpha;)</h3>
                            <div class="control-row-tuning">
                                <label for="range-alpha-yaw">Yaw Smooth: <span id="val-alpha-yaw">0.15</span></label>
                                <input type="range" id="range-alpha-yaw" min="0.01" max="1.0" step="0.01" value="0.15" class="slider" />
                            </div>
                            <div class="control-row-tuning">
                                <label for="range-alpha-fwd">Forward Smooth: <span id="val-alpha-fwd">0.10</span></label>
                                <input type="range" id="range-alpha-fwd" min="0.01" max="1.0" step="0.01" value="0.10" class="slider" />
                            </div>
                            <div class="control-row-tuning">
                                <label for="range-alpha-alt">Altitude Smooth: <span id="val-alpha-alt">0.10</span></label>
                                <input type="range" id="range-alpha-alt" min="0.01" max="1.0" step="0.01" value="0.10" class="slider" />
                            </div>
                            <div class="control-row-tuning" style="margin-top: 5px;">
                                <label for="range-max-fwd-speed">Max Fwd Speed: <span id="val-max-fwd-speed">1.0</span> m/s</label>
                                <input type="range" id="range-max-fwd-speed" min="0.1" max="3.0" step="0.1" value="1.0" class="slider" />
                            </div>
                        </div>
                        <button id="btn-tuning-save" class="btn btn-primary" style="margin-top:10px; width:100%">UPDATE TUNING</button>
                    </div>
                </section>
            </aside>
        </main>"""

pattern = re.compile(r'<main class="dashboard-grid">.*?</main>', re.DOTALL)
html = pattern.sub(new_main, html)

with open('web/index.html', 'w') as f:
    f.write(html)

# Update style.css
with open('web/style.css', 'r') as f:
    css = f.read()

css = re.sub(r'body \{[^}]*\}', '''body {
    font-family: 'Inter', sans-serif;
    background: var(--bg-dark);
    /* Subtle gradient background */
    background: radial-gradient(circle at top right, #1e1b4b 0%, var(--bg-dark) 100%);
    color: var(--text-main);
    min-height: 100vh;
    height: 100vh;
    overflow: hidden;
}''', css, count=1)

css = re.sub(r'\.app-container \{[^}]*\}', '''.app-container {
    max-width: 100%;
    height: 100vh;
    margin: 0 auto;
    padding: 0.75rem 1rem;
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
}''', css, count=1)

css = re.sub(r'\.dashboard-grid \{[^}]*\}', '''.dashboard-grid {
    display: grid;
    grid-template-columns: 320px 1fr 320px;
    gap: 1rem;
    flex: 1;
    min-height: 0;
}''', css, count=1)

css = re.sub(r'\.card \{[^}]*\}', '''.card {
    padding: 1rem;
}''', css, count=1)

css = re.sub(r'\.video-section \{[^}]*\}', '''.video-section {
    display: flex;
    flex-direction: column;
    padding: 0;
    overflow: hidden;
    height: 100%;
}''', css, count=1)

css = re.sub(r'\.video-container \{[^}]*\}', '''.video-container {
    position: relative;
    width: 100%;
    flex: 1;
    background: #000;
    display: flex;
    align-items: center;
    justify-content: center;
    overflow: hidden;
}''', css, count=1)

css = re.sub(r'\.video-container img \{[^}]*\}', '''.video-container img {
    width: 100%;
    height: 100%;
    object-fit: contain;
}''', css, count=1)

css = re.sub(r'\.sidebar \{[^}]*\}', '''.sidebar {
    display: flex;
    flex-direction: column;
    gap: 0.75rem;
    overflow-y: auto;
    padding-right: 0.25rem;
}
.sidebar::-webkit-scrollbar { width: 4px; }
.sidebar::-webkit-scrollbar-thumb { background: rgba(255,255,255,0.2); border-radius: 4px; }''', css, count=1)

css = re.sub(r'h2 \{[^}]*\}', '''h2 { font-size: 1.05rem; font-weight: 600; margin-bottom: 0.5rem; color: #cbd5e1; }''', css, count=1)
css = re.sub(r'\.subtitle \{[^}]*\}', '''.subtitle {
    font-size: 0.8rem;
    margin-bottom: 0.75rem;
}''', css, count=1)

css = re.sub(r'\.card-header \{[^}]*\}', '''.card-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 0.75rem 1rem;
    border-bottom: 1px solid var(--border-color);
}''', css, count=1)

css = re.sub(r'\.btn \{[^}]*\}', '''.btn {
    padding: 0.6rem;
    border: none;
    border-radius: 8px;
    font-weight: 600;
    font-size: 0.85rem;
    font-family: inherit;
    cursor: pointer;
    transition: all 0.2s;
    letter-spacing: 0.5px;
}''', css, count=1)

css = re.sub(r'\.btn-action \{[^}]*\}', '''.btn-action {
    width: 100%;
    padding: 0.75rem;
    font-size: 1rem;
    border-radius: 12px;
}''', css, count=1)

css = re.sub(r'\.glass-header \{[^}]*\}', '''.glass-header {
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 0.75rem 1.5rem;
    background: var(--bg-card);
    backdrop-filter: blur(var(--glass-blur));
    border: 1px solid var(--border-color);
    border-radius: 16px;
}''', css, count=1)

with open('web/style.css', 'w') as f:
    f.write(css)

print("Done updating index.html and style.css")
