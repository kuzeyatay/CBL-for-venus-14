// Cell-based canvas.
// Integer cell coordinates: col increases right, row increases down.

const CELL_SIZE = 48;
const GRID_COLOR = '#d0d0dc';
const AXIS_COLOR = '#a0a0b8';

// Colors per cell status (mirrors cell_status_t in robot_types.h)
const STATUS_FILL = {
    unknown:       '#f0f0f8',
    scanned_clear: '#eef8f2',
    reserved:      '#e8eeff',
    explored:      '#ffffff',
    unsafe:        '#ffe8e8',
    tape:          '#e8e8e8',
    hill:          '#f5e8d0',
    sample:        '#f0e0f8',
};

// Map protocol color strings to CSS colors for rock samples
const SAMPLE_CSS_COLOR = {
    white:   '#cccccc',
    black:   '#333333',
    red:     '#e74c3c',
    green:   '#27ae60',
    blue:    '#2980b9',
    unknown: '#888888',
};

const CONTENT_ICONS = {
    rock_sample: { symbol: '◆', baseColor: '#555' },
    hill:        { symbol: '▲', baseColor: '#8b6914' },
    black_tape:  { symbol: '⬟', baseColor: '#222' },
    // legacy
    rock:        { symbol: '◆', baseColor: '#555' },
    cliff:       { symbol: '⬡', baseColor: '#7c4daa' },
};

let offsetX = 0;
let offsetY = 0;
let scale   = 3;

const canvas = document.getElementById('canvas');
const ctx    = canvas.getContext('2d');

// cellMap: key = "col,row" → { status: string, contents: [] }
const cellMap = new Map();

function cellKey(col, row) { return `${col},${row}`; }

function getCell(col, row) {
    const k = cellKey(col, row);
    if (!cellMap.has(k)) cellMap.set(k, { status: 'unknown', contents: [] });
    return cellMap.get(k);
}

// Called when a robot visits a cell; only advances unknown → scanned_clear
function markVisited(col, row) {
    const cell = getCell(col, row);
    if (cell.status === 'unknown') cell.status = 'scanned_clear';
}

// Set cell status explicitly (from CELL_UPDATE messages)
export function updateCellStatus(col, row, status) {
    getCell(Math.round(col), Math.round(row)).status = status;
    draw();
}

// Add a detected object to a cell (from EVENT messages)
export function addCellContent(col, row, item) {
    const cellCol = Math.round(col);
    const cellRow = Math.round(row);
    const cell    = getCell(cellCol, cellRow);

    if (cell.status === 'unknown') cell.status = 'explored';

    const isDupe = cell.contents.some(c =>
        c.type === item.type && c.color === item.color
    );
    if (!isDupe) cell.contents.push(item);
    draw();
}

// ---------------------------------------------------------------------------
// Robots
// ---------------------------------------------------------------------------
const robots = new Map(); // key: robotId → { col, row, heading, color }

export function updateRobot(robotId, col, row, headingRad) {
    const cellCol = Math.round(col);
    const cellRow = Math.round(row);

    if (!robots.has(robotId)) {
        const palette = ['#e74c3c', '#2980b9', '#27ae60', '#8e44ad'];
        robots.set(robotId, {
            col: cellCol, row: cellRow,
            heading: headingRad,
            color: palette[robots.size % palette.length],
        });
    } else {
        const r = robots.get(robotId);
        r.col = cellCol; r.row = cellRow; r.heading = headingRad;
    }
    markVisited(cellCol, cellRow);
    draw();
    updateSidebar();
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------
function cellToScreen(col, row) {
    return [col * CELL_SIZE * scale + offsetX, row * CELL_SIZE * scale + offsetY];
}

export function screenToCell(screenX, screenY) {
    return [
        Math.floor((screenX - offsetX) / (CELL_SIZE * scale)),
        Math.floor((screenY - offsetY) / (CELL_SIZE * scale)),
    ];
}

// ---------------------------------------------------------------------------
// Drawing
// ---------------------------------------------------------------------------
function drawCells() {
    const [x0, y0] = screenToCell(0, 0);
    const [x1, y1] = screenToCell(canvas.width, canvas.height);
    const pad = 2;
    const sw  = CELL_SIZE * scale;

    for (let col = x0 - pad; col <= x1 + pad; col++) {
        for (let row = y0 - pad; row <= y1 + pad; row++) {
            const [sx, sy] = cellToScreen(col, row);
            const cell     = cellMap.get(cellKey(col, row));
            const status   = cell?.status ?? 'unknown';

            ctx.fillStyle = STATUS_FILL[status] ?? STATUS_FILL.unknown;
            ctx.fillRect(sx, sy, sw, sw);

            ctx.strokeStyle = GRID_COLOR;
            ctx.lineWidth   = 0.5;
            ctx.strokeRect(sx + 0.25, sy + 0.25, sw - 0.5, sw - 0.5);

            const contents = cell?.contents ?? [];
            if (!contents.length) {
                if (sw > 56 && status !== 'unknown') {
                    ctx.save();
                    ctx.font          = `${Math.min(sw * 0.14, 9)}px monospace`;
                    ctx.textAlign     = 'left';
                    ctx.textBaseline  = 'top';
                    ctx.fillStyle     = '#b0b0c8';
                    ctx.fillText(status.replace(/_/g, ' '), sx + 3, sy + 3);
                    ctx.restore();
                }
            } else if (contents.length === 1) {
                const item = contents[0];
                const info = CONTENT_ICONS[item.type] ?? { symbol: '?', baseColor: '#aaa' };
                const cssColor = item.type === 'rock_sample'
                    ? (SAMPLE_CSS_COLOR[item.color] ?? info.baseColor)
                    : (item.color ?? info.baseColor);
                const fontSize = Math.max(8, Math.min(sw * 0.45, 28));
                ctx.save();
                ctx.font          = `${fontSize}px monospace`;
                ctx.textAlign     = 'center';
                ctx.textBaseline  = 'middle';
                ctx.fillStyle     = cssColor;
                ctx.fillText(info.symbol, sx + sw / 2, sy + sw / 2);
                ctx.restore();
            } else {
                const fontSize = Math.max(7, Math.min(sw * 0.28, 16));
                ctx.save();
                ctx.font          = `${fontSize}px monospace`;
                ctx.textAlign     = 'center';
                ctx.textBaseline  = 'middle';
                ctx.fillStyle     = '#888';
                ctx.fillText(`×${contents.length}`, sx + sw / 2, sy + sw / 2);
                ctx.restore();
            }

            if (sw > 40) {
                ctx.save();
                ctx.font          = `${Math.min(sw * 0.18, 11)}px monospace`;
                ctx.textAlign     = 'center';
                ctx.textBaseline  = 'middle';
                ctx.fillStyle     = '#c8c8d8';
                ctx.fillText(`${col},${row}`, sx + sw / 2, sy + sw / 2);
                ctx.restore();
            }
        }
    }
}

function drawRobots() {
    const sw = CELL_SIZE * scale;
    for (const [id, r] of robots) {
        const [sx, sy] = cellToScreen(r.col, r.row);
        const cx = sx + sw / 2;
        const cy = sy + sw / 2;
        const rw = Math.max(6, sw * 0.35);
        const rh = Math.max(8, sw * 0.5);

        ctx.save();
        ctx.translate(cx, cy);
        // yaw 0 = forward = up in canvas convention; heading stored as radians
        ctx.rotate(-r.heading);

        ctx.fillStyle   = r.color;
        ctx.strokeStyle = 'rgba(0,0,0,0.25)';
        ctx.lineWidth   = Math.max(0.5, sw * 0.02);
        ctx.beginPath();
        ctx.roundRect(-rw / 2, -rh / 2, rw, rh, rw * 0.2);
        ctx.fill();
        ctx.stroke();

        ctx.strokeStyle = 'rgba(255,255,255,0.9)';
        ctx.lineWidth   = Math.max(0.8, sw * 0.025);
        ctx.beginPath();
        ctx.moveTo(-rh / 4, 0);
        ctx.lineTo(rh * 0.4 - rh / 4, 0);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(rh * 0.4 - rh / 4, 0);
        ctx.lineTo(rh * 0.2 - rh / 4, -rw * 0.18);
        ctx.moveTo(rh * 0.4 - rh / 4, 0);
        ctx.lineTo(rh * 0.2 - rh / 4,  rw * 0.18);
        ctx.stroke();

        ctx.restore();

        if (sw > 30) {
            ctx.save();
            ctx.font          = `bold ${Math.min(sw * 0.2, 11)}px monospace`;
            ctx.textAlign     = 'center';
            ctx.textBaseline  = 'top';
            ctx.fillStyle     = r.color;
            ctx.fillText(id, cx, sy + sw + 2);
            ctx.restore();
        }
    }
}

function drawAxes() {
    const [ox, oy] = cellToScreen(0, 0);

    ctx.save();
    ctx.strokeStyle = AXIS_COLOR;
    ctx.lineWidth   = 1;

    ctx.beginPath(); ctx.moveTo(0, oy); ctx.lineTo(canvas.width, oy); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(ox, 0); ctx.lineTo(ox, canvas.height); ctx.stroke();

    if (scale > 1) {
        ctx.font          = `${Math.min(scale * 4, 11)}px monospace`;
        ctx.fillStyle     = AXIS_COLOR;
        ctx.textAlign     = 'left';
        ctx.textBaseline  = 'top';
        ctx.fillText('0,0', ox + 3, oy + 2);
    }
    ctx.restore();
}

export function draw() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    ctx.fillStyle = '#f5f5fc';
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    drawCells();
    drawRobots();
}

// ---------------------------------------------------------------------------
// Resize
// ---------------------------------------------------------------------------
function resize() {
    canvas.width  = canvas.offsetWidth  || window.innerWidth;
    canvas.height = canvas.offsetHeight || window.innerHeight;
    draw();
}

window.addEventListener('resize', resize);

function initCenter() {
    canvas.width  = canvas.offsetWidth  || window.innerWidth;
    canvas.height = canvas.offsetHeight || window.innerHeight;
    offsetX = canvas.width  / 2;
    offsetY = canvas.height / 2;
    draw();
}

if (document.readyState === 'loading') {
    window.addEventListener('DOMContentLoaded', initCenter);
} else {
    initCenter();
}

// ---------------------------------------------------------------------------
// Pan + zoom
// ---------------------------------------------------------------------------
let isDragging = false;
let lastX, lastY;
let hoveredCell = null;

canvas.addEventListener('mousedown',  e => { isDragging = true; lastX = e.clientX; lastY = e.clientY; });
canvas.addEventListener('mouseup',    ()  => { isDragging = false; });
canvas.addEventListener('mouseleave', ()  => { isDragging = false; });

canvas.addEventListener('mousemove', e => {
    if (isDragging) {
        offsetX += e.clientX - lastX;
        offsetY += e.clientY - lastY;
        lastX = e.clientX; lastY = e.clientY;
        draw();
        return;
    }
    const rect = canvas.getBoundingClientRect();
    const [col, row] = screenToCell(e.clientX - rect.left, e.clientY - rect.top);
    if (!hoveredCell || hoveredCell[0] !== col || hoveredCell[1] !== row) {
        hoveredCell = [col, row];
        updateSidebar(col, row);
    }
});

canvas.addEventListener('wheel', e => {
    e.preventDefault();
    const zoomFactor = 1.12;
    const rect  = canvas.getBoundingClientRect();
    const mx    = e.clientX - rect.left;
    const my    = e.clientY - rect.top;
    const cellX = (mx - offsetX) / (CELL_SIZE * scale);
    const cellY = (my - offsetY) / (CELL_SIZE * scale);
    scale *= e.deltaY < 0 ? zoomFactor : 1 / zoomFactor;
    scale = Math.max(0.3, Math.min(scale, 20));
    offsetX = mx - cellX * CELL_SIZE * scale;
    offsetY = my - cellY * CELL_SIZE * scale;
    draw();
}, { passive: false });

// ---------------------------------------------------------------------------
// Sidebar
// ---------------------------------------------------------------------------
let selectedCell = null;

canvas.addEventListener('click', e => {
    const rect = canvas.getBoundingClientRect();
    const [col, row] = screenToCell(e.clientX - rect.left, e.clientY - rect.top);
    selectedCell = [col, row];
    updateSidebar(col, row);
});

export function updateSidebar(hoverCol, hoverRow) {
    const sidebar = document.getElementById('sidebar-content');
    if (!sidebar) return;

    const col = selectedCell ? selectedCell[0] : hoverCol;
    const row = selectedCell ? selectedCell[1] : hoverRow;

    if (col === undefined) { sidebar.innerHTML = sidebarEmpty(); return; }

    const cell       = cellMap.get(cellKey(col, row));
    const robotsHere = [...robots.entries()].filter(([, r]) => r.col === col && r.row === row);
    sidebar.innerHTML = sidebarCell(col, row, cell, robotsHere);
}

function statusLabel(status) {
    return status.replace(/_/g, ' ');
}

function sidebarEmpty() {
    return `
        <div class="sb-title">Venus GUI</div>
        <div class="sb-subtitle">Hover or click a cell to inspect it.</div>
        <div class="sb-section-title">Objects</div>
        <div class="sb-legend">
            <div class="sb-legend-item"><span class="sb-icon" style="color:#555">◆</span> Rock Sample</div>
            <div class="sb-legend-item"><span class="sb-icon" style="color:#8b6914">▲</span> Hill</div>
            <div class="sb-legend-item"><span class="sb-icon" style="color:#222">⬟</span> Black Tape</div>
        </div>
        <div class="sb-section-title">Cell Status</div>
        <div class="sb-legend">
            <div class="sb-legend-item"><span class="sb-status-dot" style="background:#f0f0f8"></span> Unknown</div>
            <div class="sb-legend-item"><span class="sb-status-dot" style="background:#eef8f2"></span> Scanned Clear</div>
            <div class="sb-legend-item"><span class="sb-status-dot" style="background:#ffffff;border:1px solid #d0d0dc"></span> Explored</div>
            <div class="sb-legend-item"><span class="sb-status-dot" style="background:#e8eeff"></span> Reserved</div>
            <div class="sb-legend-item"><span class="sb-status-dot" style="background:#ffe8e8"></span> Unsafe</div>
            <div class="sb-legend-item"><span class="sb-status-dot" style="background:#e8e8e8;border:1px solid #d0d0d0"></span> Tape</div>
            <div class="sb-legend-item"><span class="sb-status-dot" style="background:#f5e8d0"></span> Hill</div>
        </div>
    `;
}

function sidebarCell(col, row, cell, robotsHere) {
    const status   = cell?.status ?? 'unknown';
    const contents = cell?.contents ?? [];

    let html = `
        <div class="sb-coords">(${col}, ${row})</div>
        <div class="sb-status sb-status-${status}">${statusLabel(status)}</div>
    `;

    if (robotsHere.length) {
        html += `<div class="sb-section-title">Robots here</div>`;
        for (const [id, r] of robotsHere) {
            const yawDeg = ((r.heading * 180 / Math.PI) % 360).toFixed(1);
            html += `
                <div class="sb-robot" style="border-left:3px solid ${r.color}">
                    <span class="sb-robot-id">${id}</span>
                    <span class="sb-robot-heading">Heading: ${yawDeg}°</span>
                </div>
            `;
        }
    }

    if (contents.length) {
        html += `<div class="sb-section-title">Contents (${contents.length})</div>`;
        for (const item of contents) {
            const info     = CONTENT_ICONS[item.type] ?? { symbol: '?', baseColor: '#aaa' };
            const cssColor = item.type === 'rock_sample'
                ? (SAMPLE_CSS_COLOR[item.color] ?? info.baseColor)
                : (item.color ?? info.baseColor);
            html += `<div class="sb-item">`;
            html += `<span class="sb-item-icon" style="color:${cssColor}">${info.symbol}</span>`;
            html += `<div class="sb-item-details">`;
            html += `<span class="sb-item-type">${item.type.replace(/_/g, ' ').toUpperCase()}</span>`;
            if (item.color)                     html += `<span class="sb-item-attr">Color: ${item.color}</span>`;
            if (item.size   != null)            html += `<span class="sb-item-attr">Size: ${item.size} cm</span>`;
            if (item.temperature != null)       html += `<span class="sb-item-attr">Temp: ${item.temperature}°C</span>`;
            html += `</div></div>`;
        }
    } else if (status !== 'unknown') {
        html += `<div class="sb-empty">No objects detected.</div>`;
    }

    return html;
}

export { updateSidebar as default };
