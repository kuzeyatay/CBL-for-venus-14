import { updateRobot, addCellContent, updateCellStatus } from './canvas.js';

const WS_URL = 'ws://localhost:8765';
const RECONNECT_DELAY_MS = 3000;

let socket = null;
let terminalEl = null;

export function setTerminalElement(el) {
    terminalEl = el;
}

function terminalAppend(text, cls) {
    if (!terminalEl) return;
    const now = new Date().toTimeString().slice(0, 8);
    const line = document.createElement('div');
    line.className = `term-line ${cls}`;
    line.textContent = `[${now}] ${text}`;
    terminalEl.appendChild(line);
    if (terminalEl.children.length > 500) terminalEl.removeChild(terminalEl.firstChild);
    terminalEl.scrollTop = terminalEl.scrollHeight;
}

function terminalRecv(raw) { terminalAppend(`← ${raw}`, 'term-recv'); }
function terminalSend(raw) { terminalAppend(`→ ${raw}`, 'term-send'); }
function terminalInfo(msg) { terminalAppend(`  ${msg}`,  'term-info'); }

// ---------------------------------------------------------------------------
// Inbound message parsing
// Protocol defined in comms/communication.c
//
//  POSE,<id>,<x_cm>,<y_cm>,<yaw_deg>,<cell_x>,<cell_y>
//  EVENT,<id>,<type>,<x_cm>,<y_cm>,<yaw_deg>,<cell_x>,<cell_y>,<dist_mm>,<width_cm>,<color>,<size_cm>,<temp_c>
//  CELL_UPDATE,<id>,<cell_x>,<cell_y>,<status>
//  STATUS,<id>,<state>,-1,0,<error_code>
//  ERROR,<id>,<error_code>
//  MISSION_STARTED,<id>
//  PONG,<id>
// ---------------------------------------------------------------------------
function parseCSVMessage(raw) {
    const parts = raw.trim().split(',');
    if (parts.length < 2) return;
    const type = parts[0];

    switch (type) {
        case 'POSE': {
            if (parts.length < 7) { console.warn('[client] Malformed POSE:', raw); return; }
            const robotId = parts[1];
            const yawDeg  = parseFloat(parts[4]);
            const cellX   = parseInt(parts[5], 10);
            const cellY   = parseInt(parts[6], 10);
            if (isNaN(yawDeg) || isNaN(cellX) || isNaN(cellY)) {
                console.warn('[client] POSE NaN:', raw); return;
            }
            updateRobot(robotId, cellX, cellY, yawDeg * Math.PI / 180);
            break;
        }

        case 'EVENT': {
            // EVENT,<id>,<type>,<x>,<y>,<yaw>,<cell_x>,<cell_y>,<dist>,<width>,<color>,<size>,<temp>
            if (parts.length < 8) { console.warn('[client] Malformed EVENT:', raw); return; }
            const eventType = parts[2].trim().toLowerCase();
            const cellX     = parseInt(parts[6], 10);
            const cellY     = parseInt(parts[7], 10);
            if (isNaN(cellX) || isNaN(cellY)) break;

            if (eventType === 'rock_sample') {
                const color = (parts[10] ?? 'unknown').trim().toLowerCase();
                const size  = parseInt(parts[11], 10);
                const temp  = parseFloat(parts[12]);
                addCellContent(cellX, cellY, {
                    type: 'rock_sample',
                    color,
                    size:        isNaN(size) ? undefined : size,
                    temperature: isNaN(temp) ? undefined : temp,
                });
            } else if (eventType === 'hill') {
                addCellContent(cellX, cellY, { type: 'hill' });
            } else if (eventType === 'black_tape') {
                addCellContent(cellX, cellY, { type: 'black_tape' });
            }
            break;
        }

        case 'CELL_UPDATE': {
            // CELL_UPDATE,<id>,<cell_x>,<cell_y>,<status>
            if (parts.length < 5) { console.warn('[client] Malformed CELL_UPDATE:', raw); return; }
            const cellX  = parseInt(parts[2], 10);
            const cellY  = parseInt(parts[3], 10);
            const status = parts[4].trim().toLowerCase();
            if (!isNaN(cellX) && !isNaN(cellY)) updateCellStatus(cellX, cellY, status);
            break;
        }

        case 'STATUS': {
            // STATUS,<id>,<state>,-1,0,<error_code>
            const robotId   = parts[1] ?? '?';
            const state     = parts[2] ?? 'unknown';
            const errorCode = parts[5] ?? 'none';
            console.log(`[client] Robot ${robotId} status: ${state}, error: ${errorCode}`);
            break;
        }

        case 'ERROR': {
            const robotId   = parts[1] ?? '?';
            const errorCode = parts[2] ?? 'unknown';
            console.warn(`[client] Robot ${robotId} error: ${errorCode}`);
            break;
        }

        case 'LOG': {
            const robotId = parts[1] ?? '?';
            const level   = (parts[2] ?? 'INFO').trim().toUpperCase();
            const message = parts.slice(3).join(',').trim();
            const cls = level === 'ERROR' ? 'term-log-error'
                      : level === 'WARN'  ? 'term-log-warn'
                      :                     'term-log-info';
            terminalAppend(`[${robotId}] ${level}: ${message}`, cls);
            break;
        }

        case 'MISSION_STARTED':
        case 'PONG':
            console.log(`[client] ${type} from robot ${parts[1] ?? '?'}`);
            break;

        default:
            console.log('[client] Unknown message type:', raw);
    }
}

// ---------------------------------------------------------------------------
// Outbound
// ---------------------------------------------------------------------------
export function sendMessage(msg) {
    if (!socket || socket.readyState !== WebSocket.OPEN) {
        terminalInfo(`Cannot send "${msg}" — not connected`);
        return;
    }
    socket.send(msg);
    terminalSend(msg);
}

// ---------------------------------------------------------------------------
// WebSocket lifecycle
// ---------------------------------------------------------------------------
function connect() {
    socket = new WebSocket(WS_URL);

    socket.onopen = () => {
        terminalInfo(`Connected to ${WS_URL}`);
        console.log('[client] WebSocket connected');
        updateConnectionIndicator(true);
    };

    socket.onmessage = (event) => {
        const raw = event.data;
        if (!raw.startsWith('LOG,')) terminalRecv(raw);
        parseCSVMessage(raw);
    };

    socket.onclose = (event) => {
        terminalInfo(`Disconnected (code ${event.code}). Reconnecting in ${RECONNECT_DELAY_MS / 1000}s…`);
        updateConnectionIndicator(false);
        setTimeout(connect, RECONNECT_DELAY_MS);
    };

    socket.onerror = () => {
        terminalInfo('WebSocket error — check server');
    };
}

function updateConnectionIndicator(connected) {
    const dot = document.getElementById('conn-dot');
    const label = document.getElementById('conn-label');
    if (dot)   dot.style.background   = connected ? '#27ae60' : '#c0392b';
    if (label) label.textContent       = connected ? 'Connected' : 'Disconnected';
}

connect();
