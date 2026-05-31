import { screenToCell } from './canvas.js';
import { setTerminalElement, sendMessage } from './client.js';

const canvas      = document.getElementById('canvas');
const coordOverlay = document.getElementById('coord-overlay');
const terminalLog  = document.getElementById('terminal-log');

if (terminalLog) setTerminalElement(terminalLog);

document.getElementById('btn-start')?.addEventListener('click', () => sendMessage('START_MISSION'));
document.getElementById('btn-stop') ?.addEventListener('click', () => sendMessage('STOP_MISSION'));
document.getElementById('btn-ping') ?.addEventListener('click', () => sendMessage('PING'));

document.getElementById('btn-clear-log')?.addEventListener('click', () => {
    if (terminalLog) terminalLog.innerHTML = '';
});

canvas.addEventListener('mousemove', e => {
    const rect = canvas.getBoundingClientRect();
    const [col, row] = screenToCell(e.clientX - rect.left, e.clientY - rect.top);
    if (coordOverlay) coordOverlay.textContent = `cell (${col}, ${row})`;
});
