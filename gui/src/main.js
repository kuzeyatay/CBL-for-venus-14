import './mqtt_communication.js'
import { screenToCell } from './canvas.js'

const canvas = document.getElementById('canvas');
const coordOverlay = document.getElementById('coord-overlay');

canvas.addEventListener('mousemove', e => {
  const rect = canvas.getBoundingClientRect();
  const [col, row] = screenToCell(e.clientX - rect.left, e.clientY - rect.top);
  if (coordOverlay) coordOverlay.textContent = `cell (${col}, ${row})`;
});