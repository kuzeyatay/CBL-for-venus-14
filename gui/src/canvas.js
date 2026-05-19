// cell based canvas
// integer cell coordinates, col increasing: right, row increasing: down

const CELL_SIZE = 48; // pixels per cell rendering
const GRID_COLOR = '#d0d0dc';
const UNEXPLORED_COLOR = '#f0f0f8';
const EXPLORED_EMPTY_COLOR = '#ffffff';
const AXIS_COLOR = '#a0a0b8';

let offsetX = 0;
let offsetY = 0;
let scale = 3;

const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

// cellMap: key = "col,row", value = { explored, contents: [ ... ] }
const cellMap = new Map();

function cellKey(col, row) { return `${col},${row}`; }

function getCell(col, row) {
  const k = cellKey(col, row);
  if (!cellMap.has(k)) cellMap.set(k, { explored: false, contents: [] });
  return cellMap.get(k);
}

function markExplored(col, row) {
  getCell(col, row).explored = true;
}

// item: { type: 'rock'|'hill'|'cliff', color?, size?, temperature? }

export function addCellContent(col, row, item) {
  const cell = getCell(col, row);
  cell.explored = true;

  // Avoid 2 of same exact item in same cell (may have to modify based on irl cell size, however making irl cells small seems best)
  const isDupe = cell.contents.some(c =>
    c.type === item.type && c.color === item.color
  );
  if (!isDupe) {
    cell.contents.push(item);
  }
  draw();
}

// robot stuff
const robots = new Map(); // key: robotId, value: { col, row, heading, color }

export function updateRobot(robotId, col, row, heading) {
  if (!robots.has(robotId)) {
    const colors = ['#c0392b', '#2980b9'];
    const colorIndex = robots.size % colors.length;
    robots.set(robotId, { col, row, heading, color: colors[colorIndex] });
  } else {
    const r = robots.get(robotId);
    r.col = col; r.row = row; r.heading = heading;
  }
  markExplored(col, row);
  draw();
  updateSidebar();
}

// screen/cell conversion
function cellToScreen(col, row) {
  return [col * CELL_SIZE * scale + offsetX, row * CELL_SIZE * scale + offsetY];
}

export function screenToCell(screenX, screenY) {
  return [
    Math.floor((screenX - offsetX) / (CELL_SIZE * scale)),
    Math.floor((screenY - offsetY) / (CELL_SIZE * scale))
  ];
}

// drawing!
const CONTENT_ICONS = {
  rock:  { symbol: '◆', baseColor: '#555' },
  hill:  { symbol: '▲', baseColor: '#8b6914' },
  cliff: { symbol: '⬡', baseColor: '#7c4daa' },
};

function drawCells() {
  const [x0, y0] = screenToCell(0, 0);
  const [x1, y1] = screenToCell(canvas.width, canvas.height);
  const pad = 2;
  const sw = CELL_SIZE * scale;

  for (let col = x0 - pad; col <= x1 + pad; col++) {
    for (let row = y0 - pad; row <= y1 + pad; row++) {
      const [sx, sy] = cellToScreen(col, row);

      const cell = cellMap.get(cellKey(col, row));
      const explored = cell?.explored ?? false;

      // cell bg
      ctx.fillStyle = explored ? EXPLORED_EMPTY_COLOR : UNEXPLORED_COLOR;
      ctx.fillRect(sx, sy, sw, sw);

      // cell border
      ctx.strokeStyle = GRID_COLOR;
      ctx.lineWidth = 0.5;
      ctx.strokeRect(sx + 0.25, sy + 0.25, sw - 0.5, sw - 0.5);

      if (!explored || !cell?.contents.length) continue;

      const contents = cell.contents;

      if (contents.length === 1) {
        const item = contents[0];
        const info = CONTENT_ICONS[item.type];
        const fontSize = Math.max(8, Math.min(sw * 0.45, 28));
        ctx.save();
        ctx.font = `${fontSize}px monospace`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = item.color ?? info.baseColor;
        ctx.fillText(info.symbol, sx + sw / 2, sy + sw / 2);
        ctx.restore();
      } else {
        // multiple, write how many items there are instead
        const fontSize = Math.max(7, Math.min(sw * 0.28, 16));
        ctx.save();
        ctx.font = `${fontSize}px monospace`;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillStyle = '#888';
        ctx.fillText(`×${contents.length}`, sx + sw / 2, sy + sw / 2);
        ctx.restore();
      }

      // coordinates on botto right when zoomed in enough
      if (sw > 56) {
        ctx.save();
        ctx.font = `${Math.min(sw * 0.16, 10)}px monospace`;
        ctx.textAlign = 'right';
        ctx.textBaseline = 'bottom';
        ctx.fillStyle = '#c0c0d0';
        ctx.fillText(`${col},${row}`, sx + sw - 3, sy + sw - 2);
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
    ctx.rotate(-r.heading);

    // robot body
    ctx.fillStyle = r.color;
    ctx.strokeStyle = 'rgba(0,0,0,0.25)';
    ctx.lineWidth = Math.max(0.5, sw * 0.02);
    ctx.beginPath();
    ctx.roundRect(-rw / 2, -rh / 2, rw, rh, rw * 0.2);
    ctx.fill();
    ctx.stroke();

    // heading arrow
    ctx.strokeStyle = 'rgba(255,255,255,0.9)';
    ctx.lineWidth = Math.max(0.8, sw * 0.025);
    ctx.beginPath();
    ctx.moveTo(-rh / 4, 0);
    ctx.lineTo(rh * 0.4 - rh / 4, 0);
    ctx.stroke();

    // arrow head
    ctx.beginPath();
    ctx.moveTo(rh * 0.4 - rh / 4, 0);
    ctx.lineTo(rh * 0.2 - rh / 4, -rw * 0.18);
    ctx.moveTo(rh * 0.4 - rh / 4, 0);
    ctx.lineTo(rh * 0.2 - rh / 4,  rw * 0.18);
    ctx.stroke();

    ctx.restore();

    // show robot id below robot
    if (sw > 30) {
      ctx.save();
      ctx.font = `bold ${Math.min(sw * 0.2, 11)}px monospace`;
      ctx.textAlign = 'center';
      ctx.textBaseline = 'top';
      ctx.fillStyle = r.color;
      ctx.fillText(`R${id}`, cx, sy + sw + 2);
      ctx.restore();
    }
  }
}

function drawAxes() {
  const [ox, oy] = cellToScreen(0, 0);

  ctx.save();
  ctx.strokeStyle = AXIS_COLOR;
  ctx.lineWidth = 1;

  ctx.beginPath();
  ctx.moveTo(0, oy);
  ctx.lineTo(canvas.width, oy);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(ox, 0);
  ctx.lineTo(ox, canvas.height);
  ctx.stroke();

  if (scale > 1) {
    ctx.font = `${Math.min(scale * 4, 11)}px monospace`;
    ctx.fillStyle = AXIS_COLOR;
    ctx.textAlign = 'left';
    ctx.textBaseline = 'top';
    ctx.fillText('0,0', ox + 3, oy + 2);
  }

  ctx.restore();
}

function draw() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  ctx.fillStyle = '#f5f5fc';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  drawCells();
  drawAxes();
  drawRobots();
}

// window resizing
function resize() {
  canvas.width  = window.innerWidth;
  canvas.height = window.innerHeight;
  draw();
}
window.addEventListener('resize', resize);

window.addEventListener('DOMContentLoaded', () => {
  canvas.width  = window.innerWidth;
  canvas.height = window.innerHeight;
  offsetX = canvas.width  / 2;
  offsetY = canvas.height / 2;
  draw();
});
resize();

// panning
let isDragging = false;
let lastX, lastY;
let hoveredCell = null;

canvas.addEventListener('mousedown', e => {
  isDragging = true;
  lastX = e.clientX; lastY = e.clientY;
});
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
canvas.addEventListener('mouseup',    () => isDragging = false);
canvas.addEventListener('mouseleave', () => isDragging = false);

canvas.addEventListener('mousemove', e => {
  const rect = canvas.getBoundingClientRect();
  const [col, row] = screenToCell(e.clientX - rect.left, e.clientY - rect.top);
  selectCell(col, row);
});

// zooming
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const zoomFactor = 1.12;
  const rect = canvas.getBoundingClientRect();
  const mx = e.clientX - rect.left;
  const my = e.clientY - rect.top;
  const cellX = (mx - offsetX) / (CELL_SIZE * scale);
  const cellY = (my - offsetY) / (CELL_SIZE * scale);
  scale *= e.deltaY < 0 ? zoomFactor : 1 / zoomFactor;
  scale = Math.max(0.3, Math.min(scale, 20));
  offsetX = mx - cellX * CELL_SIZE * scale;
  offsetY = my - cellY * CELL_SIZE * scale;
  draw();
}, { passive: false });

// sidebar
let selectedCell = null;

function selectCell(col, row) {
  selectedCell = [col, row];
  updateSidebar(col, row);
}

function updateSidebar(hoverCol, hoverRow) {
  const sidebar = document.getElementById('sidebar');
  if (!sidebar) return;

  const targetCol = selectedCell ? selectedCell[0] : hoverCol;
  const targetRow = selectedCell ? selectedCell[1] : hoverRow;

  if (targetCol === undefined) {
    sidebar.innerHTML = sidebarEmpty();
    return;
  }

  const cell = cellMap.get(cellKey(targetCol, targetRow));
  const robotsHere = [...robots.entries()].filter(([, r]) => r.col === targetCol && r.row === targetRow);

  sidebar.innerHTML = sidebarCell(targetCol, targetRow, cell, robotsHere);
}

function sidebarEmpty() {
  return `
    <div class="sb-title">Venus GUI</div>
    <div class="sb-subtitle">Hover or click a cell to inspect it.</div>
    <div class="sb-legend">
      <div class="sb-legend-item"><span class="sb-icon" style="color:#555">◆</span> Rock</div>
      <div class="sb-legend-item"><span class="sb-icon" style="color:#8b6914">▲</span> Hill</div>
      <div class="sb-legend-item"><span class="sb-icon" style="color:#7c4daa">⬡</span> Cliff</div>
    </div>
  `;
}

function sidebarCell(col, row, cell, robotsHere) {
  const explored = cell?.explored ?? false;
  const contents = cell?.contents ?? [];

  let html = `
    <div class="sb-coords">(${col}, ${row})</div>
    <div class="sb-status ${explored ? 'explored' : 'unexplored'}">${explored ? 'Explored' : 'Unexplored'}</div>
  `;

  if (robotsHere.length) {
    html += `<div class="sb-section-title">Robots here</div>`;
    for (const [id, r] of robotsHere) {
      html += `
        <div class="sb-robot" style="border-left: 3px solid ${r.color}">
          <span class="sb-robot-id">Robot ${id}</span>
          <span class="sb-robot-heading">Heading: ${((r.heading * 180 / Math.PI) % 360).toFixed(1)}°</span>
        </div>
      `;
    }
  }

  if (contents.length) {
    html += `<div class="sb-section-title">Contents (${contents.length})</div>`;
    for (const item of contents) {
      const info = CONTENT_ICONS[item.type] ?? {};
      html += `<div class="sb-item">`;
      html += `<span class="sb-item-icon" style="color:${item.color ?? info.baseColor}">${info.symbol}</span>`;
      html += `<div class="sb-item-details">`;
      html += `<span class="sb-item-type">${item.type.toUpperCase()}</span>`;
      if (item.color)                     html += `<span class="sb-item-attr">Color: ${item.color}</span>`;
      if (item.size)                      html += `<span class="sb-item-attr">Size: ${item.size}</span>`;
      if (item.temperature !== undefined) html += `<span class="sb-item-attr">Temp: ${item.temperature}°</span>`;
      html += `</div></div>`;
    }
  } else if (explored) {
    html += `<div class="sb-empty">No objects detected.</div>`;
  }

  return html;
}

export { draw, updateSidebar };
