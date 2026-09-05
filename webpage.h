#ifndef WEBPAGE_H
#define WEBPAGE_H

#include <Arduino.h>

const char WEBPAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>RECON-1</title>
<style>
html,body{
  margin:0;
  padding:0;
  width:100%;
  height:100%;
  background:#000;
  overflow:hidden;
  font-family:Arial,sans-serif;
}
body{
  position:relative;
  display:flex;
  align-items:center;
  justify-content:center;
}
#thermal{
  display:block;
  width:100vw;
  height:100vh;
  object-fit:contain;
  background:#000;
}
#cameraBox{
  position:absolute;
  top:14px;
  right:14px;
  width:min(31vw,420px);
  aspect-ratio:4/3;
  background:#111;
  border:2px solid #fff;
  border-radius:8px;
  overflow:hidden;
  box-shadow:0 0 18px rgba(0,0,0,.7);
}
#camera{
  width:100%;
  height:100%;
  display:block;
  object-fit:cover;
}
#controls{
  position:absolute;
  right:14px;
  bottom:14px;
  display:grid;
  grid-template-columns:110px 110px;
  gap:10px;
}
.control{
  width:110px;
  height:68px;
  border:2px solid #fff;
  border-radius:12px;
  background:rgba(20,20,20,.82);
  color:#fff;
  font-size:27px;
  font-weight:700;
  user-select:none;
  -webkit-user-select:none;
  touch-action:none;
  cursor:pointer;
}
.control:active{
  background:rgba(255,255,255,.25);
}
#status{
  position:absolute;
  left:14px;
  top:14px;
  padding:7px 10px;
  color:#fff;
  background:rgba(0,0,0,.55);
  border:1px solid rgba(255,255,255,.5);
  border-radius:6px;
  font-size:14px;
}
@media (max-width:700px){
  #cameraBox{
    width:40vw;
    top:8px;
    right:8px;
  }
  #controls{
    right:8px;
    bottom:8px;
    grid-template-columns:86px 86px;
    gap:7px;
  }
  .control{
    width:86px;
    height:58px;
    font-size:24px;
  }
  #status{
    left:8px;
    top:8px;
    font-size:12px;
  }
}
</style>
</head>
<body>
<canvas id="thermal" width="512" height="384"></canvas>

<div id="cameraBox">
  <img id="camera" src="/stream" alt="Camera feed">
</div>

<div id="status">RECON-1 • COMMAND LINK: READY</div>

<div id="controls">
  <button class="control" data-command="forward">↑</button>
  <button class="control" data-command="backward">↓</button>
  <button class="control" data-command="left">←</button>
  <button class="control" data-command="right">→</button>
</div>

<script>
const SENSOR_W = 32;
const SENSOR_H = 24;
const OUT_W = 512;
const OUT_H = 384;
const FETCH_DELAY_MS = 85;
const TEMPORAL_AMOUNT = 0.35;
const COMMAND_REPEAT_MS = 180;

const canvas = document.getElementById("thermal");
const ctx = canvas.getContext("2d", {alpha:false});
const imageData = ctx.createImageData(OUT_W, OUT_H);
const statusEl = document.getElementById("status");

let temporalPixels = null;
let isFetching = false;
let commandTimer = null;
let activeCommand = null;

function clamp(v, lo, hi){
  return Math.max(lo, Math.min(hi, v));
}

function lerp(a, b, t){
  return a + (b - a) * t;
}

function getPixel(pixels, x, y){
  x = clamp(x, 0, SENSOR_W - 1);
  y = clamp(y, 0, SENSOR_H - 1);
  return pixels[y * SENSOR_W + x];
}

function sampleBilinear(pixels, x, y){
  const x0 = Math.floor(x);
  const y0 = Math.floor(y);
  const x1 = Math.min(x0 + 1, SENSOR_W - 1);
  const y1 = Math.min(y0 + 1, SENSOR_H - 1);
  const fx = x - x0;
  const fy = y - y0;

  const p00 = getPixel(pixels, x0, y0);
  const p10 = getPixel(pixels, x1, y0);
  const p01 = getPixel(pixels, x0, y1);
  const p11 = getPixel(pixels, x1, y1);

  return lerp(lerp(p00, p10, fx), lerp(p01, p11, fx), fy);
}

function applyTemporalSmoothing(newPixels){
  if(!temporalPixels || temporalPixels.length !== newPixels.length){
    temporalPixels = newPixels.slice();
    return temporalPixels;
  }

  for(let i = 0; i < newPixels.length; i++){
    temporalPixels[i] += TEMPORAL_AMOUNT * (newPixels[i] - temporalPixels[i]);
  }
  return temporalPixels;
}

function thermalColor(t){
  t = clamp(t, 0, 1);
  const stops = [
    [0.00, [0,0,12]],
    [0.18, [34,6,75]],
    [0.36, [132,20,78]],
    [0.55, [224,58,48]],
    [0.72, [255,151,31]],
    [0.88, [255,243,105]],
    [1.00, [255,255,255]]
  ];

  for(let i = 0; i < stops.length - 1; i++){
    const a = stops[i];
    const b = stops[i + 1];
    if(t >= a[0] && t <= b[0]){
      const k = (t - a[0]) / (b[0] - a[0]);
      return [
        lerp(a[1][0], b[1][0], k),
        lerp(a[1][1], b[1][1], k),
        lerp(a[1][2], b[1][2], k)
      ];
    }
  }
  return stops[stops.length - 1][1];
}

function drawThermal(data){
  const pixels = applyTemporalSmoothing(data.pixels);
  const displayMin = data.min;
  const displayMax = data.max;
  const span = Math.max(0.25, displayMax - displayMin);

  for(let py = 0; py < OUT_H; py++){
    const sy = (py / (OUT_H - 1)) * (SENSOR_H - 1);
    for(let px = 0; px < OUT_W; px++){
      const sx = (px / (OUT_W - 1)) * (SENSOR_W - 1);
      const temp = sampleBilinear(pixels, sx, sy);
      const normalized = (temp - displayMin) / span;
      const c = thermalColor(normalized);
      const o = (py * OUT_W + px) * 4;
      imageData.data[o + 0] = c[0];
      imageData.data[o + 1] = c[1];
      imageData.data[o + 2] = c[2];
      imageData.data[o + 3] = 255;
    }
  }

  ctx.putImageData(imageData, 0, 0);
}

async function updateFrame(){
  if(isFetching) return;
  isFetching = true;

  try{
    const response = await fetch("/data?ts=" + Date.now(), {cache:"no-store"});
    const data = await response.json();
    if(data.ok) drawThermal(data);
  }catch(e){
    // Keep last valid frame visible.
  }finally{
    isFetching = false;
    setTimeout(updateFrame, FETCH_DELAY_MS);
  }
}

async function sendCommand(command){
  try{
    const response = await fetch("/move?cmd=" + encodeURIComponent(command) + "&ts=" + Date.now(), {cache:"no-store"});
    if(!response.ok) throw new Error("HTTP " + response.status);
    statusEl.textContent = "RECON-1 • MOVING: " + command.toUpperCase();
  }catch(e){
    statusEl.textContent = "RECON-1 • COMMAND LINK ERROR";
  }
}

async function stopRobot(){
  if(commandTimer){
    clearInterval(commandTimer);
    commandTimer = null;
  }
  activeCommand = null;
  await sendCommand("stop");
  statusEl.textContent = "RECON-1 • COMMAND LINK: READY";
}

function beginCommand(command){
  if(command === activeCommand) return;
  if(commandTimer) clearInterval(commandTimer);
  activeCommand = command;
  sendCommand(command);
  commandTimer = setInterval(() => {
    if(activeCommand) sendCommand(activeCommand);
  }, COMMAND_REPEAT_MS);
}

document.querySelectorAll(".control").forEach(button => {
  const command = button.dataset.command;
  button.addEventListener("pointerdown", event => {
    event.preventDefault();
    beginCommand(command);
  });
});

window.addEventListener("pointerup", event => {
  if(activeCommand){
    event.preventDefault();
    stopRobot();
  }
});
window.addEventListener("pointercancel", stopRobot);
window.addEventListener("blur", stopRobot);

updateFrame();
</script>
</body>
</html>
)rawliteral";

#endif
