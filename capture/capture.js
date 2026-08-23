/*
Capture one full spoofed path loop as a GIF.

Waits for loop 2 (initial splat burst from page load has faded by then),
records CDP screencast frames with timestamps until loop 3 begins, then
resamples to a fixed fps and encodes with gifenc.

Needs the repo served at localhost:8080 (npm start) and these dev deps:
  npm install puppeteer-core pngjs gifenc gifsicle
Then shrink the output:
  node_modules/gifsicle/vendor/gifsicle.exe -O3 --lossy=30 fluid-sim-loop.gif -o out.gif
*/

const puppeteer = require('puppeteer-core');
const { PNG } = require('pngjs');
const { GIFEncoder, quantize, applyPalette } = require('gifenc');
const fs = require('fs');

const WIDTH = 640;
const HEIGHT = 360;
const FPS = 12;
const OUT = 'fluid-sim-loop.gif';

async function waitForLoop(page, n, timeoutMs) {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    const loop = await page.evaluate(() => window.SPOOF_LOOP || 0);
    if (loop >= n) return;
    await new Promise(r => setTimeout(r, 100));
  }
  throw new Error(`timed out waiting for loop ${n}`);
}

(async () => {
  const browser = await puppeteer.launch({
    executablePath: 'C:\\Program Files\\Google\\Chrome\\Application\\chrome.exe',
    headless: 'new',
    args: [`--window-size=${WIDTH},${HEIGHT}`]
  });
  const page = await browser.newPage();
  await page.setViewport({ width: WIDTH, height: HEIGHT });
  await page.goto('http://localhost:8080/web/spoof.html', { waitUntil: 'networkidle0' });

  const cdp = await page.createCDPSession();
  const frames = [];
  let recording = false;

  cdp.on('Page.screencastFrame', async ({ data, metadata, sessionId }) => {
    if (recording) frames.push({ data, ts: metadata.timestamp });
    try { await cdp.send('Page.screencastFrameAck', { sessionId }); } catch (e) {}
  });

  console.log('waiting for loop 2...');
  await waitForLoop(page, 2, 60000);

  recording = true;
  await cdp.send('Page.startScreencast', {
    format: 'png', everyNthFrame: 1,
    maxWidth: WIDTH, maxHeight: HEIGHT
  });
  console.log('recording loop 2...');

  await waitForLoop(page, 3, 60000);
  recording = false;
  await cdp.send('Page.stopScreencast');
  await browser.close();

  console.log(`captured ${frames.length} frames over ${(frames[frames.length - 1].ts - frames[0].ts).toFixed(1)}s`);

  // resample to fixed fps by nearest captured frame
  const t0 = frames[0].ts;
  const duration = frames[frames.length - 1].ts - t0;
  const step = 1 / FPS;
  const picked = [];
  let idx = 0;
  for (let t = 0; t <= duration; t += step) {
    while (idx < frames.length - 1 && Math.abs(frames[idx + 1].ts - t0 - t) <= Math.abs(frames[idx].ts - t0 - t)) idx++;
    picked.push(frames[idx]);
  }
  console.log(`encoding ${picked.length} frames at ${FPS} fps...`);

  const gif = GIFEncoder();
  const delay = Math.round(1000 / FPS);
  for (let i = 0; i < picked.length; i++) {
    const png = PNG.sync.read(Buffer.from(picked[i].data, 'base64'));
    const rgba = new Uint8ClampedArray(png.data.buffer, png.data.byteOffset, png.data.length);
    const palette = quantize(rgba, 256);
    const index = applyPalette(rgba, palette);
    gif.writeFrame(index, png.width, png.height, { palette, delay });
    if (i % 25 === 0) console.log(`  frame ${i}/${picked.length}`);
  }
  gif.finish();

  fs.writeFileSync(OUT, Buffer.from(gif.bytes()));
  const mb = fs.statSync(OUT).size / 1024 / 1024;
  console.log(`wrote ${OUT} (${mb.toFixed(2)} MB)`);
})().catch(e => { console.error(e); process.exit(1); });
