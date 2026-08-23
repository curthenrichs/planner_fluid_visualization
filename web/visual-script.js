/*
MIT License

Copyright (c) 2017 Pavel Dobryakov

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
Shim for the ROS-era pages (index.html + ros-node.js, spoof.html).
The sim now lives in fluid-sim.js; this file only restores the old
visualize_* globals on top of it. Loaded as a module.
*/

import { initFluidSim } from './fluid-sim.js';

const sim = initFluidSim(document.getElementsByTagName('canvas')[0]);

window.visualize_trajectoryStart = (offsetX, offsetY) => { sim.start({ x: offsetX, y: offsetY }); };
window.visualize_trajectoryUpdate = (offsetX, offsetY) => { sim.move({ x: offsetX, y: offsetY }); };
window.visualize_trajectoryStop = () => { sim.stop(); };
window.visualize_setConfig = (newConfig) => { sim.setConfig(newConfig); };
window.visualize_getConfig = () => sim.getConfig();
