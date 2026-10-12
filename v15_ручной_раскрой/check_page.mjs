// Static code/UI contract checks only. This is NOT a browser render test.
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';

const root = path.dirname(fileURLToPath(import.meta.url));
const page = fs.readFileSync(path.join(root, 'index.html'), 'utf8');
const scripts = [...page.matchAll(/<script[^>]*>([\s\S]*?)<\/script>/g)];
assert.equal(scripts.length, 1);
const elements = {};
function element(id) {
  return elements[id] ??= { style: {}, dataset: {}, textContent: '', innerHTML: '',
    classes: new Set(), classList: {
      toggle(name, on) { on ? elements[id].classes.add(name) : elements[id].classes.delete(name); },
      remove(name) { elements[id].classes.delete(name); }
    }
  };
}
const buttons = ['side', 'inner', 'rear', 'lower', 'upper'].map(key => {
  const e = element(key); e.dataset.part = key; return e;
});
element('model-frame').dataset.src = 'viewer.html';
const caption = element('caption');
const context = vm.createContext({ document: {
  getElementById: element,
  querySelectorAll(selector) { assert.equal(selector, '[data-part]'); return buttons; },
  querySelector(selector) { assert.equal(selector, '#diagram + .caption'); return caption; }
}});
vm.runInContext(scripts[0][1], context);
for (const b of buttons) {
  b.onclick();
  assert(element('diagram').innerHTML.includes('<svg'));
  assert(b.classes.has('active'));
  assert(fs.existsSync(path.join(root, element('active-download').href)));
}
buttons[2].onclick();
assert(caption.textContent.includes('левого края'));
element('load-model').onclick();
assert.equal(element('model-frame').src, 'viewer.html');
assert.equal(element('load-model').style.display, 'none');
for (const [, href] of page.matchAll(/href="([^"]+)"/g)) {
  assert(fs.existsSync(path.join(root, href)), href);
}
const viewer = fs.readFileSync(path.join(root, 'viewer.html'), 'utf8');
const modules = [...viewer.matchAll(/<script type="module">([\s\S]*?)<\/script>/g)];
assert.equal(modules.length, 1);
const module = new vm.SourceTextModule(modules[0][1]);
assert.equal(module.dependencySpecifiers.length, 0, 'Viewer must not fetch ESM imports under file://');
assert(!/\b(?:559|517)\.00\s*\//.test(viewer));
assert(!viewer.includes('toFixed(2)'));
assert(!viewer.includes('70×Ø47'));
console.log('PASS: five drawing selectors, matching download files, rear origin caption, lazy 3D button, all links, embedded module syntax and zero external imports. Not a browser rendering test.');
