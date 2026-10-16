import { cp, mkdir, readdir, readFile, rm, stat, writeFile } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const sourceDir = path.join(root, 'documents');
const siteDir = path.join(root, 'site');
const outDir = path.join(root, 'dist');

async function markdownFiles(dir, relative = '') {
  let entries;
  try { entries = await readdir(dir, { withFileTypes: true }); }
  catch (error) { if (error.code === 'ENOENT') return []; throw error; }
  const found = [];
  for (const entry of entries) {
    if (entry.name.startsWith('.')) continue;
    const rel = path.posix.join(relative, entry.name);
    if (entry.isDirectory()) found.push(...await markdownFiles(path.join(dir, entry.name), rel));
    else if (entry.isFile() && entry.name.toLowerCase().endsWith('.md')) found.push(rel);
  }
  return found;
}

await rm(outDir, { recursive: true, force: true });
await mkdir(path.join(outDir, 'documents'), { recursive: true });
await cp(siteDir, outDir, { recursive: true });
await cp(sourceDir, path.join(outDir, 'documents'), { recursive: true });
const files = (await markdownFiles(sourceDir)).sort((a, b) => a.localeCompare(b, 'zh-CN'));
const docs = [];
for (const relative of files) {
  const source = await readFile(path.join(sourceDir, relative), 'utf8');
  const heading = source.match(/^#\s+(.+)$/m);
  const filename = path.basename(relative, path.extname(relative));
  const modified = new Date((await stat(path.join(sourceDir, relative))).mtimeMs);
  const date = Number.isNaN(modified.valueOf()) ? '' : modified.toISOString().slice(0, 10);
  docs.push({ path: relative.split(path.sep).join('/'), title: heading ? heading[1].replace(/[*_`]/g, '').trim() : filename, date });
}
await writeFile(path.join(outDir, 'documents.json'), `${JSON.stringify(docs, null, 2)}\n`);
console.log(`Built ${docs.length} Markdown document(s) into dist/.`);
