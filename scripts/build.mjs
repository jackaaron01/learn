import { cp, mkdir, rm } from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
const outDir = path.join(root, 'dist');

await rm(outDir, { recursive: true, force: true });
await mkdir(outDir, { recursive: true });
for (const file of ['index.html', 'app.js', 'styles.css', 'overrides.css', '.nojekyll']) {
  await cp(path.join(root, file), path.join(outDir, file));
}
console.log('Copied the static frontend into dist/. Markdown content loads directly from GitHub.');
