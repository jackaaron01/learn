# 个人 Markdown 阅读站

一个适配手机、平板和桌面浏览器的极简 Markdown 阅读站。当前文档放在 [`documents/`](documents/)；构建时会自动扫描其中所有 `.md` 文件并生成目录。

## 在线阅读

[https://learn-785.pages.dev](https://learn-785.pages.dev)

## 添加文档

把 Markdown 文件放进 `documents/`，也可以放进它的子文件夹。提交并推送到 `main` 后，GitHub Actions 会重新构建站点并发布更新。

```bash
cp /path/to/your-note.md documents/
git add documents/
git commit -m "add reading notes"
git push
```

## 本地预览

需要 Node.js 18 或更新版本：

```bash
node scripts/build.mjs
python3 -m http.server 8000 --directory dist
```

然后打开 `http://localhost:8000`。Markdown 排版和公式组件从 jsDelivr 加载，预览和阅读时需要联网。

## Cloudflare Pages 发布

当前站点地址为 [learn-785.pages.dev](https://learn-785.pages.dev)。仓库中的 [`wrangler.toml`](wrangler.toml) 固定了 Pages 项目名和输出目录。Cloudflare Pages 项目需连接此 GitHub 仓库，生产分支使用 `main`，根目录留空，构建命令设为 `node scripts/build.mjs`。推送到 `main` 后，Cloudflare 会自动重新构建并更新在线站点。

若根地址仍返回 404，请到 Cloudflare **Workers & Pages → learn-785 → Deployments → View details → Build log** 检查最近一次构建是否生成了 `dist/index.html`。
