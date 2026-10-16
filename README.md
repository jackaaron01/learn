# 个人 Markdown 阅读站

纯静态前端阅读器：页面从 GitHub 仓库读取 `documents/` 下的 Markdown 文件，不维护单独的文档清单，也不需要服务端保存文档。

在线地址：[Cloudflare Pages](https://learn-785.pages.dev) · [GitHub Pages](https://jackaaron01.github.io/learn/)

## 添加文档

把 `.md` 文件放进仓库的 [`documents/`](documents/) 目录，可继续按子目录分类。提交并推送到 `main` 后，阅读器会自动发现新文档并读取内容。目录和正文通过 jsDelivr 从 GitHub 仓库加载，避免依赖额度有限的 GitHub REST API。

```bash
cp /path/to/your-note.md documents/
git add documents/
git commit -m "add reading notes"
git push
```

仓库需要公开，浏览器才能通过 GitHub API 和 Raw 内容地址读取文档。网页排版、公式和文档列表均在浏览器端完成。

## 本地预览

在仓库根目录启动任意静态文件服务器，例如：

```bash
python3 -m http.server 8000
```

打开 `http://localhost:8000`。Markdown 排版和数学公式依赖 jsDelivr，预览和阅读时需要联网。

## 部署

### GitHub Pages

仓库包含 GitHub Actions 发布工作流。首次启用时，在 **Settings → Pages → Build and deployment → Source** 选择 **GitHub Actions**；之后推送到 `main` 会自动发布根目录静态页面。首次设置完成并等待工作流成功后，GitHub Pages 地址才会可用。

### Cloudflare Pages

Cloudflare Pages 连接此仓库，生产分支使用 `main`，根目录留空，Framework preset 选 `None`，Build command 留空，Build output directory 设为 `/`。这是纯静态前端，不需要构建步骤；网页和 Markdown 均由浏览器端从仓库/CDN 加载。
