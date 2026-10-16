(() => {
  const repository = 'jackaaron01/learn';
  const branch = 'main';
  const treeUrl = `https://api.github.com/repos/${repository}/git/trees/${branch}?recursive=1`;
  const rawBase = `https://raw.githubusercontent.com/${repository}/${branch}/`;
  const list = document.querySelector('#document-list');
  const search = document.querySelector('#search');
  const count = document.querySelector('#document-count');
  const welcome = document.querySelector('#welcome');
  const articleView = document.querySelector('#article-view');
  const markdownBody = document.querySelector('#markdown-body');
  const currentTitle = document.querySelector('#current-title');
  const errorState = document.querySelector('#error-state');
  const tocPanel = document.querySelector('#toc-panel');
  const toc = document.querySelector('#toc');
  const sidebar = document.querySelector('#sidebar');
  const backdrop = document.querySelector('#sidebar-backdrop');
  const menuToggle = document.querySelector('#menu-toggle');
  let documents = [];
  let activePath = '';

  const closeMenu = () => {
    sidebar.classList.remove('open'); backdrop.classList.remove('show'); menuToggle.setAttribute('aria-expanded', 'false');
  };
  const safeText = (value) => String(value ?? '');
  const repoFileUrl = (filePath) => new URL(filePath.split('/').map(encodeURIComponent).join('/'), rawBase);
  const titleFromMarkdown = (markdown, fallback) => {
    const heading = markdown.match(/^#\s+(.+)$/m);
    return heading ? heading[1].replace(/[*_`]/g, '').trim() : fallback;
  };
  const formatDate = (date) => {
    if (!date) return 'MARKDOWN';
    const d = new Date(`${date}T00:00:00`);
    return Number.isNaN(d.valueOf()) ? 'MARKDOWN' : new Intl.DateTimeFormat('zh-CN', { year: 'numeric', month: '2-digit', day: '2-digit' }).format(d);
  };

  function renderList(filter = '') {
    const query = filter.trim().toLocaleLowerCase();
    const filtered = documents.filter((doc) => `${doc.title} ${doc.path}`.toLocaleLowerCase().includes(query));
    count.textContent = String(documents.length).padStart(2, '0');
    list.replaceChildren();
    if (!filtered.length) {
      const empty = document.createElement('div'); empty.className = 'empty-list'; empty.textContent = documents.length ? '没有匹配的文档' : '还没有 Markdown 文档'; list.append(empty); return;
    }
    const group = document.createElement('div'); group.className = 'doc-group-title'; group.textContent = '全部文档'; list.append(group);
    for (const doc of filtered) {
      const button = document.createElement('button'); button.className = `doc-item${doc.path === activePath ? ' active' : ''}`; button.type = 'button'; button.dataset.path = doc.path;
      const title = document.createElement('span'); title.className = 'doc-title'; title.textContent = doc.title;
      const date = document.createElement('span'); date.className = 'doc-date'; date.textContent = formatDate(doc.date);
      button.append(title, date); button.addEventListener('click', () => openDocument(doc)); list.append(button);
    }
  }

  async function openDocument(doc, updateHash = true) {
    activePath = doc.path; renderList(search.value); closeMenu(); errorState.hidden = true; welcome.hidden = true; articleView.hidden = false;
    tocPanel.hidden = true; toc.replaceChildren(); markdownBody.innerHTML = '<p>正在载入文档…</p>'; currentTitle.textContent = doc.title;
    if (updateHash) history.replaceState(null, '', `#${encodeURIComponent(doc.path)}`);
    try {
      const rawDocumentUrl = repoFileUrl(`documents/${doc.path}`);
      const response = await fetch(rawDocumentUrl);
      if (!response.ok) throw new Error(`文档读取失败（${response.status}）`);
      const source = await response.text();
      currentTitle.textContent = titleFromMarkdown(source, doc.title);
      if (!window.marked || !window.DOMPurify) throw new Error('Markdown 渲染组件加载失败，请检查网络后刷新页面。');
      marked.setOptions({ gfm: true, breaks: false, headerIds: false, mangle: false });
      const rendered = marked.parse(source);
      markdownBody.innerHTML = DOMPurify.sanitize(rendered, { USE_PROFILES: { html: true } });
      markdownBody.querySelectorAll('img[src]').forEach((image) => {
        const src = image.getAttribute('src') || '';
        if (src && !/^(?:[a-z]+:|\/\/|#)/i.test(src)) image.src = new URL(src, rawDocumentUrl).href;
      });
      markdownBody.querySelectorAll('a[href]').forEach((link) => {
        const href = link.getAttribute('href') || '';
        if (!href || /^(?:[a-z]+:|\/\/|#)/i.test(href)) return;
        const resolved = new URL(href, rawDocumentUrl);
        const pathMarker = `/${branch}/`;
        const markerIndex = resolved.pathname.indexOf(pathMarker);
        const repositoryPath = markerIndex >= 0 ? decodeURIComponent(resolved.pathname.slice(markerIndex + pathMarker.length)) : '';
        if (repositoryPath.toLowerCase().endsWith('.md')) {
          const target = documents.find((item) => `documents/${item.path}` === repositoryPath);
          if (target) {
            link.addEventListener('click', (event) => { event.preventDefault(); openDocument(target); });
            return;
          }
        }
        link.href = resolved.href;
        link.target = '_blank';
        link.rel = 'noreferrer';
      });
      const words = source.replace(/```[\s\S]*?```/g, '').trim().split(/\s+/).length;
      document.querySelector('#article-reading-time').textContent = `约 ${Math.max(1, Math.ceil(words / 350))} 分钟阅读`;
      buildToc();
      if (window.MathJax?.typesetPromise) await window.MathJax.typesetPromise([markdownBody]);
    } catch (error) {
      articleView.hidden = true; errorState.hidden = false; errorState.textContent = `${error.message} 请稍后重试。`;
    }
    window.scrollTo({ top: 0, behavior: 'smooth' });
  }

  function buildToc() {
    const headings = [...markdownBody.querySelectorAll('h2, h3')];
    if (headings.length < 2) return;
    for (const [index, heading] of headings.entries()) {
      const id = `section-${index + 1}`; heading.id = id;
      const link = document.createElement('a'); link.href = `#${id}`; link.className = heading.tagName === 'H3' ? 'level-3' : ''; link.textContent = heading.textContent;
      link.addEventListener('click', (event) => { event.preventDefault(); heading.scrollIntoView({ behavior: 'smooth', block: 'start' }); history.replaceState(null, '', `#${encodeURIComponent(activePath)}&section=${id}`); });
      toc.append(link);
    }
    tocPanel.hidden = false;
  }

  search.addEventListener('input', () => renderList(search.value));
  search.addEventListener('keydown', (event) => { if (event.key === 'Escape') { search.value = ''; renderList(); search.blur(); } });
  menuToggle.addEventListener('click', () => { const open = sidebar.classList.toggle('open'); backdrop.classList.toggle('show', open); menuToggle.setAttribute('aria-expanded', String(open)); });
  backdrop.addEventListener('click', closeMenu);
  document.addEventListener('keydown', (event) => { if (event.key === '/' && !['INPUT', 'TEXTAREA'].includes(document.activeElement.tagName)) { event.preventDefault(); search.focus(); } if (event.key === 'Escape') closeMenu(); });

  fetch(treeUrl, {
    headers: {
      Accept: 'application/vnd.github+json',
      'X-GitHub-Api-Version': '2022-11-28',
    },
  })
    .then((response) => { if (!response.ok) throw new Error('文档索引载入失败'); return response.json(); })
    .then((data) => {
      documents = (data.tree || [])
        .filter((item) => item.type === 'blob' && /^documents\/.+\.md$/i.test(item.path))
        .map((item) => {
          const path = item.path.slice('documents/'.length);
          return { path, title: path.split('/').pop().replace(/\.md$/i, ''), date: '' };
        })
        .sort((a, b) => a.path.localeCompare(b.path, 'zh-CN'));
      renderList();
      const hashPath = decodeURIComponent(location.hash.slice(1).split('&')[0]);
      const initial = documents.find((doc) => doc.path === hashPath);
      if (initial) openDocument(initial, false);
    })
    .catch(() => {
      list.innerHTML = '<div class="empty-list">无法读取 GitHub 文档目录。请确认仓库公开，并检查网络连接后刷新。</div>';
    });
})();
