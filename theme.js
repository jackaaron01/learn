(() => {
  const themeButtons = [...document.querySelectorAll('[data-theme-choice]')];
  const modeToggle = document.querySelector('#mode-toggle');
  const modeLabel = document.querySelector('#mode-label');
  const themes = ['warm', 'ocean', 'apricot', 'lavender'];
  const legacyThemes = { forest: 'warm', clay: 'apricot', violet: 'lavender' };

  function readPreference(key, fallback) {
    try { return localStorage.getItem(key) || fallback; } catch (_) { return fallback; }
  }

  function setTheme(theme) {
    const migrated = legacyThemes[theme] || theme;
    const selected = themes.includes(migrated) ? migrated : 'warm';
    document.body.dataset.theme = selected;
    themeButtons.forEach((button) => {
      button.setAttribute('aria-pressed', String(button.dataset.themeChoice === selected));
    });
    try { localStorage.setItem('learn-reader-theme', selected); } catch (_) { /* Keep the selected color for this page view. */ }
  }

  function setMode(mode) {
    const selected = mode === 'night' ? 'night' : 'day';
    const isNight = selected === 'night';
    document.body.dataset.mode = selected;
    if (modeToggle) {
      const label = isNight ? '日间' : '夜间';
      modeToggle.setAttribute('aria-label', `切换到${label}模式`);
      modeToggle.title = `切换到${label}模式`;
    }
    if (modeLabel) modeLabel.textContent = isNight ? '日间' : '夜间';
    try { localStorage.setItem('learn-reader-mode', selected); } catch (_) { /* Keep the selected mode for this page view. */ }
  }

  setTheme(readPreference('learn-reader-theme', 'warm'));
  setMode(readPreference('learn-reader-mode', 'day'));
  themeButtons.forEach((button) => button.addEventListener('click', () => setTheme(button.dataset.themeChoice)));
  modeToggle?.addEventListener('click', () => {
    setMode(document.body.dataset.mode === 'night' ? 'day' : 'night');
  });
})();
