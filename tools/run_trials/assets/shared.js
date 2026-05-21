/* QR Nav — Trial Runner shared module.
 *
 * Public surface lives on window.QRTrials. Each page calls into it; no
 * framework, no bundler. State persists to a single localStorage key so
 * navigating between index.html and the per-trial pages round-trips data.
 */
(function () {
  'use strict';

  const STATE_KEY = 'qr_nav_run_trials_state_v1';

  // ---------------------------------------------------------------------------
  // Schema definitions — single source of truth for trial structure.
  // ---------------------------------------------------------------------------

  const COMMANDS = [
    'TURN_LEFT',
    'TURN_RIGHT',
    'STOP',
    'GO',
    'SPEED_UP',
    'SPEED_DOWN',
    'U_TURN',
  ];
  const DISTANCES = ['0.3', '0.6', '1.0', '1.5'];
  const ANGLES = ['0', '15', '30', '45'];
  const LIGHTING = ['bright', 'normal', 'dim'];
  const T6_CONFIGS = ['baseline', 'filter_off'];

  const TRIAL_DEFS = {
    t1: { reps: 10, rows: COMMANDS, total: 70 },
    t2: { reps: 10, rows: DISTANCES, total: 40 },
    t3: { reps: 10, rows: ANGLES, total: 40 },
    t4: { reps: 10, rows: LIGHTING, total: 30 },
    t5: { reps: 10, total: 10 },
    t6: { reps: 15, rows: T6_CONFIGS, total: 30 },
  };

  const PRE_CHECKLIST_KEYS = [
    'colcon_build',
    'topic_hz',
    'topic_info_cmd_vel',
    'data_logger_writes',
    'results_dir',
  ];

  const POST_CHECKLIST_KEYS = [
    'six_csvs_present',
    'experiment_log_entry',
    'photos_saved',
    'csv_spot_check',
  ];

  // ---------------------------------------------------------------------------
  // Defaults — used both for first load and after Reset.
  // ---------------------------------------------------------------------------

  // Row factories -------------------------------------------------------------
  // Trials with per-row notes (T1, T4) wrap the rep array under {notes, reps}
  // so the UI can bind a single notes textarea per row. The CSV writer
  // expands the row note onto every rep at export time, which keeps the
  // analyze.ipynb schema unchanged.

  function makeT1Row() {
    return {
      notes: '',
      reps: Array.from({ length: 10 }, () => ({
        detected: null,
        executed: null,
        latency_ms: null,
      })),
    };
  }

  function makeT2Row() {
    return Array.from({ length: 10 }, () => ({
      detected: null,
      bbox_area: null,
    }));
  }

  function makeT3Row() {
    return Array.from({ length: 10 }, () => ({
      detected: null,
      decode_stage: '',
    }));
  }

  function makeT4Row() {
    return {
      notes: '',
      reps: Array.from({ length: 10 }, () => ({
        detected: null,
      })),
    };
  }

  function makeT5Rows() {
    // closer/further default to the typical pair (STOP closer, GO further)
    // for fast data entry, but acted_on is left blank so a row only counts
    // toward progress once the operator records what the robot actually
    // did. Otherwise progress would read "10/10 done" before any rep ran.
    return Array.from({ length: 10 }, () => ({
      closer: 'STOP',
      further: 'GO',
      acted_on: '',
      notes: '',
    }));
  }

  function makeT6Row() {
    return Array.from({ length: 15 }, () => ({
      tp: null,
      fp: null,
    }));
  }

  function defaultTrial(id) {
    if (id === 't1') {
      const rows = {};
      for (const cmd of COMMANDS) rows[cmd] = makeT1Row();
      return { rows };
    }
    if (id === 't2') {
      const rows = {};
      for (const d of DISTANCES) rows[d] = makeT2Row();
      return { rows };
    }
    if (id === 't3') {
      const rows = {};
      for (const a of ANGLES) rows[a] = makeT3Row();
      return { rows };
    }
    if (id === 't4') {
      const rows = {};
      for (const l of LIGHTING) rows[l] = makeT4Row();
      return { rows };
    }
    if (id === 't5') {
      return { rows: makeT5Rows() };
    }
    if (id === 't6') {
      const rows = {};
      for (const c of T6_CONFIGS) rows[c] = makeT6Row();
      return { rows };
    }
    return null;
  }

  function defaultState() {
    const pre = {};
    for (const k of PRE_CHECKLIST_KEYS) pre[k] = false;
    const post = {};
    for (const k of POST_CHECKLIST_KEYS) post[k] = false;
    return {
      preChecklist: pre,
      t1: defaultTrial('t1'),
      t2: defaultTrial('t2'),
      t3: defaultTrial('t3'),
      t4: defaultTrial('t4'),
      t5: defaultTrial('t5'),
      t6: defaultTrial('t6'),
      postChecklist: post,
    };
  }

  // ---------------------------------------------------------------------------
  // Persistence — load/save with a debounced writer.
  // ---------------------------------------------------------------------------

  function load() {
    try {
      const raw = localStorage.getItem(STATE_KEY);
      if (!raw) return defaultState();
      const parsed = JSON.parse(raw);
      // Shallow-merge against defaults so a stale schema does not break the UI.
      const base = defaultState();
      return {
        preChecklist: { ...base.preChecklist, ...(parsed.preChecklist || {}) },
        t1: parsed.t1 || base.t1,
        t2: parsed.t2 || base.t2,
        t3: parsed.t3 || base.t3,
        t4: parsed.t4 || base.t4,
        t5: parsed.t5 || base.t5,
        t6: parsed.t6 || base.t6,
        postChecklist: {
          ...base.postChecklist,
          ...(parsed.postChecklist || {}),
        },
      };
    } catch (err) {
      console.warn('QRTrials.load: falling back to defaults —', err);
      return defaultState();
    }
  }

  let saveTimer = null;
  function save(state) {
    if (saveTimer) clearTimeout(saveTimer);
    saveTimer = setTimeout(() => {
      try {
        localStorage.setItem(STATE_KEY, JSON.stringify(state));
      } catch (err) {
        console.error('QRTrials.save:', err);
      }
    }, 200);
  }

  function saveNow(state) {
    if (saveTimer) clearTimeout(saveTimer);
    saveTimer = null;
    localStorage.setItem(STATE_KEY, JSON.stringify(state));
  }

  function resetTrial(id) {
    const state = load();
    state[id] = defaultTrial(id);
    saveNow(state);
    return state;
  }

  function resetAll() {
    localStorage.removeItem(STATE_KEY);
    return defaultState();
  }

  // ---------------------------------------------------------------------------
  // Counts — drive the dashboard and per-page badges.
  // ---------------------------------------------------------------------------

  function isCellRecorded(id, cell) {
    if (id === 't1') return cell.detected !== null;
    if (id === 't2') return cell.detected !== null;
    if (id === 't3') return cell.detected !== null;
    if (id === 't4') return cell.detected !== null;
    if (id === 't5') return !!cell.acted_on;
    if (id === 't6') return cell.tp !== null || cell.fp !== null;
    return false;
  }

  // Returns the rep array for a given trial+row regardless of whether the
  // schema wraps reps under {notes, reps} (T1/T4) or stores them flat.
  function rowReps(id, rowValue) {
    if (id === 't1' || id === 't4') return rowValue.reps || [];
    return rowValue || [];
  }

  function countDone(state, id) {
    const def = TRIAL_DEFS[id];
    let done = 0;
    if (id === 't5') {
      for (const cell of state.t5.rows) {
        if (isCellRecorded('t5', cell)) done += 1;
      }
    } else {
      const trial = state[id];
      for (const key of Object.keys(trial.rows)) {
        const reps = rowReps(id, trial.rows[key]);
        for (const cell of reps) {
          if (isCellRecorded(id, cell)) done += 1;
        }
      }
    }
    return { done, total: def.total };
  }

  function countAll(state) {
    let done = 0;
    let total = 0;
    for (const id of ['t1', 't2', 't3', 't4', 't5', 't6']) {
      const c = countDone(state, id);
      done += c.done;
      total += c.total;
    }
    return { done, total };
  }

  // ---------------------------------------------------------------------------
  // CSV builders — schemas must match results/analyze.ipynb.
  // Booleans serialise as 1/0 so pandas can sum them directly.
  // ---------------------------------------------------------------------------

  function csvEscape(v) {
    if (v === null || v === undefined) return '';
    const s = String(v);
    if (s.includes(',') || s.includes('"') || s.includes('\n')) {
      return `"${s.replace(/"/g, '""')}"`;
    }
    return s;
  }

  function rowsToCsv(header, rows) {
    const lines = [header.join(',')];
    for (const row of rows) {
      lines.push(row.map(csvEscape).join(','));
    }
    return lines.join('\n') + '\n';
  }

  function boolCell(v) {
    if (v === 1 || v === true) return 1;
    if (v === 0 || v === false) return 0;
    return '';
  }

  function buildCsv(state, id) {
    if (id === 't1') {
      const header = [
        'rep',
        'command',
        'detected',
        'executed_correctly',
        'latency_ms',
        'notes',
      ];
      const rows = [];
      for (const cmd of COMMANDS) {
        const row = state.t1.rows[cmd] || { notes: '', reps: [] };
        const reps = row.reps || [];
        reps.forEach((cell, i) => {
          rows.push([
            i + 1,
            cmd,
            boolCell(cell.detected),
            boolCell(cell.executed),
            cell.latency_ms ?? '',
            row.notes || '',
          ]);
        });
      }
      return { filename: 't1_per_command.csv', contents: rowsToCsv(header, rows) };
    }

    if (id === 't2') {
      const header = ['rep', 'distance_m', 'detected', 'bbox_area_px2'];
      const rows = [];
      for (const d of DISTANCES) {
        const reps = state.t2.rows[d] || [];
        reps.forEach((cell, i) => {
          rows.push([i + 1, d, boolCell(cell.detected), cell.bbox_area ?? '']);
        });
      }
      return { filename: 't2_distance.csv', contents: rowsToCsv(header, rows) };
    }

    if (id === 't3') {
      const header = ['rep', 'angle_deg', 'detected', 'decode_stage'];
      const rows = [];
      for (const a of ANGLES) {
        const reps = state.t3.rows[a] || [];
        reps.forEach((cell, i) => {
          rows.push([i + 1, a, boolCell(cell.detected), cell.decode_stage || '']);
        });
      }
      return { filename: 't3_angle.csv', contents: rowsToCsv(header, rows) };
    }

    if (id === 't4') {
      const header = ['rep', 'condition', 'detected', 'notes'];
      const rows = [];
      for (const l of LIGHTING) {
        const row = state.t4.rows[l] || { notes: '', reps: [] };
        const reps = row.reps || [];
        reps.forEach((cell, i) => {
          rows.push([i + 1, l, boolCell(cell.detected), row.notes || '']);
        });
      }
      return { filename: 't4_lighting.csv', contents: rowsToCsv(header, rows) };
    }

    if (id === 't5') {
      const header = [
        'rep',
        'closer_card',
        'further_card',
        'acted_on',
        'correct',
        'notes',
      ];
      const rows = [];
      state.t5.rows.forEach((cell, i) => {
        const correct = cell.acted_on && cell.acted_on === cell.closer ? 1 : 0;
        rows.push([
          i + 1,
          cell.closer || '',
          cell.further || '',
          cell.acted_on || '',
          cell.acted_on ? correct : '',
          cell.notes || '',
        ]);
      });
      return { filename: 't5_simultaneous.csv', contents: rowsToCsv(header, rows) };
    }

    if (id === 't6') {
      const header = ['rep', 'configuration', 'true_positive', 'false_positive'];
      const rows = [];
      for (const cfg of T6_CONFIGS) {
        const reps = state.t6.rows[cfg] || [];
        reps.forEach((cell, i) => {
          rows.push([i + 1, cfg, boolCell(cell.tp), boolCell(cell.fp)]);
        });
      }
      return { filename: 't6_ablation.csv', contents: rowsToCsv(header, rows) };
    }

    return null;
  }

  function downloadBlob(filename, contents) {
    const blob = new Blob([contents], { type: 'text/csv;charset=utf-8' });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    setTimeout(() => {
      document.body.removeChild(a);
      URL.revokeObjectURL(url);
    }, 100);
  }

  function downloadCsv(state, id) {
    const out = buildCsv(state, id);
    if (out) downloadBlob(out.filename, out.contents);
  }

  async function downloadAll(state) {
    // Sequential with a small delay so browsers do not coalesce the
    // downloads into a single popup-blocker prompt.
    for (const id of ['t1', 't2', 't3', 't4', 't5', 't6']) {
      const out = buildCsv(state, id);
      if (out) downloadBlob(out.filename, out.contents);
      await new Promise((r) => setTimeout(r, 250));
    }
  }

  // ---------------------------------------------------------------------------
  // Latency stopwatch — used by the T1 page only.
  // ---------------------------------------------------------------------------

  const latency = {
    _t0: null,
    _t1: null,

    start() {
      this._t0 = performance.now();
      this._t1 = null;
    },
    stop() {
      if (this._t0 === null) return null;
      this._t1 = performance.now();
      return Math.round(this._t1 - this._t0);
    },
    reset() {
      this._t0 = null;
      this._t1 = null;
    },
    delta() {
      if (this._t0 === null || this._t1 === null) return null;
      return Math.round(this._t1 - this._t0);
    },
  };

  // ---------------------------------------------------------------------------
  // Misc helpers.
  // ---------------------------------------------------------------------------

  async function copyToClipboard(text) {
    try {
      await navigator.clipboard.writeText(text);
      return true;
    } catch (err) {
      // Fallback for non-secure contexts (file:// counts as secure on Chrome,
      // but Safari sometimes balks).
      const ta = document.createElement('textarea');
      ta.value = text;
      ta.style.position = 'fixed';
      ta.style.opacity = '0';
      document.body.appendChild(ta);
      ta.select();
      try {
        document.execCommand('copy');
        return true;
      } finally {
        document.body.removeChild(ta);
      }
    }
  }

  // ---------------------------------------------------------------------------
  // Trial-page chrome — shared header buttons + copy delegation. Pages call
  // QRTrials.installTrialChrome('t1', {...}) once and provide an onChange
  // callback so the page can re-render its grid after resets.
  // ---------------------------------------------------------------------------

  function installTrialChrome(trialId, opts) {
    const { onReset } = opts || {};

    function refreshProgress() {
      const state = load();
      const { done, total } = countDone(state, trialId);
      const text = document.getElementById('trial-progress-text');
      const bar = document.getElementById('trial-progress-bar');
      if (text) text.textContent = `${done} / ${total}`;
      if (bar) bar.style.width = `${total > 0 ? (done / total) * 100 : 0}%`;
    }

    // Reset button
    const resetBtn = document.getElementById('reset-trial-btn');
    if (resetBtn) {
      resetBtn.addEventListener('click', () => {
        if (!confirm(`Reset all data for ${trialId.toUpperCase()}? This cannot be undone.`)) return;
        const newState = resetTrial(trialId);
        if (typeof onReset === 'function') onReset(newState);
        refreshProgress();
      });
    }

    // Download button
    const dlBtn = document.getElementById('download-csv-btn');
    if (dlBtn) {
      dlBtn.addEventListener('click', () => {
        const state = load();
        downloadCsv(state, trialId);
      });
    }

    // Copy buttons (delegated)
    document.body.addEventListener('click', async (e) => {
      const btn = e.target.closest('.copy-btn');
      if (!btn) return;
      e.preventDefault();
      const targetId = btn.dataset.target;
      const text = targetId
        ? document.getElementById(targetId).textContent
        : btn.dataset.cmd || '';
      await copyToClipboard(text);
      flashCopied(btn);
    });

    refreshProgress();

    return { refreshProgress };
  }

  function flashCopied(btn, label = 'Copied') {
    const prev = btn.textContent;
    btn.textContent = label;
    btn.classList.add('!bg-emerald-600', 'text-white');
    setTimeout(() => {
      btn.textContent = prev;
      btn.classList.remove('!bg-emerald-600', 'text-white');
    }, 900);
  }

  // ---------------------------------------------------------------------------
  // Public surface.
  // ---------------------------------------------------------------------------

  window.QRTrials = {
    STATE_KEY,
    COMMANDS,
    DISTANCES,
    ANGLES,
    LIGHTING,
    T6_CONFIGS,
    TRIAL_DEFS,
    PRE_CHECKLIST_KEYS,
    POST_CHECKLIST_KEYS,
    defaultState,
    load,
    save,
    saveNow,
    resetTrial,
    resetAll,
    countDone,
    countAll,
    rowReps,
    buildCsv,
    downloadCsv,
    downloadAll,
    latency,
    copyToClipboard,
    flashCopied,
    installTrialChrome,
  };
})();
