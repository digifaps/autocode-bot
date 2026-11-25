// autocode-bot Foxglove panels (Mapping, Vision). Plain JavaScript, no build step: package with
// scripts/package_foxglove_extension.py (or zip package.json + dist/ + README.md as a .foxe).
"use strict";

const BUTTONS = [
  { label: "Start mapping", service: "/mapping/start", color: "#2e7d32",
    tip: "Start slam_toolbox; the map appears in the 3D view" },
  { label: "Stop mapping", service: "/mapping/stop", color: "#c62828",
    tip: "Stop slam_toolbox (save first if you want to keep the map)" },
  { label: "Save map", service: "/mapping/save", color: "#1565c0",
    tip: "Write the map to ~/maps on the robot" },
];
const LEVEL_COLORS = ["#2e7d32", "#ef6c00", "#c62828", "#757575"]; // OK, WARN, ERROR, STALE

function el(tag, style, text) {
  const e = document.createElement(tag);
  Object.assign(e.style, style || {});
  if (text !== undefined) e.textContent = text;
  return e;
}

function levelOf(v) {
  if (typeof v === "number") return v;
  if (v && typeof v.length === "number" && v.length > 0) return v[0]; // byte decoded as array
  return 3;
}

function initMappingPanel(context) {
  const root = context.panelElement;
  Object.assign(root.style, {
    display: "flex", flexDirection: "column", gap: "8px", padding: "10px",
    fontFamily: "sans-serif", fontSize: "13px", boxSizing: "border-box", height: "100%",
    overflow: "auto",
  });

  const statusRow = el("div", { display: "flex", alignItems: "center", gap: "8px" });
  const dot = el("span", { width: "10px", height: "10px", borderRadius: "50%",
                           background: LEVEL_COLORS[3], flex: "none" });
  const statusText = el("span", {}, "Mapping: waiting for /diagnostics");
  statusRow.append(dot, statusText);

  const row = el("div", { display: "flex", gap: "8px", flexWrap: "wrap" });
  const result = el("div", { minHeight: "1.4em", opacity: "0.9", wordBreak: "break-word" });

  let busy = false;
  const buttons = BUTTONS.map((b) => {
    const btn = el("button", {
      flex: "1 1 110px", padding: "10px 6px", border: "none", borderRadius: "4px",
      background: b.color, color: "#fff", fontSize: "13px", fontWeight: "600", cursor: "pointer",
    }, b.label);
    btn.title = b.tip;
    btn.onclick = () => call(b);
    row.append(btn);
    return btn;
  });

  function setBusy(v) {
    busy = v;
    for (const btn of buttons) {
      btn.disabled = v || !context.callService;
      btn.style.opacity = btn.disabled ? "0.5" : "1";
      btn.style.cursor = btn.disabled ? "default" : "pointer";
    }
  }

  async function call(b) {
    if (busy || !context.callService) return;
    setBusy(true);
    result.style.color = "";
    result.textContent = b.label + "…";
    try {
      const res = await context.callService(b.service, {});
      const ok = !res || res.success !== false;
      result.style.color = ok ? "" : LEVEL_COLORS[2];
      result.textContent = (res && res.message) || (ok ? "done" : "failed");
    } catch (e) {
      result.style.color = LEVEL_COLORS[2];
      result.textContent = b.service + ": " + (e && e.message ? e.message : String(e));
    } finally {
      setBusy(false);
    }
  }

  root.append(statusRow, row, result);
  if (!context.callService) {
    result.textContent = "Service calls need a live connection (Foxglove WebSocket to the robot).";
  }
  setBusy(false);

  context.watch("currentFrame");
  context.watch("colorScheme");
  context.subscribe([{ topic: "/diagnostics" }]);

  context.onRender = (renderState, done) => {
    if (renderState.colorScheme) {
      root.style.color = renderState.colorScheme === "dark" ? "#e0e0e0" : "#212121";
    }
    for (const ev of renderState.currentFrame || []) {
      const statuses = (ev.message && ev.message.status) || [];
      for (const st of statuses) {
        if (st.name === "Mapping") {
          const lvl = levelOf(st.level);
          dot.style.background = LEVEL_COLORS[lvl] || LEVEL_COLORS[3];
          statusText.textContent = "Mapping: " + st.message;
        }
      }
    }
    done();
  };
}

// ---------------------------------------------------------------------------------------------------------
// Vision panel: what the GPU / DLA run (robot_supervisor/vision_manager.py). Switches call the manager's
// std_srvs/SetBool services; the switch positions come from its "Vision" status, the live numbers from the
// "Stereo depth" and "Object detection" statuses on /diagnostics.

const FEATURES = [
  { key: "depth", label: "Stereo depth", service: "/vision/depth/enable", status: "Stereo depth",
    tip: "Depth image and point cloud from the stereo cameras (GPU)" },
  { key: "detection", label: "Object detection", service: "/vision/detection/enable", status: "Object detection",
    tip: "People and objects in the left camera (YOLOX on the DLA); distances need stereo depth" },
];

function initVisionPanel(context) {
  const root = context.panelElement;
  Object.assign(root.style, {
    display: "flex", flexDirection: "column", gap: "8px", padding: "10px",
    fontFamily: "sans-serif", fontSize: "13px", boxSizing: "border-box", height: "100%",
    overflow: "auto",
  });

  let want = null;       // switch positions from the "Vision" status (null until it arrives)
  let busy = false;
  const controls = [];

  function switchButton(onClick) {
    const b = el("button", {
      flex: "none", width: "64px", padding: "6px 0", border: "none", borderRadius: "14px",
      color: "#fff", fontSize: "12px", fontWeight: "600", cursor: "pointer", background: LEVEL_COLORS[3],
    }, "…");
    b.onclick = onClick;
    controls.push(b);
    return b;
  }

  const rows = {};
  for (const f of FEATURES) {
    const row = el("div", { display: "flex", alignItems: "center", gap: "8px" });
    const dot = el("span", { width: "10px", height: "10px", borderRadius: "50%", background: LEVEL_COLORS[3], flex: "none" });
    const text = el("div", { flex: "1 1 auto", minWidth: "0" });
    const name = el("div", { fontWeight: "600" }, f.label);
    const status = el("div", { opacity: "0.8", fontSize: "12px", wordBreak: "break-word" }, "waiting for /diagnostics");
    text.append(name, status);
    text.title = f.tip;
    const sw = switchButton(() => want && call(f.service, !want[f.key], f.label + (want[f.key] ? " off" : " on")));
    row.append(dot, text, sw);
    root.append(row);
    rows[f.key] = { dot, status, sw };
  }

  // Detection model: Fast (yolox_tiny) / Accurate (yolox_s)
  const modelRow = el("div", { display: "flex", alignItems: "center", gap: "6px", paddingLeft: "18px" });
  const modelLabel = el("span", { fontSize: "12px", opacity: "0.8", flex: "1 1 auto" }, "Detection model");
  const fast = el("button", {}, "Fast");
  const accurate = el("button", {}, "Accurate");
  fast.title = "yolox_tiny: ~57 per second on the DLA";
  accurate.title = "yolox_s: more accurate, especially for small or far objects; ~25 per second on the DLA";
  for (const b of [fast, accurate]) {
    Object.assign(b.style, { padding: "4px 10px", border: "1px solid #888", borderRadius: "4px",
                             background: "transparent", color: "inherit", fontSize: "12px", cursor: "pointer" });
    controls.push(b);
  }
  fast.onclick = () => want && want.accurate && call("/vision/detection/accurate", false, "Fast model");
  accurate.onclick = () => want && !want.accurate && call("/vision/detection/accurate", true, "Accurate model");
  modelRow.append(modelLabel, fast, accurate);
  root.append(modelRow);

  const footer = el("div", { fontSize: "12px", opacity: "0.8", borderTop: "1px solid rgba(128,128,128,0.4)", paddingTop: "6px" },
                    "GPU: – · memory: –");
  const result = el("div", { minHeight: "1.4em", fontSize: "12px", wordBreak: "break-word" });
  root.append(footer, result);

  function refresh() {
    const ready = want !== null && !busy && !!context.callService;
    for (const b of controls) {
      b.disabled = !ready;
      b.style.opacity = ready ? "1" : "0.5";
      b.style.cursor = ready ? "pointer" : "default";
    }
    if (!want) return;
    for (const f of FEATURES) {
      const sw = rows[f.key].sw;
      sw.textContent = want[f.key] ? "ON" : "OFF";
      sw.style.background = want[f.key] ? LEVEL_COLORS[0] : "#616161";
    }
    const sel = { fontWeight: "700", background: "#1565c0", color: "#fff", borderColor: "#1565c0" };
    const unsel = { fontWeight: "400", background: "transparent", color: "inherit", borderColor: "#888" };
    Object.assign(fast.style, want.accurate ? unsel : sel);
    Object.assign(accurate.style, want.accurate ? sel : unsel);
  }

  async function call(service, data, label) {
    if (busy || !context.callService) return;
    busy = true;
    refresh();
    result.style.color = "";
    result.textContent = label + "… (loading a model takes a few seconds)";
    try {
      const res = await context.callService(service, { data });
      const ok = !res || res.success !== false;
      result.style.color = ok ? "" : LEVEL_COLORS[2];
      result.textContent = (res && res.message) || (ok ? "done" : "failed");
    } catch (e) {
      result.style.color = LEVEL_COLORS[2];
      result.textContent = service + ": " + (e && e.message ? e.message : String(e));
    } finally {
      busy = false;
      refresh();
    }
  }

  if (!context.callService) {
    result.textContent = "Switching needs a live connection (Foxglove WebSocket to the robot).";
  }
  refresh();

  context.watch("currentFrame");
  context.watch("colorScheme");
  context.subscribe([{ topic: "/diagnostics" }]);

  context.onRender = (renderState, done) => {
    if (renderState.colorScheme) {
      root.style.color = renderState.colorScheme === "dark" ? "#e0e0e0" : "#212121";
    }
    for (const ev of renderState.currentFrame || []) {
      const statuses = (ev.message && ev.message.status) || [];
      for (const st of statuses) {
        const lvl = levelOf(st.level);
        const f = FEATURES.find((x) => x.status === st.name);
        if (f) {
          rows[f.key].dot.style.background = LEVEL_COLORS[lvl] || LEVEL_COLORS[3];
          rows[f.key].status.textContent = st.message;
        } else if (st.name === "Vision") {
          const v = {};
          for (const kv of st.values || []) v[kv.key] = kv.value;
          want = { depth: v.depth === "on", detection: v.detection === "on", accurate: v.accurate === "true" };
          const gpu = v.gpu_load_percent && v.gpu_load_percent !== "-" ? v.gpu_load_percent + " %" : "–";
          const mem = v.memory_used_mb ? (v.memory_used_mb / 1024).toFixed(1) + " / " + (v.memory_total_mb / 1024).toFixed(1) + " GB" : "–";
          footer.textContent = "GPU load: " + gpu + " · memory: " + mem + (lvl > 0 ? " · " + st.message : "");
          footer.style.color = lvl > 0 ? LEVEL_COLORS[lvl] : "";
          refresh();
        }
      }
    }
    done();
  };
}

module.exports.activate = function activate(extensionContext) {
  extensionContext.registerPanel({ name: "Mapping", initPanel: initMappingPanel });
  extensionContext.registerPanel({ name: "Vision", initPanel: initVisionPanel });
};
