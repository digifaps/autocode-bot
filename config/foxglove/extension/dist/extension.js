// autocode-bot Foxglove panels. Plain JavaScript, no build step: package with
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

module.exports.activate = function activate(extensionContext) {
  extensionContext.registerPanel({ name: "Mapping", initPanel: initMappingPanel });
};
