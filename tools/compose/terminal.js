// The run terminal: everything a run prints, raw and as it comes, the raw SITL
// and mission logs, and MAVProxy's MAV> prompt. Its colours are its own
// (presets, or any colours you pick), remembered in this browser.
"use strict";

const Terminal = (() => {
  const SCHEMES = {
    midnight: { name: "Midnight", bg: "#0b0f17", fg: "#c8d3e6", cmd: "#7ee787", sys: "#79c0ff", dim: "#5d6b82" },
    matrix: { name: "Matrix", bg: "#020a04", fg: "#39ff7a", cmd: "#e6ffe9", sys: "#1fae4f", dim: "#1c6a35" },
    amber: { name: "Amber", bg: "#140d02", fg: "#ffb43b", cmd: "#fff1cf", sys: "#d98a14", dim: "#8a5a12" },
    solarized: { name: "Solarized", bg: "#002b36", fg: "#93a1a1", cmd: "#b58900", sys: "#268bd2", dim: "#586e75" },
    dracula: { name: "Dracula", bg: "#1e1f29", fg: "#f8f8f2", cmd: "#50fa7b", sys: "#bd93f9", dim: "#6272a4" },
    paper: { name: "Paper", bg: "#fbf8ef", fg: "#2b2b2b", cmd: "#0b6e4f", sys: "#2357c6", dim: "#8c8a80" },
  };
  const ROLES = [["bg", "Background"], ["fg", "Output"], ["cmd", "Your commands"], ["sys", "Notes"], ["dim", "Prompt hint"]];
  const KEY = "compose-terminal";
  const MAX_SPANS = 6000;
  const ANSI = /\x1b\[[0-9;?]*[A-Za-z]|\x1b\][^\x07]*\x07/g;

  let root, screens = {}, tabs, form, input, note, quick, onSend = () => {}, active = "out";
  let scheme = { preset: "midnight", custom: { ...SCHEMES.midnight }, size: 12.5, height: 0, collapsed: false };

  function el(tag, attrs = {}, ...kids) {
    const e = document.createElement(tag);
    for (const [k, v] of Object.entries(attrs)) {
      if (k === "class") e.className = v;
      else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
      else if (v !== false && v != null) e.setAttribute(k, v === true ? "" : v);
    }
    for (const k of kids.flat()) if (k != null) e.append(k);
    return e;
  }

  // --- colours -----------------------------------------------------------------
  function load() {
    try { Object.assign(scheme, JSON.parse(localStorage.getItem(KEY) || "{}")); } catch (e) { /* none */ }
    const asked = new URLSearchParams(location.search).get("terminal");
    if (asked in SCHEMES) scheme.preset = asked;
  }
  function save() { try { localStorage.setItem(KEY, JSON.stringify(scheme)); } catch (e) { /* none */ } }
  function colours() { return scheme.preset === "custom" ? scheme.custom : SCHEMES[scheme.preset] || SCHEMES.midnight; }
  function paint() {
    const c = colours();
    for (const [role] of ROLES) root.style.setProperty(`--t-${role}`, c[role]);
    root.style.setProperty("--t-size", `${scheme.size}px`);
    // Docked at the bottom of the window: its height, or only its tab bar.
    const h = scheme.height ? `${scheme.height}px` : "42vh";
    document.body.style.setProperty("--t-height", h);
    root.style.setProperty("--t-height", h);
    root.classList.toggle("collapsed", scheme.collapsed);
    document.body.classList.toggle("collapsed", scheme.collapsed);
    const fold = root.querySelector(".t-fold");
    fold.textContent = scheme.collapsed ? "▴ Show" : "▾ Hide";
    fold.setAttribute("aria-expanded", String(!scheme.collapsed));
    const custom = root.querySelector(".t-custom");
    custom.hidden = scheme.preset !== "custom";
    for (const [role] of ROLES) custom.querySelector(`[data-role="${role}"]`).value = scheme.custom[role];
    root.querySelector(".t-scheme").value = scheme.preset;
  }

  // --- the screens ---------------------------------------------------------------
  function write(kind, text, screen = "out") {
    const s = screens[screen];
    const atEnd = s.scrollTop + s.clientHeight >= s.scrollHeight - 12;
    text = text.replace(ANSI, "").replace(/\r\n/g, "\n").replace(/\r/g, "\n");
    if (!text) return;
    s.append(el("span", { class: `t-${kind}` }, text));
    while (s.childElementCount > MAX_SPANS) s.firstElementChild.remove();
    if (atEnd || kind === "cmd") s.scrollTop = s.scrollHeight;
  }
  function clear(screen) {
    for (const k of screen ? [screen] : Object.keys(screens)) screens[k].replaceChildren();
  }
  function show(which) {
    active = which;
    for (const [k, s] of Object.entries(screens)) s.hidden = k !== which;
    for (const b of tabs.querySelectorAll("button")) b.setAttribute("aria-selected", String(b.dataset.tab === which));
    screens[which].scrollTop = screens[which].scrollHeight;
  }
  function files(names) {
    for (const b of tabs.querySelectorAll("button[data-tab]"))
      if (b.dataset.tab !== "out") b.disabled = !names.includes(b.dataset.tab);
  }

  // --- the prompt ------------------------------------------------------------------
  const history = [];
  let historyAt = 0;
  function prompt(on, commands = []) {
    form.hidden = quick.hidden = !on;
    quick.replaceChildren(...commands.map(([label, line]) =>
      el("button", { type: "button", title: line, onclick: () => submit(line) }, label)));
    if (on) note.textContent = "";
  }
  async function submit(line) {
    line = line.trim();
    if (!line) return;
    if (history[history.length - 1] !== line) history.push(line);
    historyAt = history.length;
    show("out");
    note.textContent = `sending: ${line}`;
    note.className = "t-note";
    const r = await onSend(line);
    note.textContent = r.ok ? `sent ✓ ${line}` : `not sent: ${r.message}`;
    note.className = "t-note " + (r.ok ? "ok" : "bad");
  }

  function mount(container, send) {
    onSend = send;
    load();
    root = container;
    root.classList.add("term");
    tabs = el("div", { class: "t-tabs", role: "tablist" },
      el("span", { class: "t-title" }, "Terminal"),
      [["out", "Output"], ["sitl", "SITL log"], ["mission", "Mission log"]].map(([k, label]) =>
        el("button", { type: "button", role: "tab", "data-tab": k, disabled: k !== "out",
                       onclick: () => show(k) }, label)),
      el("span", { class: "t-gap" }),
      el("select", { class: "t-scheme", "aria-label": "Terminal colours",
        onchange: (e) => { scheme.preset = e.target.value; save(); paint(); } },
        Object.entries(SCHEMES).map(([k, v]) => el("option", { value: k }, v.name)),
        el("option", { value: "custom" }, "Custom…")),
      el("button", { type: "button", class: "t-small", title: "Smaller text",
        onclick: () => { scheme.size = Math.max(9, scheme.size - 1); save(); paint(); } }, "A−"),
      el("button", { type: "button", class: "t-small", title: "Larger text",
        onclick: () => { scheme.size = Math.min(20, scheme.size + 1); save(); paint(); } }, "A+"),
      el("button", { type: "button", class: "t-small", title: "Clear this tab",
        onclick: () => clear(active) }, "Clear"),
      el("button", { type: "button", class: "t-small t-fold",
        onclick: () => { scheme.collapsed = !scheme.collapsed; save(); paint(); } }, "▾ Hide"),
      el("button", { type: "button", class: "t-small", title: "Close the terminal (the run goes on)",
        onclick: () => dock(false) }, "✕"));
    // Drag the top edge to resize.
    const grip = el("div", { class: "t-grip", title: "Drag to resize" });
    grip.addEventListener("pointerdown", (e) => {
      grip.setPointerCapture(e.pointerId);
      const move = (m) => { scheme.height = Math.max(120, Math.min(innerHeight - 80, innerHeight - m.clientY));
                            scheme.collapsed = false; paint(); };
      const up = () => { grip.removeEventListener("pointermove", move); save(); };
      grip.addEventListener("pointermove", move);
      grip.addEventListener("pointerup", up, { once: true });
    });
    const custom = el("div", { class: "t-custom", hidden: true },
      ROLES.map(([role, label]) => el("label", {}, el("input", { type: "color", "data-role": role,
        oninput: (e) => { scheme.custom[role] = e.target.value; save(); paint(); } }), label)),
      el("button", { type: "button", class: "t-small",
        onclick: () => { scheme.custom = { ...(SCHEMES[Object.keys(SCHEMES)[0]]) }; save(); paint(); } }, "Reset"));
    for (const k of ["out", "sitl", "mission"])
      screens[k] = el("pre", { class: "t-screen", hidden: k !== "out", tabindex: "0",
                                "aria-label": `${k} log`, "aria-live": k === "out" ? "polite" : "off" });
    input = el("input", { type: "text", autocomplete: "off", spellcheck: "false", "aria-label": "MAVProxy command",
                          placeholder: "type a MAVProxy command: arm throttle, mode guided, takeoff 10 …" });
    input.addEventListener("keydown", (e) => {
      if (e.key !== "ArrowUp" && e.key !== "ArrowDown") return;
      e.preventDefault();
      historyAt = Math.max(0, Math.min(history.length, historyAt + (e.key === "ArrowUp" ? -1 : 1)));
      input.value = history[historyAt] ?? "";
    });
    form = el("form", { class: "t-prompt", hidden: true,
      onsubmit: (e) => { e.preventDefault(); const v = input.value; input.value = ""; submit(v); } },
      el("span", { class: "t-ps" }, "MAV>"), input);
    quick = el("div", { class: "t-quick", hidden: true });
    note = el("div", { class: "t-note", "aria-live": "polite" });
    root.replaceChildren(grip, tabs, custom, ...Object.values(screens), form, quick, note);
    paint();
  }

  // Open (docked at the bottom) or closed; opening also unfolds it.
  function dock(on) {
    root.hidden = !on;
    document.body.classList.toggle("docked", on);
    if (on && scheme.collapsed) { scheme.collapsed = false; save(); paint(); }
  }

  return { mount, write, clear, show, files, prompt, dock, get tab() { return active; },
           focus: () => input && !form.hidden && input.focus() };
})();
