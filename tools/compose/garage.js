// The garage: an abstract drawing of the vehicle above the part cards. Picking a
// part (its card, one of its pickers, or the part in the drawing) moves the
// camera onto it and lights it; the rest dims. The drawing follows the picks:
// propeller diameter, battery cells and size, motor stator size.
"use strict";

const Garage = (() => {
  const NS = "http://www.w3.org/2000/svg";
  const VIEW = [0, 0, 800, 420];
  let svg, caption, chips, shelf, onPick = () => {}, onChoose = () => {}, view = VIEW.slice(), anim = 0;
  let focusKey = "overview", info = null;

  // --- what the picks say about the drawing ----------------------------------
  const num = (re, s, d) => { const m = re.exec(s || ""); return m ? parseFloat(m[1]) : d; };
  const propRadius = (name) => 6 + 5.2 * num(/(\d+(?:\.\d+)?)x/i, name, 11);
  const motorRadius = (name) => 7 + 0.3 * num(/(\d{2})\d{2}/, name, 28);
  const cells = (name) => num(/(\d)S\b/, name, 4);
  const mah = (name) => num(/(\d{3,5})\s*mAh/i, name, 4000);

  // --- drawing helpers --------------------------------------------------------
  // a(focus, extra attrs) tags an element: data-focus is what lights it,
  // data-anchor the instance the camera frames when there are several.
  const a = (focus, anchor = false) => `data-focus="${focus}"${anchor ? ' data-anchor=""' : ""}`;
  const chip = (x, y, w, h, focus, label, anchor = true) =>
    `<g ${a(focus, anchor)}><rect class="g-chip" x="${x}" y="${y}" width="${w}" height="${h}" rx="2"/>` +
    `<text class="g-tiny" x="${x + w / 2}" y="${y + h + 9}">${label}</text></g>`;
  const puck = (x, y, r, focusG, focusM) =>
    `<g ${a(focusG, true)}><circle class="g-puck" cx="${x}" cy="${y}" r="${r}"/>` +
    `<path class="g-mark" d="M${x} ${y - r * 0.55} l${r * 0.32} ${r * 0.8} l${-r * 0.32} ${-r * 0.22} l${-r * 0.32} ${r * 0.22} z"/>` +
    `<text class="g-tiny" x="${x}" y="${y + r + 11}">GNSS</text></g>` +
    chip(x + r + 6, y - 6, 12, 12, focusM, "MAG");
  const fc = (x, y, w, h) =>
    `<g class="g-neutral"><rect class="g-board" x="${x}" y="${y}" width="${w}" height="${h}" rx="4"/>` +
    `<text class="g-tiny" x="${x + w / 2}" y="${y + h - 5}">FC</text></g>` +
    chip(x + 8, y + 8, 15, 15, "imu", "IMU") + chip(x + w - 22, y + 9, 13, 11, "barometer", "BARO");

  // --- the airframes ------------------------------------------------------------
  function quad(p) {
    const r = propRadius(p.propeller), mr = motorRadius(p.motor);
    const bw = 70 + 50 * Math.min(1, mah(p.battery) / 10000), n = cells(p.battery);
    const cx = 400, cy = 210, dx = 160, dy = 128;
    const arms = [[1, -1], [-1, -1], [-1, 1], [1, 1]];   // front right first: the anchor
    let s = "";
    s += `<g ${a("airframe")}>`;
    for (const [sx, sy] of arms)
      s += `<line class="g-arm" x1="${cx}" y1="${cy}" x2="${cx + sx * dx}" y2="${cy + sy * dy}"/>`;
    s += `<rect class="g-gear" x="${cx - 100}" y="${cy - 70}" width="12" height="140" rx="6"/>` +
         `<rect class="g-gear" x="${cx + 88}" y="${cy - 70}" width="12" height="140" rx="6"/>` +
         `<rect class="g-body" x="${cx - 78}" y="${cy - 72}" width="156" height="144" rx="18"/>` +
         `<path class="g-mark" d="M${cx} ${cy - 92} l12 16 h-24 z"/></g>`;
    arms.forEach(([sx, sy], i) => {
      const ex = cx + sx * dx * 0.48, ey = cy + sy * dy * 0.48;
      s += `<g ${a("propulsion.esc", i === 0)}><rect class="g-esc" x="${ex - 13}" y="${ey - 8}" width="26" height="16" rx="3" ` +
           `transform="rotate(${Math.atan2(sy * dy, sx * dx) * 180 / Math.PI} ${ex} ${ey})"/></g>`;
    });
    s += `<g ${a("propulsion.battery", true)}><rect class="g-batt" x="${cx - bw / 2}" y="${cy + 6}" width="${bw}" height="48" rx="6"/>` +
         `<rect class="g-strap" x="${cx - bw / 4 - 4}" y="${cy + 2}" width="8" height="56"/>` +
         `<rect class="g-strap" x="${cx + bw / 4 - 4}" y="${cy + 2}" width="8" height="56"/>` +
         `<text class="g-label" x="${cx}" y="${cy + 35}">${n}S</text></g>`;
    s += fc(cx - 34, cy - 58, 68, 52);
    s += `<line class="g-line g-neutral" x1="${cx}" y1="${cy + 72}" x2="${cx}" y2="${cy + 108}"/>`;
    s += puck(cx, cy + 118, 20, "gnss", "magnetometer");
    arms.forEach(([sx, sy], i) => {
      const mx = cx + sx * dx, my = cy + sy * dy, spin = sx * sy > 0 ? "g-cw" : "g-ccw";
      s += `<g ${a("propulsion.propeller", i === 0)}><circle class="g-disc" cx="${mx}" cy="${my}" r="${r}"/>` +
           `<g class="g-spin ${spin}" style="transform-origin:${mx}px ${my}px">` +
           `<path class="g-blade" d="M${mx - r} ${my} q${r / 2} ${-r * 0.13} ${r} 0 q${r / 2} ${r * 0.13} ${r} 0 q${-r / 2} ${r * 0.13} ${-r} 0 q${-r / 2} ${-r * 0.13} ${-r} 0 z"/></g></g>`;
      s += `<g ${a("propulsion.motor", i === 0)}><circle class="g-motor" cx="${mx}" cy="${my}" r="${mr}"/>` +
           `<circle class="g-hub" cx="${mx}" cy="${my}" r="${mr * 0.38}"/></g>`;
    });
    return s;
  }

  function rover(p) {
    const bw = 70 + 40 * Math.min(1, mah(p.battery) / 10000), n = cells(p.battery);
    let s = `<g ${a("airframe")}>`;
    for (const [x, y] of [[255, 92], [475, 92], [255, 294], [475, 294]])
      s += `<rect class="g-wheel" x="${x}" y="${y}" width="74" height="34" rx="9"/>`;
    s += `<line class="g-axle" x1="292" y1="126" x2="292" y2="294"/>` +
         `<rect class="g-body" x="215" y="126" width="370" height="168" rx="16"/>` +
         `<rect class="g-bumper" x="585" y="150" width="14" height="120" rx="5"/>` +
         `<path class="g-mark" d="M612 210 l-16 -12 v24 z"/></g>`;
    s += `<g ${a("drive.part", true)}><rect class="g-motor-r" x="300" y="196" width="66" height="28" rx="10"/>` +
         `<circle class="g-hub" cx="292" cy="210" r="13"/><text class="g-tiny" x="334" y="236">gearmotor</text></g>`;
    s += `<g ${a("drive.battery", true)}><rect class="g-batt" x="${395 - bw / 2}" y="236" width="${bw}" height="44" rx="6"/>` +
         `<text class="g-label" x="395" y="263">${n}S</text></g>`;
    s += `<g ${a("steering.part", true)}><rect class="g-servo" x="520" y="194" width="34" height="22" rx="3"/>` +
         `<line class="g-link" x1="537" y1="194" x2="512" y2="126"/><line class="g-link" x1="537" y1="216" x2="512" y2="294"/>` +
         `<text class="g-tiny" x="537" y="230">servo</text></g>`;
    s += fc(372, 140, 68, 52);
    s += puck(482, 162, 18, "gnss", "magnetometer");
    return s;
  }

  function plane() {
    let s = `<g ${a("airframe")}>` +
      `<path class="g-body" d="M118 210 C130 196 220 188 300 188 L640 190 C680 192 700 202 704 210 C700 218 680 228 640 230 L300 232 C220 232 130 224 118 210 z"/>` +
      `<path class="g-wing" d="M360 196 L404 196 L420 44 L380 44 z"/><path class="g-wing" d="M360 224 L404 224 L420 376 L380 376 z"/>` +
      `<path class="g-wing" d="M126 204 L170 204 L178 132 L144 132 z"/><path class="g-wing" d="M126 216 L170 216 L178 288 L144 288 z"/>` +
      `<rect class="g-fin" x="122" y="206" width="64" height="8" rx="3"/>` +
      `<rect class="g-surface" x="374" y="52" width="10" height="92"/><rect class="g-surface" x="374" y="276" width="10" height="92"/>` +
      `<rect class="g-surface" x="140" y="140" width="9" height="60"/><rect class="g-surface" x="140" y="220" width="9" height="60"/>` +
      `<rect class="g-motor-r" x="690" y="198" width="22" height="24" rx="5"/>` +
      `<ellipse class="g-disc" cx="716" cy="210" rx="5" ry="64"/>` +
      `<rect class="g-batt" x="600" y="196" width="60" height="28" rx="5"/></g>`;
    const servo = (x, y, sx, sy, anchor) =>
      `<g ${a("servos.part", anchor)}><rect class="g-servo" x="${x}" y="${y}" width="22" height="14" rx="2"/>` +
      `<line class="g-link" x1="${x + 11}" y1="${y + 7}" x2="${sx}" y2="${sy}"/></g>`;
    s += servo(392, 100, 384, 98, true) + servo(392, 306, 384, 322, false) +
         servo(232, 196, 149, 170, false) + servo(232, 212, 149, 250, false);
    s += fc(452, 186, 68, 48);
    s += puck(558, 210, 14, "gnss", "magnetometer");
    return s;
  }

  const DRAW = { quad, rover, plane };

  // --- peripherals and payloads ------------------------------------------------------
  // Drawn when fitted: a labelled module each, sensors in one bay and payloads
  // in another, in the catalog's order, clear of the airframe.
  const LABEL = { rangefinder: "RANGE", airspeed: "PITOT", optical_flow: "FLOW", rpm: "RPM",
    power_module: "POWER", temperature: "TEMP", external_ahrs: "AHRS", visual_odometry: "VIO",
    beacon: "UWB", precision_landing: "IR-LOCK", gripper: "GRIPPER", winch: "WINCH",
    parachute: "CHUTE", sprayer: "SPRAYER", landing_gear: "GEAR", gimbal: "GIMBAL",
    tether: "TETHER", generator: "GENSET" };
  const grid = (x0, y0, cols, rows, dx, dy) => {
    const out = [];
    for (let r = 0; r < rows; r++) for (let c = 0; c < cols; c++) out.push([x0 + c * dx, y0 + r * dy]);
    return out;
  };
  const BAYS = {   // [sensor slots, payload slots]
    quad: [grid(150, 160, 2, 5, 58, 24), grid(594, 160, 2, 4, 58, 24)],
    rover: [grid(250, 18, 5, 2, 62, 26), grid(250, 336, 5, 2, 62, 26)],
    plane: [grid(450, 118, 4, 3, 62, 22), grid(450, 252, 4, 2, 62, 22)],
  };
  function modules(airframe, parts, levels) {
    const [sensors, payloads] = BAYS[airframe];
    let s = "", i = 0, j = 0;
    for (const p of parts) {
      if (!p.table || !levels[p.id] || levels[p.id].id === "off") continue;
      const bay = p.group === "Payloads" ? payloads : sensors;
      const slot = bay[p.group === "Payloads" ? j++ : i++];
      if (!slot) continue;
      const [x, y] = slot, w = 52, h = 18;
      s += `<g ${a(p.id, true)}><rect class="${p.group === "Payloads" ? "g-payload" : "g-chip"}" x="${x}" y="${y}" ` +
           `width="${w}" height="${h}" rx="4"/><text class="g-tiny g-in" x="${x + w / 2}" y="${y + 12.5}">` +
           `${LABEL[p.id] || p.id}</text></g>`;
    }
    return s;
  }

  // --- camera ---------------------------------------------------------------------
  // The stage's shape: the camera frames to it, so nothing is letterboxed.
  const ratio = () => (svg.clientWidth && svg.clientHeight ? svg.clientWidth / svg.clientHeight : VIEW[2] / VIEW[3]);

  function frameOf(key) {
    let els = key === "overview" ? [svg.querySelector(".g-vehicle")] :
      [...svg.querySelectorAll(`[data-focus="${key}"][data-anchor]`)];
    if (!els.length) els = [...svg.querySelectorAll(`[data-focus="${key}"], [data-focus^="${key}."]`)];
    if (!els.length || !els[0]) return frameOf("overview");
    let x0 = Infinity, y0 = Infinity, x1 = -Infinity, y1 = -Infinity;
    for (const e of els) {
      const b = e.getBBox();
      x0 = Math.min(x0, b.x); y0 = Math.min(y0, b.y);
      x1 = Math.max(x1, b.x + b.width); y1 = Math.max(y1, b.y + b.height);
    }
    // Room around the part (the whole vehicle: a margin), in the stage's shape.
    const room = key === "overview" ? 1.18 : 1.45;
    let w = Math.max(x1 - x0, 110) * room, h = Math.max(y1 - y0, 80) * room;
    const r = ratio();
    if (w / h < r) w = h * r; else h = w / r;
    return [(x0 + x1) / 2 - w / 2, (y0 + y1) / 2 - h / 2, w, h];
  }

  function fly(to) {
    cancelAnimationFrame(anim);
    const from = view.slice(), start = performance.now();
    const still = matchMedia("(prefers-reduced-motion: reduce)").matches;
    const ms = still ? 0 : 650;
    const step = (now) => {
      const k = ms ? Math.min(1, (now - start) / ms) : 1;
      const e = k < 0.5 ? 4 * k * k * k : 1 - Math.pow(-2 * k + 2, 3) / 2;
      view = from.map((v, i) => v + (to[i] - v) * e);
      svg.setAttribute("viewBox", view.join(" "));
      if (k < 1) anim = requestAnimationFrame(step);
    };
    anim = requestAnimationFrame(step);
  }

  function light() {
    const part = focusKey.split(".")[0];
    svg.classList.toggle("g-focused", focusKey !== "overview");
    for (const e of svg.querySelectorAll("[data-focus]")) {
      const f = e.dataset.focus;
      e.classList.toggle("g-on", focusKey !== "overview" && (f === focusKey || f.startsWith(focusKey + ".")));
      e.classList.toggle("g-near", focusKey !== "overview" && f !== focusKey && f.split(".")[0] === part);
    }
    svg.classList.toggle("g-fast", part === "propulsion");
    for (const c of chips.querySelectorAll("button"))
      c.setAttribute("aria-pressed", String(c.dataset.focus === focusKey));
    showCaption();
    showShelf();
  }

  // The shelf: the real parts that fit the focused part, as cards with their
  // headline figures; the installed one marked. Picking one installs it.
  function card(title, rows, pressed, act, badge) {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "g-card";
    b.setAttribute("aria-pressed", String(pressed));
    const name = Object.assign(document.createElement("strong"), { textContent: title });
    b.append(name);
    if (badge) b.append(Object.assign(document.createElement("em"), { textContent: badge }));
    const dl = document.createElement("dl");
    for (const [k, v] of rows) {
      if (k) dl.append(Object.assign(document.createElement("dt"), { textContent: k }));
      dl.append(Object.assign(document.createElement("dd"), { textContent: v }));
    }
    b.append(dl);
    b.addEventListener("click", act);
    return b;
  }

  function showShelf() {
    const [partId, slotKey] = focusKey.split(".");
    const p = info && info.parts.find((x) => x.id === partId);
    const slots = p ? (p.slots || []).filter((x) => x.options.length) : [];
    let cards = [];
    const generic = (pressed, act) => card("Generic", [["", "set the parameters yourself"]], pressed, act,
                                           pressed ? "installed" : "");
    if (!p) {
      cards = [];
    } else if (slots.length > 1 && !slotKey) {
      // An assembly (propulsion, drive): one card per slot, to go to it.
      cards = slots.map((sl) => {
        const id = info.chosen.slots[p.id]?.[sl.key];
        const o = sl.options.find((x) => x.id === id);
        return card(sl.title, o ? [["", o.name]] : [["", "generic"]], false,
                    () => { focus(`${p.id}.${sl.key}`); onPick(`${p.id}.${sl.key}`); }, "choose ›");
      });
    } else if (slots.length) {
      const sl = slots.find((x) => x.key === slotKey) || slots[0];
      const picked = info.chosen.slots[p.id]?.[sl.key] || "";
      cards = [generic(!picked, () => onChoose(p.id, sl.key, "")),
               ...sl.options.map((o) => card(o.name, o.stats || [], o.id === picked,
                 () => onChoose(p.id, sl.key, o.id), o.id === picked ? "installed" : ""))];
    } else if ((p.devices || []).length) {
      const picked = info.chosen.devices[p.id] || "";
      cards = [generic(!picked, () => onChoose(p.id, null, "")),
               ...p.devices.map((d) => card(d.name, d.stats || [], d.id === picked,
                 () => onChoose(p.id, null, d.id), d.id === picked ? "installed" : ""))];
    }
    shelf.hidden = !cards.length;
    shelf.replaceChildren(...cards);
    const on = shelf.querySelector('[aria-pressed="true"]');
    if (on) shelf.scrollLeft = Math.max(0, on.offsetLeft - shelf.clientWidth / 2 + on.clientWidth / 2);
  }

  function showCaption() {
    if (!info) return;
    const [part, slot] = focusKey.split(".");
    const p = info.parts.find((x) => x.id === part);
    let title = "Your " + info.airframe, lines = ["Pick a part, here or in its card below."];
    if (p) {
      title = p.title;
      const lv = info.levels[p.id];
      lines = [];
      if (p.levels.length > 1 && lv) lines.push(lv.label);
      if (slot) {
        const s = (p.slots || []).find((x) => x.key === slot);
        title = s ? `${p.title}: ${s.title}` : title;
        lines.push(info.picked[part]?.[slot] || "Generic: set the parameters");
      } else if (p.slots) {
        const names = Object.values(info.picked[part] || {});
        lines.push(names.length ? names.join(" · ") : "Generic: set the parameters");
      } else if (p.devices && p.devices.length) {
        lines.push(info.devices[p.id] || "Generic: set the parameters");
      }
    }
    caption.replaceChildren(
      Object.assign(document.createElement("strong"), { textContent: title }),
      ...lines.map((l) => Object.assign(document.createElement("span"), { textContent: l })));
  }

  function chipsFor(parts) {
    const btn = (key, label, sub = false) => {
      const b = document.createElement("button");
      b.type = "button"; b.dataset.focus = key; b.textContent = label;
      if (sub) b.className = "sub";
      b.addEventListener("click", () => { focus(key); onPick(key); });
      return b;
    };
    const out = [btn("overview", "Overview")];
    for (const p of parts) {
      out.push(btn(p.id, p.title.replace(/ \(.*\)$/, "")));
      for (const s of (p.slots || []).filter((s) => s.options.length))
        out.push(btn(`${p.id}.${s.key}`, s.title, true));
    }
    chips.replaceChildren(...out);
  }

  // --- the page's side ---------------------------------------------------------------
  function mount(stage, pick, choose) {
    onPick = pick;
    onChoose = choose;
    stage.innerHTML = `<svg class="g-svg" viewBox="${VIEW.join(" ")}" role="img" aria-label="The vehicle"></svg>` +
      `<div class="g-caption" aria-live="polite"></div>` +
      `<div class="g-shelf" role="group" aria-label="Real parts" hidden></div>` +
      `<div class="g-chips" role="toolbar" aria-label="Parts"></div>`;
    svg = stage.querySelector("svg");
    caption = stage.querySelector(".g-caption");
    chips = stage.querySelector(".g-chips");
    shelf = stage.querySelector(".g-shelf");
    new ResizeObserver(() => { if (info) { cancelAnimationFrame(anim); view = frameOf(focusKey);
      svg.setAttribute("viewBox", view.join(" ")); } }).observe(svg);
    svg.addEventListener("click", (e) => {
      const hit = e.target.closest("[data-focus]");
      const key = hit ? hit.dataset.focus : "overview";
      focus(key);
      onPick(key);
    });
  }

  // info: {airframe, parts, levels: {id: level}, devices: {id: name},
  //        picked: {part: {slot: name}}, chosen: {devices: {part: id}, slots: {part: {slot: id}}}}
  function update(next) {
    const changedFrame = !info || info.airframe !== next.airframe;
    info = next;
    const p = info.picked;
    const names = { propeller: p.propulsion?.propeller, motor: p.propulsion?.motor,
                    battery: p.propulsion?.battery || p.drive?.battery };
    svg.innerHTML = `<defs><radialGradient id="g-floor"><stop offset="0" class="g-floor-a"/>` +
      `<stop offset="1" class="g-floor-b"/></radialGradient></defs>` +
      `<ellipse cx="400" cy="214" rx="380" ry="196" fill="url(#g-floor)"/>` +
      `<g class="g-vehicle">${DRAW[info.airframe](names)}` +
      `${modules(info.airframe, info.parts, info.levels)}</g>`;
    chipsFor(info.parts.filter((x) => !(x.table && info.levels[x.id]?.id === "off")));
    if (changedFrame) focusKey = "overview";
    light();
    cancelAnimationFrame(anim);
    view = frameOf(focusKey);  // a redraw keeps the camera on its part
    svg.setAttribute("viewBox", view.join(" "));
  }

  function focus(key) {
    if (!svg || key === focusKey) return;
    const part = key.split(".")[0];
    if (key !== "overview" && !info.parts.some((p) => p.id === part)) key = "overview";
    focusKey = key;
    light();
    fly(frameOf(key));
  }

  return { mount, update, focus, get key() { return focusKey; } };
})();
