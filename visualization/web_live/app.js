const canvas = document.getElementById("live-canvas");
const ctx = canvas.getContext("2d");

const connectionPill = document.getElementById("connection-pill");
const statusText = document.getElementById("status-text");
const sourceText = document.getElementById("source-text");
const poseHzText = document.getElementById("pose-hz-text");
const velocityHzText = document.getElementById("velocity-hz-text");
const poseCountText = document.getElementById("pose-count-text");
const velocityCountText = document.getElementById("velocity-count-text");
const rosoutCountText = document.getElementById("rosout-count-text");
const paramCountText = document.getElementById("param-count-text");
const topicList = document.getElementById("topic-list");
const trackerList = document.getElementById("tracker-list");
const rosoutList = document.getElementById("rosout-list");
const parameterList = document.getElementById("parameter-list");
const viewSummary = document.getElementById("view-summary");
const latencySummary = document.getElementById("latency-summary");

const palette = [
  "#7be0c7",
  "#ffbf69",
  "#8eb9ff",
  "#ff8f8f",
  "#c6a0ff",
  "#7fd1ff",
  "#ffd166",
  "#91f2a0",
];

const maxDiagnostics = 24;

const state = {
  config: null,
  stats: null,
  trackers: new Map(),
  diagnostics: {
    logs: [],
    parameterEvents: [],
  },
  connection: "connecting",
  lastSequence: 0,
  lastEventAt: 0,
};

function hashString(value) {
  let hash = 0;
  for (let index = 0; index < value.length; index += 1) {
    hash = (hash * 31 + value.charCodeAt(index)) >>> 0;
  }
  return hash;
}

function trackerColor(id) {
  return palette[hashString(id) % palette.length];
}

function formatMeters(value) {
  return `${value.toFixed(2)} m`;
}

function formatSpeed(value) {
  return `${value.toFixed(2)} m/s`;
}

function formatVector(sample) {
  return `${sample.x_m.toFixed(2)}, ${sample.y_m.toFixed(2)}, ${sample.z_m.toFixed(2)}`;
}

function cloneSample(sample) {
  return {
    id: sample.id,
    stamp_ns: sample.stamp_ns,
    x_m: sample.x_m,
    y_m: sample.y_m,
    z_m: sample.z_m,
    qw: sample.qw,
    qx: sample.qx,
    qy: sample.qy,
    qz: sample.qz,
  };
}

function cloneVelocity(velocity) {
  if (!velocity) {
    return null;
  }
  return {
    id: velocity.id,
    stamp_ns: velocity.stamp_ns,
    vx_mps: velocity.vx_mps,
    vy_mps: velocity.vy_mps,
    vz_mps: velocity.vz_mps,
    speed_mps: velocity.speed_mps,
  };
}

function applySnapshot(payload) {
  state.config = payload.config;
  state.stats = payload.stats;
  state.lastSequence = payload.sequence;
  state.lastEventAt = performance.now();
  state.trackers.clear();

  payload.trackers.forEach((tracker) => {
    state.trackers.set(tracker.id, {
      id: tracker.id,
      color: trackerColor(tracker.id),
      sampleCount: tracker.sample_count,
      velocityCount: tracker.velocity_count ?? 0,
      latest: tracker.latest ? cloneSample(tracker.latest) : null,
      latestVelocity: cloneVelocity(tracker.latest_velocity),
      trail: tracker.trail.map(cloneSample),
    });
  });

  state.diagnostics.logs = [...(payload.diagnostics?.logs ?? [])];
  state.diagnostics.parameterEvents = [
    ...(payload.diagnostics?.parameter_events ?? []),
  ];

  updatePanels();
}

function ensureTracker(id) {
  let tracker = state.trackers.get(id);
  if (!tracker) {
    tracker = {
      id,
      color: trackerColor(id),
      sampleCount: 0,
      velocityCount: 0,
      latest: null,
      latestVelocity: null,
      trail: [],
    };
    state.trackers.set(id, tracker);
  }
  return tracker;
}

function applyDelta(payload) {
  state.stats = payload.stats;
  state.lastSequence = payload.sequence;
  state.lastEventAt = performance.now();

  payload.samples.forEach((sample) => {
    const tracker = ensureTracker(sample.id);
    tracker.latest = cloneSample(sample);
    tracker.sampleCount += 1;
    tracker.trail.push(cloneSample(sample));

    if (sample.velocity) {
      tracker.latestVelocity = cloneVelocity(sample.velocity);
    }

    const maxTrailPoints = state.config?.maxTrailPoints ?? 1500;
    if (tracker.trail.length > maxTrailPoints) {
      tracker.trail.splice(0, tracker.trail.length - maxTrailPoints);
    }
  });

  updatePanels();
}

function applyVelocityEvent(payload) {
  state.stats = payload.stats;
  state.lastSequence = payload.sequence;
  state.lastEventAt = performance.now();

  const velocity = cloneVelocity(payload.velocity);
  const tracker = ensureTracker(velocity.id);
  tracker.latestVelocity = velocity;
  tracker.velocityCount += 1;

  updatePanels();
}

function pushDiagnostic(target, entry) {
  target.unshift(entry);
  if (target.length > maxDiagnostics) {
    target.splice(maxDiagnostics);
  }
}

function applyLogEvent(payload) {
  state.stats = payload.stats;
  state.lastSequence = payload.sequence;
  pushDiagnostic(state.diagnostics.logs, payload.log);
  updatePanels();
}

function applyParameterEvent(payload) {
  state.stats = payload.stats;
  state.lastSequence = payload.sequence;
  pushDiagnostic(state.diagnostics.parameterEvents, payload.parameter_event);
  updatePanels();
}

function setConnectionState(status) {
  state.connection = status;
  connectionPill.textContent = status;
  connectionPill.style.color = status === "streaming" ? "#7be0c7" : "#ffbf69";
  connectionPill.style.borderColor =
    status === "streaming" ? "rgba(123, 224, 199, 0.3)" : "rgba(255, 191, 105, 0.3)";
  connectionPill.style.background =
    status === "streaming" ? "rgba(123, 224, 199, 0.12)" : "rgba(255, 191, 105, 0.12)";
}

function sourceLabel() {
  const source = state.config?.source;
  if (!source) {
    return "-";
  }
  if (source.mode === "direct") {
    return `${source.device} • ${String(source.stream).toUpperCase()} • ${source.freq_hz.toFixed(0)} Hz`;
  }
  return "ROS 2 bridge";
}

function renderTopics() {
  const topics = state.config?.topics;
  if (!topics) {
    topicList.innerHTML = '<div class="empty-state">Waiting for topic configuration.</div>';
    return;
  }

  const rows = [
    ["Pose", topics.pose],
    ["Velocity", topics.velocity],
    ["Rosout", topics.rosout],
    ["Params", topics.parameter_events],
  ]
    .filter((entry) => entry[1])
    .map(
      ([label, topic]) => `
        <div class="topic-row">
          <strong>${label}</strong>
          <code>${topic}</code>
        </div>
      `
    )
    .join("");

  topicList.innerHTML = rows || '<div class="empty-state">No ROS 2 topics configured.</div>';
}

function logLevelLabel(level) {
  if (level >= 50) {
    return "FATAL";
  }
  if (level >= 40) {
    return "ERROR";
  }
  if (level >= 30) {
    return "WARN";
  }
  if (level >= 20) {
    return "INFO";
  }
  if (level >= 10) {
    return "DEBUG";
  }
  return "UNSET";
}

function renderDiagnostics() {
  const logs = state.diagnostics.logs
    .map(
      (entry) => `
        <div class="diagnostic-card">
          <strong>${logLevelLabel(entry.level)} • ${entry.name}</strong>
          <p>${entry.message}</p>
        </div>
      `
    )
    .join("");

  rosoutList.innerHTML = logs || '<div class="empty-state">No rosout messages yet.</div>';

  const parameterEvents = state.diagnostics.parameterEvents
    .map((entry) => {
      const summary = [
        entry.new_parameters?.length ? `new ${entry.new_parameters.length}` : null,
        entry.changed_parameters?.length
          ? `changed ${entry.changed_parameters.length}`
          : null,
        entry.deleted_parameters?.length
          ? `deleted ${entry.deleted_parameters.length}`
          : null,
      ]
        .filter(Boolean)
        .join(" • ");

      const names = [
        ...(entry.new_parameters ?? []),
        ...(entry.changed_parameters ?? []),
        ...(entry.deleted_parameters ?? []),
      ]
        .slice(0, 5)
        .join(", ");

      return `
        <div class="diagnostic-card">
          <strong>${entry.node}</strong>
          <p>${summary || "parameter event"}</p>
          <p>${names || "no parameter names reported"}</p>
        </div>
      `;
    })
    .join("");

  parameterList.innerHTML =
    parameterEvents || '<div class="empty-state">No parameter events yet.</div>';
}

function updatePanels() {
  const stats = state.stats ?? {};
  const trackers = Array.from(state.trackers.values()).sort((left, right) =>
    left.id.localeCompare(right.id)
  );

  statusText.textContent = stats.status ?? state.connection;
  sourceText.textContent = sourceLabel();
  poseHzText.textContent = `${(stats.pose_hz ?? 0).toFixed(1)} Hz`;
  velocityHzText.textContent = `${(stats.velocity_hz ?? 0).toFixed(1)} Hz`;
  poseCountText.textContent = String(stats.total_pose_frames ?? 0);
  velocityCountText.textContent = String(stats.total_velocity_frames ?? 0);
  rosoutCountText.textContent = String(stats.rosout_count ?? 0);
  paramCountText.textContent = String(stats.parameter_event_count ?? 0);

  const trackerCards = trackers
    .map((tracker) => {
      const latest = tracker.latest;
      const latestText = latest ? formatVector(latest) : "no pose yet";
      const speedText = tracker.latestVelocity
        ? formatSpeed(tracker.latestVelocity.speed_mps)
        : "no velocity yet";
      const lastAgeMs = state.lastEventAt ? performance.now() - state.lastEventAt : null;
      const freshnessText = lastAgeMs == null ? "n/a" : `${lastAgeMs.toFixed(0)} ms old`;
      return `
        <article class="tracker-card">
          <div class="tracker-swatch" style="background:${tracker.color}"></div>
          <div>
            <h3>Tracker ${tracker.id}</h3>
            <p>${latestText}</p>
            <p>${speedText} • ${tracker.trail.length} trail points • ${freshnessText}</p>
          </div>
        </article>
      `;
    })
    .join("");

  trackerList.innerHTML =
    trackerCards || '<div class="empty-state">No trackers received yet.</div>';

  if (trackers.length) {
    const allPoints = trackers.flatMap((tracker) => tracker.trail);
    const bounds = boundsFromPoints(allPoints);
    const maxSpeed = trackers.reduce(
      (current, tracker) =>
        Math.max(current, tracker.latestVelocity?.speed_mps ?? 0),
      0
    );
    if (bounds) {
      const width = bounds.maxX - bounds.minX;
      const depth = bounds.maxZ - bounds.minZ;
      viewSummary.textContent =
        `${trackers.length} tracker${trackers.length === 1 ? "" : "s"} • ` +
        `${formatMeters(width)} × ${formatMeters(depth)} • max ${formatSpeed(maxSpeed)}`;
    }
  } else {
    viewSummary.textContent = "Waiting for samples";
  }

  const poseAge = stats.last_pose_age_ms;
  const velocityAge = stats.last_velocity_age_ms;
  if (poseAge == null && velocityAge == null) {
    latencySummary.textContent = "No data yet";
  } else if (poseAge == null) {
    latencySummary.textContent = `vel ${velocityAge.toFixed(0)} ms`;
  } else if (velocityAge == null) {
    latencySummary.textContent = `pose ${poseAge.toFixed(0)} ms`;
  } else {
    latencySummary.textContent =
      `pose ${poseAge.toFixed(0)} ms • vel ${velocityAge.toFixed(0)} ms`;
  }

  if (stats.last_error) {
    latencySummary.textContent = stats.last_error;
  }

  renderTopics();
  renderDiagnostics();
  setConnectionState(stats.status ?? state.connection);
}

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const dpr = Math.max(window.devicePixelRatio || 1, 1);
  const width = Math.max(1, Math.round(rect.width * dpr));
  const height = Math.max(1, Math.round(rect.height * dpr));

  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width;
    canvas.height = height;
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }
}

function boundsFromPoints(points) {
  if (!points.length) {
    return null;
  }

  let minX = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let minZ = Number.POSITIVE_INFINITY;
  let maxZ = Number.NEGATIVE_INFINITY;

  points.forEach((point) => {
    minX = Math.min(minX, point.x_m);
    maxX = Math.max(maxX, point.x_m);
    minZ = Math.min(minZ, point.z_m);
    maxZ = Math.max(maxZ, point.z_m);
  });

  return { minX, maxX, minZ, maxZ };
}

function resolveRoomGeometry(trackers) {
  const room = state.config?.room;
  if (!room || room.hide) {
    return null;
  }

  const points = trackers.flatMap((tracker) => tracker.trail);
  const bounds = boundsFromPoints(points);
  const xSpan = bounds ? bounds.maxX - bounds.minX : 0;
  const zSpan = bounds ? bounds.maxZ - bounds.minZ : 0;
  const longAxis =
    room.longAxis === "auto" ? (xSpan >= zSpan ? "x" : "z") : room.longAxis;
  const sizeX = longAxis === "x" ? room.lengthM : room.widthM;
  const sizeZ = longAxis === "x" ? room.widthM : room.lengthM;

  let centerX = room.originX;
  let centerZ = room.originZ;
  if (room.anchor === "corner") {
    centerX += sizeX / 2;
    centerZ += sizeZ / 2;
  } else if (room.anchor === "trajectory_center" && bounds) {
    centerX = (bounds.minX + bounds.maxX) / 2;
    centerZ = (bounds.minZ + bounds.maxZ) / 2;
  }

  return {
    centerX,
    centerZ,
    sizeX,
    sizeZ,
  };
}

function resolveViewBounds(trackers, roomGeometry) {
  const points = trackers.flatMap((tracker) => tracker.trail);
  const pointBounds = boundsFromPoints(points);

  let minX;
  let maxX;
  let minZ;
  let maxZ;

  if (roomGeometry) {
    minX = roomGeometry.centerX - roomGeometry.sizeX / 2;
    maxX = roomGeometry.centerX + roomGeometry.sizeX / 2;
    minZ = roomGeometry.centerZ - roomGeometry.sizeZ / 2;
    maxZ = roomGeometry.centerZ + roomGeometry.sizeZ / 2;
  } else if (pointBounds) {
    minX = pointBounds.minX;
    maxX = pointBounds.maxX;
    minZ = pointBounds.minZ;
    maxZ = pointBounds.maxZ;
  } else {
    minX = -1;
    maxX = 1;
    minZ = -1;
    maxZ = 1;
  }

  const spanX = Math.max(maxX - minX, 0.5);
  const spanZ = Math.max(maxZ - minZ, 0.5);
  const padX = Math.max(spanX * 0.06, 0.35);
  const padZ = Math.max(spanZ * 0.06, 0.35);

  return {
    minX: minX - padX,
    maxX: maxX + padX,
    minZ: minZ - padZ,
    maxZ: maxZ + padZ,
  };
}

function worldToCanvas(bounds, width, height, x, z) {
  const margin = 24;
  const innerWidth = Math.max(1, width - margin * 2);
  const innerHeight = Math.max(1, height - margin * 2);
  const scaleX = innerWidth / Math.max(bounds.maxX - bounds.minX, 1e-6);
  const scaleZ = innerHeight / Math.max(bounds.maxZ - bounds.minZ, 1e-6);
  const scale = Math.min(scaleX, scaleZ);

  const contentWidth = (bounds.maxX - bounds.minX) * scale;
  const contentHeight = (bounds.maxZ - bounds.minZ) * scale;
  const originX = (width - contentWidth) / 2;
  const originY = (height - contentHeight) / 2;

  return {
    x: originX + (x - bounds.minX) * scale,
    y: height - originY - (z - bounds.minZ) * scale,
    scale,
  };
}

function drawGrid(bounds, width, height) {
  const minGrid = Math.floor(bounds.minX);
  const maxGrid = Math.ceil(bounds.maxX);
  const minDepth = Math.floor(bounds.minZ);
  const maxDepth = Math.ceil(bounds.maxZ);

  ctx.save();
  ctx.strokeStyle = "rgba(180, 220, 220, 0.08)";
  ctx.lineWidth = 1;

  for (let x = minGrid; x <= maxGrid; x += 1) {
    const top = worldToCanvas(bounds, width, height, x, bounds.maxZ);
    const bottom = worldToCanvas(bounds, width, height, x, bounds.minZ);
    ctx.beginPath();
    ctx.moveTo(top.x, top.y);
    ctx.lineTo(bottom.x, bottom.y);
    ctx.stroke();
  }

  for (let z = minDepth; z <= maxDepth; z += 1) {
    const left = worldToCanvas(bounds, width, height, bounds.minX, z);
    const right = worldToCanvas(bounds, width, height, bounds.maxX, z);
    ctx.beginPath();
    ctx.moveTo(left.x, left.y);
    ctx.lineTo(right.x, right.y);
    ctx.stroke();
  }

  ctx.restore();
}

function drawRoom(bounds, width, height, roomGeometry) {
  if (!roomGeometry) {
    return;
  }

  const topLeft = worldToCanvas(
    bounds,
    width,
    height,
    roomGeometry.centerX - roomGeometry.sizeX / 2,
    roomGeometry.centerZ + roomGeometry.sizeZ / 2
  );
  const bottomRight = worldToCanvas(
    bounds,
    width,
    height,
    roomGeometry.centerX + roomGeometry.sizeX / 2,
    roomGeometry.centerZ - roomGeometry.sizeZ / 2
  );

  ctx.save();
  ctx.fillStyle = "rgba(123, 224, 199, 0.08)";
  ctx.strokeStyle = "rgba(123, 224, 199, 0.42)";
  ctx.lineWidth = 1.5;
  ctx.setLineDash([10, 8]);
  ctx.beginPath();
  ctx.rect(
    topLeft.x,
    topLeft.y,
    bottomRight.x - topLeft.x,
    bottomRight.y - topLeft.y
  );
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawVelocityArrow(bounds, width, height, tracker) {
  if (!tracker.latest || !tracker.latestVelocity) {
    return;
  }

  const planarSpeed = Math.hypot(
    tracker.latestVelocity.vx_mps,
    tracker.latestVelocity.vz_mps
  );
  if (planarSpeed < 0.01) {
    return;
  }

  const lookaheadSeconds = 0.3;
  const start = worldToCanvas(
    bounds,
    width,
    height,
    tracker.latest.x_m,
    tracker.latest.z_m
  );
  const end = worldToCanvas(
    bounds,
    width,
    height,
    tracker.latest.x_m + tracker.latestVelocity.vx_mps * lookaheadSeconds,
    tracker.latest.z_m + tracker.latestVelocity.vz_mps * lookaheadSeconds
  );

  const dx = end.x - start.x;
  const dy = end.y - start.y;
  const length = Math.hypot(dx, dy);
  if (length < 10) {
    return;
  }

  const nx = dx / length;
  const ny = dy / length;
  const arrowSize = 10;

  ctx.save();
  ctx.strokeStyle = tracker.color;
  ctx.fillStyle = tracker.color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(start.x, start.y);
  ctx.lineTo(end.x, end.y);
  ctx.stroke();

  ctx.beginPath();
  ctx.moveTo(end.x, end.y);
  ctx.lineTo(
    end.x - nx * arrowSize - ny * (arrowSize * 0.55),
    end.y - ny * arrowSize + nx * (arrowSize * 0.55)
  );
  ctx.lineTo(
    end.x - nx * arrowSize + ny * (arrowSize * 0.55),
    end.y - ny * arrowSize - nx * (arrowSize * 0.55)
  );
  ctx.closePath();
  ctx.fill();
  ctx.restore();
}

function drawTrail(bounds, width, height, tracker) {
  if (!tracker.trail.length) {
    return;
  }

  ctx.save();
  ctx.lineJoin = "round";
  ctx.lineCap = "round";
  ctx.strokeStyle = tracker.color;
  ctx.lineWidth = 2.4;
  ctx.beginPath();

  tracker.trail.forEach((sample, index) => {
    const point = worldToCanvas(bounds, width, height, sample.x_m, sample.z_m);
    if (index === 0) {
      ctx.moveTo(point.x, point.y);
    } else {
      ctx.lineTo(point.x, point.y);
    }
  });
  ctx.stroke();

  const latest = tracker.latest;
  if (latest) {
    const point = worldToCanvas(bounds, width, height, latest.x_m, latest.z_m);
    ctx.fillStyle = tracker.color;
    ctx.strokeStyle = "rgba(3, 11, 15, 0.9)";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(point.x, point.y, 7, 0, Math.PI * 2);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = "#edf5f3";
    ctx.font = '600 12px "IBM Plex Mono", Consolas, monospace';
    ctx.fillText(`T${tracker.id}`, point.x + 10, point.y - 10);
  }

  ctx.restore();
}

function render() {
  resizeCanvas();
  const rect = canvas.getBoundingClientRect();
  const width = rect.width;
  const height = rect.height;

  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#08151b";
  ctx.fillRect(0, 0, width, height);

  const trackers = Array.from(state.trackers.values());
  const roomGeometry = resolveRoomGeometry(trackers);
  const bounds = resolveViewBounds(trackers, roomGeometry);

  drawGrid(bounds, width, height);
  drawRoom(bounds, width, height, roomGeometry);
  trackers.forEach((tracker) => drawTrail(bounds, width, height, tracker));
  trackers.forEach((tracker) => drawVelocityArrow(bounds, width, height, tracker));

  requestAnimationFrame(render);
}

async function loadSnapshot() {
  const response = await fetch("/snapshot", { cache: "no-store" });
  if (!response.ok) {
    throw new Error(`Snapshot request failed: ${response.status}`);
  }
  const payload = await response.json();
  applySnapshot(payload);
}

function connectEvents() {
  const events = new EventSource("/events");
  events.addEventListener("open", () => setConnectionState("streaming"));
  events.addEventListener("snapshot", (event) => {
    applySnapshot(JSON.parse(event.data));
  });
  events.addEventListener("delta", (event) => {
    applyDelta(JSON.parse(event.data));
  });
  events.addEventListener("velocity", (event) => {
    applyVelocityEvent(JSON.parse(event.data));
  });
  events.addEventListener("log", (event) => {
    applyLogEvent(JSON.parse(event.data));
  });
  events.addEventListener("parameter_event", (event) => {
    applyParameterEvent(JSON.parse(event.data));
  });
  events.addEventListener("error", () => {
    setConnectionState("reconnecting");
  });
}

window.addEventListener("resize", resizeCanvas);

loadSnapshot()
  .catch((error) => {
    latencySummary.textContent = error.message;
    setConnectionState("error");
  })
  .finally(() => {
    connectEvents();
    requestAnimationFrame(render);
  });
