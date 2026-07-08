const state = {
  schemas: [],
  entities: [],
  projects: [],
  selectedSchema: null,
  selectedEntity: null,
  selectedProject: null,
};

const $ = (id) => document.getElementById(id);

function toast(message, isError = false) {
  const el = $("toast");
  el.textContent = message;
  el.className = `toast show${isError ? " error" : ""}`;
  setTimeout(() => { el.className = "toast"; }, 3000);
}

async function api(path, options = {}) {
  const res = await fetch(path, {
    headers: { "Content-Type": "application/json" },
    ...options,
  });
  const data = await res.json();
  if (!data.ok) throw new Error(data.error || "API failed");
  return data;
}

function pretty(data) {
  return JSON.stringify(data, null, 2);
}

function parseEditor(id) {
  try {
    return JSON.parse($(id).value);
  } catch (err) {
    throw new Error(`JSON parse error: ${err.message}`);
  }
}

async function loadState() {
  const data = await api("/api/state");
  state.schemas = data.schemas;
  state.entities = data.entities;
  state.projects = data.projects;
  $("libraryRoot").textContent = data.library_root;
  renderAll();
}

function renderAll() {
  renderSchemaList();
  renderEntityList();
  renderProjectList();
  renderSelects();
  renderBindingProject();
}

function renderSchemaList() {
  const list = $("schemaList");
  list.innerHTML = "";
  state.schemas.forEach((item) => {
    const div = document.createElement("div");
    div.className = `list-item${state.selectedSchema?.id === item.id ? " active" : ""}`;
    div.innerHTML = `<strong>${item.id}</strong><span>${item.display_name || ""}</span>`;
    div.onclick = () => selectSchema(item.id);
    list.appendChild(div);
  });
  if (!state.selectedSchema && state.schemas.length) selectSchema(state.schemas[0].id, false);
}

function selectSchema(id, rerender = true) {
  state.selectedSchema = state.schemas.find((item) => item.id === id) || null;
  if (!state.selectedSchema) return;
  $("schemaEditorTitle").textContent = `Template JSON: ${id}`;
  $("schemaEditor").value = pretty(state.selectedSchema.data);
  if (rerender) renderSchemaList();
}

function newSchema() {
  const data = {
    id: "new_hardware_template",
    display_name: "New Hardware Template",
    description: "",
    category: "custom",
    output_subdir: "custom",
    parameters: [
      {
        name: "nameplate",
        c_name: "NAMEPLATE",
        description: "Nameplate text.",
        required: true,
        value_format: "\"{}\"",
      },
    ],
    exports: {},
  };
  state.selectedSchema = { id: data.id, display_name: data.display_name, data };
  $("schemaEditorTitle").textContent = "Template JSON: new";
  $("schemaEditor").value = pretty(data);
  renderSchemaList();
}

async function saveSchema() {
  const data = parseEditor("schemaEditor");
  await api("/api/schema", { method: "PUT", body: JSON.stringify(data) });
  toast(`Saved template ${data.id}`);
  await loadState();
  selectSchema(data.id);
}

function renderEntityList() {
  const list = $("entityList");
  list.innerHTML = "";
  state.entities.forEach((item) => {
    const div = document.createElement("div");
    div.className = `list-item${state.selectedEntity?.id === item.id ? " active" : ""}`;
    div.innerHTML = `<strong>${item.id}</strong><span>${item.schema} · ${item.display_name || ""}</span>`;
    div.onclick = () => selectEntity(item.id);
    list.appendChild(div);
  });
  if (!state.selectedEntity && state.entities.length) selectEntity(state.entities[0].id, false);
}

function renderSelects() {
  fillSelect($("entitySchemaSelect"), state.schemas.map((s) => [s.id, `${s.id} - ${s.display_name}`]));
  fillSelect($("generateEntitySelect"), state.entities.map((e) => [e.id, e.id]));
  fillSelect($("generateProjectSelect"), state.projects.map((p) => [p.id, p.id]));
  fillSelect($("bindingProjectSelect"), state.projects.map((p) => [p.id, p.id]));
}

function fillSelect(select, options) {
  const current = select.value;
  select.innerHTML = "";
  options.forEach(([value, label]) => {
    const opt = document.createElement("option");
    opt.value = value;
    opt.textContent = label;
    select.appendChild(opt);
  });
  if (options.some(([value]) => value === current)) select.value = current;
}

function selectEntity(id, rerender = true) {
  state.selectedEntity = state.entities.find((item) => item.id === id) || null;
  if (!state.selectedEntity) return;
  $("entityEditor").value = pretty(state.selectedEntity.data);
  $("entitySchemaSelect").value = state.selectedEntity.schema;
  renderEntityForm(state.selectedEntity.data);
  if (rerender) renderEntityList();
}

function newEntity() {
  const schema = state.schemas[0]?.id || "current_sensor";
  const data = {
    id: "new_entity",
    schema,
    display_name: "New Entity",
    macro_prefix: "NEW_ENTITY",
    parameters: {},
    components: {},
  };
  state.selectedEntity = { id: data.id, schema, display_name: data.display_name, data };
  $("entityEditor").value = pretty(data);
  $("entitySchemaSelect").value = schema;
  renderEntityForm(data);
  renderEntityList();
}

function renderEntityForm(data) {
  const schema = schemaById(data.schema || $("entitySchemaSelect").value);
  const root = $("entityForm");
  root.innerHTML = "";

  root.appendChild(inputRow("ID", "entity_id", data.id || ""));
  root.appendChild(inputRow("Display Name", "entity_display_name", data.display_name || ""));
  root.appendChild(inputRow("Macro Prefix", "entity_macro_prefix", data.macro_prefix || ""));

  const h = document.createElement("h3");
  h.textContent = "Parameters";
  root.appendChild(h);
  const grid = document.createElement("div");
  grid.className = "form-grid";
  (schema?.data.parameters || []).forEach((p) => {
    const value = data.parameters?.[p.name] ?? p.default ?? "";
    grid.appendChild(inputRow(`${p.name}${p.unit ? ` (${p.unit})` : ""}`, `param_${p.name}`, value));
  });
  root.appendChild(grid);

  if (schema?.data.component_slots) {
    const ch = document.createElement("h3");
    ch.textContent = "Components";
    root.appendChild(ch);
    Object.keys(schema.data.component_slots).forEach((slot) => {
      const comp = data.components?.[slot] || {};
      const row = document.createElement("div");
      row.className = "form-row";
      const current = comp.entity || "";
      const accepted = schema.data.component_slots[slot].accepted_schemas || [];
      const options = state.entities
        .filter((e) => !accepted.length || accepted.includes(e.schema))
        .map((e) => `<option value="${escapeHtml(e.id)}"${e.id === current ? " selected" : ""}>${escapeHtml(e.id)}</option>`)
        .join("");
      row.innerHTML = `<label>${slot}</label><select id="component_${slot}"><option value="">-- none / inline in JSON --</option>${options}</select>`;
      root.appendChild(row);
    });
  }
}

function inputRow(label, id, value) {
  const row = document.createElement("div");
  row.className = "form-row";
  row.innerHTML = `<label>${escapeHtml(label)}</label><input id="${escapeHtml(id)}" value="${escapeHtml(String(value))}">`;
  return row;
}

function schemaById(id) {
  return state.schemas.find((s) => s.id === id);
}

function syncEntityFormToJson() {
  const schemaId = $("entitySchemaSelect").value;
  const schema = schemaById(schemaId);
  const current = parseEditor("entityEditor");
  current.id = $("entity_id").value.trim();
  current.schema = schemaId;
  current.display_name = $("entity_display_name").value.trim();
  current.macro_prefix = $("entity_macro_prefix").value.trim();
  current.parameters = current.parameters || {};
  (schema?.data.parameters || []).forEach((p) => {
    const raw = $(`param_${p.name}`).value.trim();
    current.parameters[p.name] = castValue(raw);
  });
  current.components = current.components || {};
  if (schema?.data.component_slots) {
    Object.keys(schema.data.component_slots).forEach((slot) => {
      const value = $(`component_${slot}`)?.value || "";
      if (value) current.components[slot] = { entity: value };
    });
  }
  $("entityEditor").value = pretty(current);
}

async function saveEntity() {
  const data = parseEditor("entityEditor");
  await api("/api/entity", { method: "PUT", body: JSON.stringify(data) });
  toast(`Saved entity ${data.id}`);
  await loadState();
  selectEntity(data.id);
}

function renderProjectList() {
  const list = $("projectList");
  list.innerHTML = "";
  state.projects.forEach((item) => {
    const div = document.createElement("div");
    div.className = `list-item${state.selectedProject?.id === item.id ? " active" : ""}`;
    div.innerHTML = `<strong>${item.id}</strong><span>${item.suite} · ${item.display_name || ""}</span>`;
    div.onclick = () => selectProject(item.id);
    list.appendChild(div);
  });
  if (!state.selectedProject && state.projects.length) selectProject(state.projects[0].id, false);
}

function selectProject(id, rerender = true) {
  state.selectedProject = state.projects.find((item) => item.id === id) || null;
  if (!state.selectedProject) return;
  $("projectEditor").value = pretty(state.selectedProject.data);
  renderProjectForm(state.selectedProject.data);
  $("bindingProjectSelect").value = id;
  renderBindingProject();
  if (rerender) renderProjectList();
}

function newProject() {
  const data = {
    id: "new_suite_project",
    display_name: "New Suite Project",
    suite: "mcs_pmsm_nt",
    output_header: "sdpe_project_bindings.h",
    hardware: [],
    requirements: [],
    peripheral_bindings: {},
    global_macros: {},
  };
  state.selectedProject = { id: data.id, suite: data.suite, display_name: data.display_name, data };
  $("projectEditor").value = pretty(data);
  renderProjectForm(data);
  renderProjectList();
}

function renderProjectForm(data) {
  const root = $("projectQuickForm");
  root.innerHTML = "";
  root.appendChild(inputRow("ID", "project_id", data.id || ""));
  root.appendChild(inputRow("Display Name", "project_display_name", data.display_name || ""));
  root.appendChild(inputRow("Suite", "project_suite", data.suite || ""));
  root.appendChild(inputRow("Output Header", "project_output_header", data.output_header || "sdpe_project_bindings.h"));
  const hint = document.createElement("p");
  hint.className = "tag";
  hint.textContent = "Hardware, requirements, peripheral_bindings and global_macros can be edited in JSON or Binding page.";
  root.appendChild(hint);
}

function syncProjectFormToJson() {
  const data = parseEditor("projectEditor");
  data.id = $("project_id").value.trim();
  data.display_name = $("project_display_name").value.trim();
  data.suite = $("project_suite").value.trim();
  data.output_header = $("project_output_header").value.trim();
  data.hardware = data.hardware || [];
  data.requirements = data.requirements || [];
  $("projectEditor").value = pretty(data);
}

async function saveProjectFromEditor() {
  const data = parseEditor("projectEditor");
  await api("/api/project", { method: "PUT", body: JSON.stringify(data) });
  toast(`Saved project ${data.id}`);
  await loadState();
  selectProject(data.id);
}

async function renderBindingProject() {
  const projectId = $("bindingProjectSelect").value || state.selectedProject?.id;
  const project = state.projects.find((p) => p.id === projectId);
  const root = $("bindingMatrix");
  root.innerHTML = "";
  if (!project) return;
  state.selectedProject = project;
  const exports = [];
  for (const hw of project.data.hardware || []) {
    try {
      const res = await api(`/api/exports?entity=${encodeURIComponent(hw.entity)}`);
      res.exports.forEach((item) => exports.push(item));
    } catch (err) {
      toast(err.message, true);
    }
  }

  const exportOptions = exports.map((e) => `<option value="${escapeHtml(e.path)}">${escapeHtml(e.path)}</option>`).join("");
  (project.data.requirements || []).forEach((req, index) => {
    const current = req.binding?.export || req.binding?.macro || req.binding?.literal || "";
    const row = document.createElement("div");
    row.className = "binding-row";
    row.innerHTML = `
      <div><strong>${escapeHtml(req.macro || "")}</strong><br><span class="tag">${escapeHtml(req.role || "")}</span></div>
      <select id="binding_type_${index}">
        <option value="export"${req.binding?.export ? " selected" : ""}>export</option>
        <option value="macro"${req.binding?.macro ? " selected" : ""}>macro</option>
        <option value="literal"${req.binding?.literal ? " selected" : ""}>literal</option>
      </select>
      <div>
        <input list="export_list_${index}" id="binding_value_${index}" value="${escapeHtml(current)}">
        <datalist id="export_list_${index}">${exportOptions}</datalist>
      </div>`;
    root.appendChild(row);
  });
}

function applyBindingsToProjectJson() {
  const project = state.selectedProject;
  if (!project) return;
  const data = parseEditor("projectEditor");
  (data.requirements || []).forEach((req, index) => {
    const type = $(`binding_type_${index}`)?.value || "export";
    const value = $(`binding_value_${index}`)?.value || "";
    req.binding = { [type]: value };
  });
  $("projectEditor").value = pretty(data);
  toast("Bindings applied to project JSON");
}

async function generate() {
  const mode = $("generateMode").value;
  const body = {
    mode,
    out: $("generateOut").value,
    include_prefix: $("generateIncludePrefix").value,
  };
  if (mode === "entity") body.entity = $("generateEntitySelect").value;
  if (mode === "project") body.project = $("generateProjectSelect").value;
  const res = await api("/api/generate", { method: "POST", body: JSON.stringify(body) });
  $("generateLog").textContent = res.files.map((f) => `${f.changed ? "updated" : "unchanged"}: ${f.path}`).join("\n");
  toast(`Generated ${res.files.length} file(s)`);
}

function updateGenerateVisibility() {
  const mode = $("generateMode").value;
  $("generateEntityWrap").style.display = mode === "entity" ? "" : "none";
  $("generateProjectWrap").style.display = mode === "project" ? "" : "none";
}

function castValue(raw) {
  if (raw === "true") return true;
  if (raw === "false") return false;
  if (raw !== "" && !Number.isNaN(Number(raw))) return Number(raw);
  return raw;
}

function escapeHtml(text) {
  return String(text)
    .replaceAll("&", "&amp;")
    .replaceAll("<", "&lt;")
    .replaceAll(">", "&gt;")
    .replaceAll('"', "&quot;");
}

function initNav() {
  document.querySelectorAll(".nav-item").forEach((btn) => {
    btn.onclick = () => {
      document.querySelectorAll(".nav-item").forEach((item) => item.classList.remove("active"));
      document.querySelectorAll(".page").forEach((item) => item.classList.remove("active"));
      btn.classList.add("active");
      $(`page-${btn.dataset.page}`).classList.add("active");
      if (btn.dataset.page === "bindings") renderBindingProject();
    };
  });
}

function initEvents() {
  $("reloadBtn").onclick = () => loadState().then(() => toast("Reloaded")).catch((e) => toast(e.message, true));
  $("validateBtn").onclick = () => api("/api/validate").then((r) => toast(`Validation passed: ${r.schemas} schemas, ${r.entities} entities`)).catch((e) => toast(e.message, true));
  $("newSchemaBtn").onclick = newSchema;
  $("saveSchemaBtn").onclick = () => saveSchema().catch((e) => toast(e.message, true));
  $("newEntityBtn").onclick = newEntity;
  $("saveEntityBtn").onclick = () => saveEntity().catch((e) => toast(e.message, true));
  $("syncEntityFormBtn").onclick = () => { try { syncEntityFormToJson(); } catch (e) { toast(e.message, true); } };
  $("entitySchemaSelect").onchange = () => {
    const data = parseEditor("entityEditor");
    data.schema = $("entitySchemaSelect").value;
    $("entityEditor").value = pretty(data);
    renderEntityForm(data);
  };
  $("newProjectBtn").onclick = newProject;
  $("saveProjectBtn").onclick = () => saveProjectFromEditor().catch((e) => toast(e.message, true));
  $("syncProjectFormBtn").onclick = () => { try { syncProjectFormToJson(); } catch (e) { toast(e.message, true); } };
  $("generateMode").onchange = updateGenerateVisibility;
  $("generateBtn").onclick = () => generate().catch((e) => toast(e.message, true));
  $("bindingProjectSelect").onchange = () => {
    selectProject($("bindingProjectSelect").value);
    renderBindingProject();
  };
  $("applyBindingsBtn").onclick = applyBindingsToProjectJson;
  $("saveBindingsBtn").onclick = () => saveProjectFromEditor().catch((e) => toast(e.message, true));
}

initNav();
initEvents();
updateGenerateVisibility();
loadState().catch((err) => toast(err.message, true));
