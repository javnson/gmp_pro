import os
import sys
import json
import subprocess
import tkinter as tk
from tkinter import ttk, messagebox
from pathlib import Path

CHECKED = "☑ "
UNCHECKED = "☐ "
PARTIAL = "[-] "

class FrameworkUserGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP Project Framework Configurator V10 (Ultimate Edition)")
        self.root.geometry("1200x850")

        # Bind Ctrl+S for quick save
        self.root.bind("<Control-s>", lambda e: self.save_local_config(show_msg=True))
        self.root.bind("<Control-S>", lambda e: self.save_local_config(show_msg=True))

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("Error", "Environment variable GMP_PRO_LOCATION not found!")
            self.root.destroy()
            return
            
        # Core paths
        self.global_dic_path = Path(self.gmp_location) / "tools" / "facilities_generator" / "src_mgr" / "gmp_framework_dic.json"
        self.local_config_path = Path(os.getcwd()) / "gmp_framework_config.json"
        
        # Path to read developer markdown notes
        self.dev_notes_dir = Path(__file__).parent.resolve() / "dev_notes"
        
        self.registry = {"roots": [], "modules": {}, "macros": {}}
        self.selected_modules = set() 
        self.current_selected_node = None 
        
        self.load_global_dic()
        self.load_local_config()
        
        self.build_ui()
        self.refresh_tree()
        self.update_dashboard()

    # ================= Status Logic & Path Resolution =================
    def get_module_status(self, mod_key, mod_data):
        """Calculate module maturity level for user display"""
        if mod_key.endswith("_internal"):
            return "🟢", "Implicitly Verified (Internal Package)"

        tasks = mod_data.get("tasks", [])
        task_map = {t.get("id"): t.get("completed", False) for t in tasks}

        if task_map.get("sys_hw", False):
            return "🟢", "Ready for Production (HW Verified)"
        elif task_map.get("sys_sim", False):
            return "🔷", "Sim/Principle Verified"  
        elif task_map.get("sys_compile", False):
            return "🟨", "Compilation Passed"
        else:
            return "❌", "In Development (Unverified)"

    def resolve_paths(self, patterns, is_dir_mode=False, return_absolute=False):
        matched = set()
        base_dir = Path(self.gmp_location)
        macros = self.registry.get("macros", {})
        macros["GMP_PRO_LOCATION"] = base_dir.as_posix()

        for pat in patterns:
            resolved_pat = pat
            for mac, val in macros.items():
                resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)

            pat_obj = Path(resolved_pat)

            if pat_obj.is_absolute():
                search_base = Path(pat_obj.anchor)
                search_pattern = str(pat_obj.relative_to(pat_obj.anchor))
            else:
                search_base = base_dir
                search_pattern = str(pat_obj)

            if is_dir_mode:
                target = search_base / search_pattern
                if target.exists() and target.is_dir():
                    matched.add(target.resolve().as_posix() if return_absolute else target.as_posix())
            else:
                for f in search_base.glob(search_pattern):
                    if f.is_file():
                        matched.add(f.resolve().as_posix() if return_absolute else f.as_posix())

        return sorted(list(matched))

    # ================= Data Loading =================
    def load_global_dic(self):
        if not self.global_dic_path.exists():
            messagebox.showerror("Error", f"Global Framework Dictionary not found:\n{self.global_dic_path}")
            self.root.destroy()
            return
        try:
            with open(self.global_dic_path, 'r', encoding='utf-8') as f:
                self.registry = json.load(f)
        except Exception as e:
            messagebox.showerror("Read Error", f"Failed to parse global dictionary: {e}")

    def load_local_config(self):
        if self.local_config_path.exists():
            try:
                with open(self.local_config_path, 'r', encoding='utf-8') as f:
                    local_config = json.load(f)
                    for item in local_config.get("selected_modules", []):
                        root_name = item.get("root")
                        mod_key = item.get("module")
                        if root_name in self.registry.get("modules", {}) and mod_key in self.registry["modules"][root_name]:
                            self.selected_modules.add(f"{root_name}|{mod_key}")
            except Exception as e:
                print(f"Warning: Failed to read local configuration ({e})")

    # ================= UI Building =================
    def build_ui(self):
        top_frame = ttk.Frame(self.root, padding=10)
        top_frame.pack(fill=tk.X)
        ttk.Label(top_frame, text="Current Project Path (CWD):").pack(side=tk.LEFT)
        ttk.Label(top_frame, text=str(os.getcwd()), foreground="blue", font=("Helvetica", 10, "bold")).pack(side=tk.LEFT, padx=5)

        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # === Left Tree Panel ===
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        ttk.Label(left_frame, text="Available Modules (Double-click to toggle):", font=("Helvetica", 10, "bold")).pack(anchor=tk.W, pady=(0, 5))
        
        # [UI Layout FIX] Wrap Tree and Scrollbar in a dedicated container to avoid pack conflict
        tree_container = ttk.Frame(left_frame)
        tree_container.pack(fill=tk.BOTH, expand=True)

        tree_scroll = ttk.Scrollbar(tree_container)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(tree_container, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        
        self.tree.heading("#0", text=" Framework Repository", anchor=tk.W)
        self.tree.bind("<Double-1>", self.on_tree_double_click)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)
        
        # [UI Layout FIX] Status Legend packed strictly below the tree container, organized in grid
        legend_frame = ttk.LabelFrame(left_frame, text="Status Legend", padding=5)
        legend_frame.pack(fill=tk.X, pady=(5, 0))
        
        ttk.Label(legend_frame, text="🟢: Ready for Prod").grid(row=0, column=0, sticky=tk.W, padx=(5, 15), pady=2)
        ttk.Label(legend_frame, text="🔷: Sim/Principle").grid(row=0, column=1, sticky=tk.W, padx=5, pady=2)
        ttk.Label(legend_frame, text="🟨: Compiled").grid(row=1, column=0, sticky=tk.W, padx=(5, 15), pady=2)
        ttk.Label(legend_frame, text="❌: In Development").grid(row=1, column=1, sticky=tk.W, padx=5, pady=2)

        # === Right Tab Panel ===
        right_frame = ttk.Frame(self.paned_window, padding=5)
        self.paned_window.add(right_frame, weight=2)

        self.info_frame = ttk.LabelFrame(right_frame, text="Node Details & Resources", padding=10)
        self.info_frame.pack(fill=tk.X, pady=(0, 10))
        
        info_inner = ttk.Frame(self.info_frame)
        info_inner.pack(fill=tk.X)
        
        text_info_frame = ttk.Frame(info_inner)
        text_info_frame.pack(side=tk.LEFT, fill=tk.X, expand=True)
        self.lbl_node_name = ttk.Label(text_info_frame, font=("Helvetica", 11, "bold"), text="Please select a node on the left")
        self.lbl_node_name.pack(anchor=tk.W)
        self.lbl_node_desc = ttk.Label(text_info_frame, text="-", foreground="gray", wraplength=500)
        self.lbl_node_desc.pack(anchor=tk.W, pady=(5, 0))
        
        self.btn_open_help = ttk.Button(info_inner, text="🌐 Open Documentation", command=self.open_help_docs, state=tk.DISABLED)
        self.btn_open_help.pack(side=tk.RIGHT, padx=10, ipadx=5, ipady=5)

        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        # ------ Tab 1: Node Config Perspective ------
        self.tab_detail = ttk.Frame(self.notebook, padding=5)
        self.notebook.add(self.tab_detail, text="🔍 Node Config Perspective")
        
        pw_detail = ttk.PanedWindow(self.tab_detail, orient=tk.VERTICAL)
        pw_detail.pack(fill=tk.BOTH, expand=True)

        def create_detail_box(parent, title, height=3):
            frame = ttk.Frame(parent)
            ttk.Label(frame, text=title, font=("Helvetica", 9, "bold")).pack(anchor=tk.W)
            scroll = ttk.Scrollbar(frame)
            scroll.pack(side=tk.RIGHT, fill=tk.Y)
            txt = tk.Text(frame, height=height, bg="#fcfcfc", font=("Consolas", 9), yscrollcommand=scroll.set, state=tk.DISABLED)
            txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            scroll.config(command=txt.yview)
            parent.add(frame, weight=1)
            return txt

        self.txt_det_src = create_detail_box(pw_detail, "📄 Target Source Files (Unpacked):")
        self.txt_det_inc = create_detail_box(pw_detail, "📁 Target Header Files (Unpacked):")
        self.txt_det_dirs = create_detail_box(pw_detail, "⚙️ Include Directories (-I):")
        self.txt_det_deps = create_detail_box(pw_detail, "🔗 Required Dependencies:")

        # ------ Tab 2: Verification Report (READ-ONLY) ------
        self.tab_verification = ttk.Frame(self.notebook, padding=5)
        self.notebook.add(self.tab_verification, text="🛡️ Verification Report")
        
        verif_scroll = ttk.Scrollbar(self.tab_verification)
        verif_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.txt_verification = tk.Text(self.tab_verification, font=("Consolas", 10), bg="#f4f7f6", yscrollcommand=verif_scroll.set, state=tk.DISABLED, padx=10, pady=10)
        self.txt_verification.pack(fill=tk.BOTH, expand=True)
        verif_scroll.config(command=self.txt_verification.yview)

        # ------ Tab 3: Global Summary ------
        self.tab_summary = ttk.Frame(self.notebook, padding=5)
        self.notebook.add(self.tab_summary, text="📊 Global Summary")
        pw_summary = ttk.PanedWindow(self.tab_summary, orient=tk.VERTICAL)
        pw_summary.pack(fill=tk.BOTH, expand=True)

        frame_mods = ttk.Frame(pw_summary)
        ttk.Label(frame_mods, text="📦 Currently Enabled Modules:", font=("Helvetica", 9, "bold")).pack(anchor=tk.W)
        scroll_m = ttk.Scrollbar(frame_mods)
        scroll_m.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_summary_mods = tk.Listbox(frame_mods, yscrollcommand=scroll_m.set, bg="#fcfcfc", font=("Consolas", 10))
        self.list_summary_mods.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_m.config(command=self.list_summary_mods.yview)
        pw_summary.add(frame_mods, weight=1)

        frame_dirs = ttk.Frame(pw_summary)
        header_dirs = ttk.Frame(frame_dirs)
        header_dirs.pack(fill=tk.X, pady=(5, 2))
        ttk.Label(header_dirs, text="⚙️ Required Compiler Include Paths (-I):", font=("Helvetica", 9, "bold"), foreground="#cc6600").pack(side=tk.LEFT)
        ttk.Button(header_dirs, text="📋 Copy All", command=self.copy_inc_dirs).pack(side=tk.RIGHT)
        
        scroll_d = ttk.Scrollbar(frame_dirs)
        scroll_d.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_inc_dirs = tk.Listbox(frame_dirs, yscrollcommand=scroll_d.set, fg="#cc6600", font=("Consolas", 10, "bold"), bg="#fff9f0")
        self.list_inc_dirs.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_d.config(command=self.list_inc_dirs.yview)
        pw_summary.add(frame_dirs, weight=1)

        # ------ Tab 4: Physical Files Details ------
        self.tab_files = ttk.Frame(self.notebook, padding=5)
        self.notebook.add(self.tab_files, text="🗂️ Sync File Details")
        pw_files = ttk.PanedWindow(self.tab_files, orient=tk.VERTICAL)
        pw_files.pack(fill=tk.BOTH, expand=True)

        frame_src = ttk.Frame(pw_files)
        self.lbl_src_count = ttk.Label(frame_src, text="📄 Pending Source Files (.c / .cpp):", foreground="blue", font=("Helvetica", 9, "bold"))
        self.lbl_src_count.pack(anchor=tk.W)
        scroll_s = ttk.Scrollbar(frame_src)
        scroll_s.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_src = tk.Listbox(frame_src, yscrollcommand=scroll_s.set, fg="blue", font=("Consolas", 9), bg="#fcfcfc")
        self.list_src.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_s.config(command=self.list_src.yview)
        pw_files.add(frame_src, weight=1)

        frame_inc = ttk.Frame(pw_files)
        self.lbl_inc_count = ttk.Label(frame_inc, text="📁 Pending Header Files (.h / .hpp):", foreground="green", font=("Helvetica", 9, "bold"))
        self.lbl_inc_count.pack(anchor=tk.W)
        scroll_i = ttk.Scrollbar(frame_inc)
        scroll_i.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_inc = tk.Listbox(frame_inc, yscrollcommand=scroll_i.set, fg="green", font=("Consolas", 9), bg="#fcfcfc")
        self.list_inc.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll_i.config(command=self.list_inc.yview)
        pw_files.add(frame_inc, weight=1)

        # === Bottom Panel: Deployment Buttons ===
        bottom_frame = ttk.Frame(self.root, padding=(10, 5))
        bottom_frame.pack(fill=tk.X, side=tk.BOTTOM)
        
        # 1. Reverse Sync
        rev_frame = ttk.LabelFrame(bottom_frame, text="Reverse Sync (Push Local Changes to Core)", padding=5)
        rev_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 10))
        ttk.Button(rev_frame, text="⬆️ Reverse Sync Source", command=self.reverse_sync_src).pack(side=tk.LEFT, padx=5)
        ttk.Button(rev_frame, text="⬆️ Reverse Sync Headers", command=self.reverse_sync_inc).pack(side=tk.LEFT, padx=5)

        # 2. Forward Sync & Save
        fwd_frame = ttk.LabelFrame(bottom_frame, text="Configuration & Forward Deployment (Core to Local)", padding=5)
        fwd_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        ttk.Button(fwd_frame, text="💾 Save Config (Ctrl+S)", command=lambda: self.save_local_config(show_msg=True)).pack(side=tk.LEFT, padx=5)
        self.btn_sync_src = ttk.Button(fwd_frame, text="⬇️ Generate Source (Source)", command=lambda: self.save_and_run_script("src_only"))
        self.btn_sync_src.pack(side=tk.LEFT, padx=5)
        self.btn_sync_inc = ttk.Button(fwd_frame, text="⬇️ Generate Headers (Include)", command=lambda: self.save_and_run_script("inc_only"))
        self.btn_sync_inc.pack(side=tk.LEFT, padx=5)

    # ================= Dependency Engine =================
    def get_all_dependencies(self, root_name, mod_key):
        closure = set()
        queue = [f"{root_name}|{mod_key}"]
        while queue:
            current = queue.pop(0)
            if current in closure: continue
            closure.add(current)
            c_root, c_mod = current.split('|', 1)
            deps = self.registry.get("modules", {}).get(c_root, {}).get(c_mod, {}).get("depends_on", [])
            for dep in deps:
                if dep not in closure: queue.append(dep)
        closure.discard(f"{root_name}|{mod_key}")
        return closure

    def check_can_uncheck(self, root_name, mod_key):
        target = f"{root_name}|{mod_key}"
        conflicts = []
        for sel_mod in self.selected_modules:
            if sel_mod == target: continue
            s_root, s_mod = sel_mod.split('|', 1)
            deps = self.get_all_dependencies(s_root, s_mod)
            if target in deps: conflicts.append(sel_mod)

        if target.endswith("|_internal"):
            folder_prefix = target[:-10] 
            for sel_mod in self.selected_modules:
                if sel_mod != target and sel_mod.startswith(folder_prefix + "|"):
                    conflicts.append(f"{sel_mod} (Sub-modules require internal package)")
        return conflicts

    def set_module_checked(self, root_name, mod_key, state, silent=False):
        target = f"{root_name}|{mod_key}"
        if state: 
            if root_name == "csp":
                chip_family = mod_key.split('|')[0]
                to_uncheck = []
                for sel in self.selected_modules:
                    s_root, s_mod = sel.split('|', 1)
                    if s_root == "csp" and s_mod.split('|')[0] != chip_family:
                        to_uncheck.append((s_root, s_mod))
                for r, m in to_uncheck:
                    self.set_module_checked(r, m, False, silent=True)

            self.selected_modules.add(target)
            for dep in self.get_all_dependencies(root_name, mod_key):
                self.selected_modules.add(dep)
                
            parts = mod_key.split('|')
            for i in range(1, len(parts)):
                parent_internal = f"{'|'.join(parts[:i])}|_internal"
                if parent_internal in self.registry["modules"][root_name]:
                    self.selected_modules.add(f"{root_name}|{parent_internal}")
        else: 
            conflicts = self.check_can_uncheck(root_name, mod_key)
            if conflicts:
                if not silent:
                    conflict_str = "\n".join([f" - {c.replace('|', ' / ')}" for c in set(conflicts)])
                    messagebox.showwarning("Dependency Conflict", f"Cannot uncheck '{mod_key}'!\n\nReason:\n{conflict_str}")
                return False 
            else:
                self.selected_modules.discard(target)
        return True

    # ================= Tree Interaction =================
    def refresh_tree(self):
        for item in self.tree.get_children(): self.tree.delete(item)

        for root_name in self.registry.get("roots", []):
            root_id = f"ROOT|{root_name}"
            self.tree.insert("", tk.END, iid=root_id, text=f"📦 {root_name}", open=True)
            
            modules = self.registry.get("modules", {}).get(root_name, {})
            for mod_key in sorted(modules.keys()):
                mod_data = modules[mod_key]
                if mod_data.get("type") == "module":
                    parts = mod_key.split('|')
                    parent_id = root_id
                    
                    for i in range(len(parts) - 1):
                        folder_name = parts[i]
                        current_folder_path = "|".join(parts[:i+1])
                        folder_id = f"FOLDER|{root_name}|{current_folder_path}"
                        if not self.tree.exists(folder_id):
                            is_open = False if folder_name == "dev" else True
                            self.tree.insert(parent_id, tk.END, iid=folder_id, text=f"{UNCHECKED}📁 {folder_name}", open=is_open)
                        parent_id = folder_id

                    leaf_name = parts[-1]
                    mod_id = f"MOD|{root_name}|{mod_key}"
                    
                    icon, _ = self.get_module_status(mod_key, mod_data)
                    
                    if leaf_name == "_internal":
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{UNCHECKED}{icon} ⚙️ _internal")
                    else:
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{UNCHECKED}{icon} 📄 {leaf_name}")

        self.update_tree_checkboxes()

    def update_tree_checkboxes(self, node=""):
        children = self.tree.get_children(node)
        if not children:
            if node.startswith("MOD|"):
                _, root_name, mod_key = node.split('|', 2)
                is_checked = f"{root_name}|{mod_key}" in self.selected_modules
                base_text = self.tree.item(node, "text").replace(CHECKED, "").replace(UNCHECKED, "")
                self.tree.item(node, text=f"{CHECKED if is_checked else UNCHECKED}{base_text}")
                return is_checked
            return False

        all_checked, any_checked = True, False
        for child in children:
            if self.update_tree_checkboxes(child): any_checked = True
            else: all_checked = False

        if node and node.startswith("FOLDER|"):
            base_text = self.tree.item(node, "text").replace(CHECKED, "").replace(UNCHECKED, "").replace(PARTIAL, "")
            if all_checked: self.tree.item(node, text=f"{CHECKED}{base_text}")
            elif any_checked: self.tree.item(node, text=f"{PARTIAL}{base_text}")
            else: self.tree.item(node, text=f"{UNCHECKED}{base_text}")

        return all_checked

    def on_tree_double_click(self, event):
        region = self.tree.identify("region", event.x, event.y)
        if region not in ("cell", "tree"): return "break"
        
        item_id = self.tree.focus()
        if not item_id or item_id.startswith("ROOT|"): return "break"

        if item_id.startswith("MOD|"):
            _, root_name, mod_key = item_id.split('|', 2)
            target_state = False if CHECKED in self.tree.item(item_id, "text") else True
            self.set_module_checked(root_name, mod_key, target_state)
            
        elif item_id.startswith("FOLDER|"):
            _, root_name, folder_path = item_id.split('|', 2)
            modules_in_folder = [m for m in self.registry["modules"][root_name].keys() if m.startswith(folder_path + '|')]
            internal_key = f"{folder_path}|_internal"
            has_internal = internal_key in modules_in_folder
            
            selected_in_folder = [m for m in modules_in_folder if f"{root_name}|{m}" in self.selected_modules]
            S, N = len(selected_in_folder), len(modules_in_folder)

            if S == N and N > 0:
                for m in modules_in_folder:
                    if m != internal_key: self.set_module_checked(root_name, m, False, silent=True)
                if has_internal: self.set_module_checked(root_name, internal_key, True, silent=True)
            elif has_internal and S == 1 and selected_in_folder[0] == internal_key:
                self.set_module_checked(root_name, internal_key, False, silent=True)
            else:
                for m in modules_in_folder: self.set_module_checked(root_name, m, True, silent=True)

        self.update_tree_checkboxes()
        self.update_dashboard()
        self.on_tree_select(None)
        return "break" 

    # ================= Node Perspective & Reporting =================
    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        
        self.btn_open_help.config(state=tk.DISABLED)
        self.clear_detail_tab()
        
        if node_id.startswith("ROOT|"):
            self.current_selected_node = None
            root_name = node_id.split('|')[1]
            self.lbl_node_name.config(text=f"📦 {root_name}")
            
            sub_count = 0
            all_src_pat, all_inc_pat, all_dirs_pat, all_deps = set(), set(), set(), set()
            for sel in self.selected_modules:
                r, m = sel.split('|', 1)
                if r == root_name:
                    sub_count += 1
                    mod_data = self.registry["modules"][r][m]
                    all_src_pat.update(mod_data.get("src_patterns", []))
                    all_inc_pat.update(mod_data.get("inc_patterns", []))
                    all_dirs_pat.update(mod_data.get("inc_dirs", []))
                    all_deps.update(mod_data.get("depends_on", []))
                    
            self.lbl_node_desc.config(text=f"System Root Directory (Currently {sub_count} sub-modules enabled)")
            
            self.fill_detail_box(self.txt_det_src, self.resolve_paths(list(all_src_pat)))
            self.fill_detail_box(self.txt_det_inc, self.resolve_paths(list(all_inc_pat)))
            self.fill_detail_box(self.txt_det_dirs, self.resolve_paths(list(all_dirs_pat), is_dir_mode=True))
            self.fill_detail_box(self.txt_det_deps, sorted(list(all_deps)))

            self.render_verification_report(None, None, None)

        elif node_id.startswith("FOLDER|"):
            self.current_selected_node = None
            _, root_name, folder_path = node_id.split('|', 2)
            self.lbl_node_name.config(text=f"📁 {root_name} | {folder_path.replace('|', ' / ')}")
            
            sub_count = 0
            all_src_pat, all_inc_pat, all_dirs_pat, all_deps = set(), set(), set(), set()
            for sel in self.selected_modules:
                r, m = sel.split('|', 1)
                if r == root_name and m.startswith(folder_path + '|'):
                    sub_count += 1
                    mod_data = self.registry["modules"][r][m]
                    all_src_pat.update(mod_data.get("src_patterns", []))
                    all_inc_pat.update(mod_data.get("inc_patterns", []))
                    all_dirs_pat.update(mod_data.get("inc_dirs", []))
                    all_deps.update(mod_data.get("depends_on", []))
                    
            self.lbl_node_desc.config(text=f"Package Folder (Currently {sub_count} sub-modules enabled)")
            
            self.fill_detail_box(self.txt_det_src, self.resolve_paths(list(all_src_pat)))
            self.fill_detail_box(self.txt_det_inc, self.resolve_paths(list(all_inc_pat)))
            self.fill_detail_box(self.txt_det_dirs, self.resolve_paths(list(all_dirs_pat), is_dir_mode=True))
            self.fill_detail_box(self.txt_det_deps, sorted(list(all_deps)))
            
            self.render_verification_report(None, None, None)
            
        elif node_id.startswith("MOD|"):
            _, r, m = node_id.split('|', 2)
            self.current_selected_node = (r, m)
            mod_data = self.registry["modules"][r][m]
            
            icon, status_text = self.get_module_status(m, mod_data)
            
            self.lbl_node_name.config(text=f"{icon} {m.replace('|', ' / ')}  [{status_text}]")
            self.lbl_node_desc.config(text=mod_data.get("description", "No description available."))
            
            if mod_data.get("help_docs"):
                self.btn_open_help.config(state=tk.NORMAL)
                
            self.fill_detail_box(self.txt_det_src, self.resolve_paths(mod_data.get("src_patterns", [])))
            self.fill_detail_box(self.txt_det_inc, self.resolve_paths(mod_data.get("inc_patterns", [])))
            self.fill_detail_box(self.txt_det_dirs, self.resolve_paths(mod_data.get("inc_dirs", []), is_dir_mode=True))
            self.fill_detail_box(self.txt_det_deps, mod_data.get("depends_on", []))

            self.render_verification_report(r, m, mod_data)

    def render_verification_report(self, root_name, mod_key, mod_data):
        self.txt_verification.config(state=tk.NORMAL)
        self.txt_verification.delete(1.0, tk.END)

        if not mod_key:
            self.txt_verification.insert(tk.END, "Please select a specific leaf module to view its Verification Report.")
            self.txt_verification.config(state=tk.DISABLED)
            return

        icon, status = self.get_module_status(mod_key, mod_data)
        
        self.txt_verification.insert(tk.END, f"📦 Target Module: {root_name} | {mod_key.replace('|', ' / ')}\n")
        self.txt_verification.insert(tk.END, f"🛡️ Trust Rating : {icon} {status}\n")
        self.txt_verification.insert(tk.END, "="*65 + "\n\n")

        if mod_key.endswith("_internal"):
            self.txt_verification.insert(tk.END, "ℹ️ Internal packages are implicitly verified and highly stable.\n")
            self.txt_verification.insert(tk.END, "   (No manual verification checklists are required for this node.)\n")
        else:
            tasks = mod_data.get("tasks", [])
            if not tasks:
                self.txt_verification.insert(tk.END, "⚠️ No verification tracking data available for this module yet.\n")
            else:
                self.txt_verification.insert(tk.END, "--- Verification Checklist & Notes ---\n\n")
                for task in tasks:
                    check_mark = "☑" if task.get("completed") else "☐"
                    date_str = f" ({task.get('date')})" if task.get("date") else ""
                    self.txt_verification.insert(tk.END, f"{check_mark} {task.get('summary')}{date_str}\n")

                    reply = task.get("reply", "")
                    safe_mod = mod_key.replace('|', '_')
                    md_path = self.dev_notes_dir / f"{root_name}___{safe_mod}___{task.get('id')}.md"
                    
                    if md_path.exists():
                        try:
                            with open(md_path, 'r', encoding='utf-8') as f:
                                reply = f.read()
                        except Exception:
                            pass

                    if reply.strip():
                        self.txt_verification.insert(tk.END, "    Notes:\n")
                        indented_reply = "\n".join([f"    > {line}" for line in reply.strip().split('\n')])
                        self.txt_verification.insert(tk.END, f"{indented_reply}\n")
                    else:
                        self.txt_verification.insert(tk.END, "    (No notes provided)\n")
                        
                    self.txt_verification.insert(tk.END, "\n" + "-"*65 + "\n\n")

        self.txt_verification.config(state=tk.DISABLED)

    def fill_detail_box(self, txt_widget, data_list):
        txt_widget.config(state=tk.NORMAL)
        txt_widget.delete(1.0, tk.END)
        if data_list:
            txt_widget.insert(tk.END, "\n".join(data_list))
        else:
            txt_widget.insert(tk.END, "(None / Not Configured)")
        txt_widget.config(state=tk.DISABLED)

    def clear_detail_tab(self):
        for txt in (self.txt_det_src, self.txt_det_inc, self.txt_det_dirs, self.txt_det_deps):
            self.fill_detail_box(txt, [])

    def open_help_docs(self):
        if not self.current_selected_node: return
        r, m = self.current_selected_node
        docs = self.registry["modules"][r][m].get("help_docs", [])
        
        abs_paths = self.resolve_paths(docs, return_absolute=True)
        if not abs_paths:
            messagebox.showwarning("Warning", "Documentation not found! Please check path configurations.")
            return
            
        for p in abs_paths:
            try: os.startfile(p)
            except Exception as e: print(f"Failed to open {p}: {e}")

    # ================= Dashboard & Globals =================
    def update_dashboard(self):
        self.list_summary_mods.delete(0, tk.END)
        self.list_inc_dirs.delete(0, tk.END)
        self.list_src.delete(0, tk.END)
        self.list_inc.delete(0, tk.END)

        if not self.selected_modules:
            self.notebook.tab(2, text="📊 Global Summary (0)")
            self.notebook.tab(3, text="🗂️ Sync File Details (C:0 | H:0)")
            return

        for sel in sorted(list(self.selected_modules)):
            self.list_summary_mods.insert(tk.END, f"  ✅ {sel.replace('|', ' / ')}")

        all_src_patterns, all_inc_patterns, all_inc_dirs_patterns = [], [], []

        for sel in self.selected_modules:
            r, m = sel.split('|', 1)
            mod_data = self.registry["modules"][r][m]
            all_src_patterns.extend(mod_data.get("src_patterns", []))
            all_inc_patterns.extend(mod_data.get("inc_patterns", []))
            all_inc_dirs_patterns.extend(mod_data.get("inc_dirs", []))

        dirs_files = self.resolve_paths(all_inc_dirs_patterns, is_dir_mode=True)
        for d in dirs_files: self.list_inc_dirs.insert(tk.END, d)

        src_files = self.resolve_paths(all_src_patterns)
        for f in src_files: self.list_src.insert(tk.END, f)
        
        inc_files = self.resolve_paths(all_inc_patterns)
        for f in inc_files: self.list_inc.insert(tk.END, f)
        
        self.notebook.tab(2, text=f"📊 Global Summary (Modules:{len(self.selected_modules)})")
        self.notebook.tab(3, text=f"🗂️ Sync File Details (C:{len(src_files)} | H:{len(inc_files)})")
        
        self.lbl_src_count.config(text=f"📄 Pending Source Files (.c / .cpp) - Total {len(src_files)}:")
        self.lbl_inc_count.config(text=f"📁 Pending Header Files (.h / .hpp) - Total {len(inc_files)}:")

    def copy_inc_dirs(self):
        dirs = self.list_inc_dirs.get(0, tk.END)
        if not dirs:
            messagebox.showinfo("Info", "No paths to copy!")
            return
        self.root.clipboard_clear()
        self.root.clipboard_append("\n".join(dirs))
        messagebox.showinfo("Copied", f"Copied {len(dirs)} paths to clipboard!\nPaste them directly into your IDE's Include Paths.")

    # ================= Save & Deploy =================
    def save_local_config(self, sync_mode="all", show_msg=False):
        config_data = {
            "sync_mode": sync_mode,
            "selected_modules": []
        }
        for sel in sorted(list(self.selected_modules)):
            root_name, mod_key = sel.split('|', 1)
            config_data["selected_modules"].append({"root": root_name, "module": mod_key})
            
        try:
            with open(self.local_config_path, 'w', encoding='utf-8') as f:
                json.dump(config_data, f, indent=4, ensure_ascii=False)
            if show_msg:
                messagebox.showinfo("Saved", "Configuration successfully saved!\n(You can use Ctrl+S to quick save)")
            return True
        except Exception as e:
            messagebox.showerror("Save Failed", f"Cannot write config file:\n{e}")
            return False

    def save_and_run_script(self, mode):
        if not self.save_local_config(sync_mode=mode, show_msg=False):
            return

        script_dir = Path(__file__).parent.resolve()
        
        if mode == "src_only":
            target_script = script_dir / "framework_sync_src_v3.py"
            title = "Source Sync Console"
        else:
            target_script = script_dir / "framework_sync_inc_v3.py"
            title = "Header Sync Console"

        if not target_script.exists():
            messagebox.showerror("Missing Tool", f"Cannot find background script:\n{target_script}")
            return

        self.execute_script_live(target_script, title)

    def execute_script_live(self, script_path, title):
        top = tk.Toplevel(self.root)
        top.title(title)
        top.geometry("850x550")
        top.grab_set() 
        
        txt = tk.Text(top, font=("Consolas", 10), bg="#1e1e1e", fg="#d4d4d4")
        txt.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        txt.insert(tk.END, f">>> Booting Deployment Engine...\n")
        txt.insert(tk.END, f">>> Script: {script_path.name}\n\n")
        top.update_idletasks()

        creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0
        
        try:
            process = subprocess.Popen(
                [sys.executable, str(script_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=os.getcwd(),
                creationflags=creationflags,
                encoding='utf-8',
                errors='replace'
            )
            
            for line in iter(process.stdout.readline, ''):
                txt.insert(tk.END, line)
                txt.see(tk.END)
                top.update_idletasks()
                
            process.stdout.close()
            process.wait()
            
            if process.returncode == 0:
                txt.insert(tk.END, "\n>>> [✔] Process completed successfully. You may close this window.")
            else:
                txt.insert(tk.END, f"\n>>> [❌] Engine aborted with Error Code: {process.returncode}")
                
        except Exception as e:
            txt.insert(tk.END, f"\n>>> [FATAL] Failed to launch backend engine:\n{e}\n")
            
        txt.config(state=tk.DISABLED)

    # ================= Reverse Sync Guard =================
    def reverse_sync_src(self):
        src_dir = Path(os.getcwd()) / "gmp_src"
        if not src_dir.exists() or not any(src_dir.iterdir()):
            messagebox.showerror("Denied", "Local 'gmp_src' directory is missing or empty.\nYou must generate source files and make edits before pushing.")
            return
            
        script_path = Path(__file__).parent.resolve() / "framework_reverse_sync_src.py"
        self.execute_reverse_script_flow(script_path, "Reverse Sync [Source] Audit")

    def reverse_sync_inc(self):
        inc_dir = Path(os.getcwd()) / "gmp_inc"
        if not inc_dir.exists() or not any(inc_dir.iterdir()):
            messagebox.showerror("Denied", "Local 'gmp_inc' directory is missing or empty.\nYou must generate header files and make edits before pushing.")
            return
            
        script_path = Path(__file__).parent.resolve() / "framework_reverse_sync_inc.py"
        self.execute_reverse_script_flow(script_path, "Reverse Sync [Header] Audit")

    def execute_reverse_script_flow(self, script_path, title):
        if not script_path.exists():
            messagebox.showerror("Missing Tool", f"Cannot find reverse sync engine:\n{script_path}")
            return
            
        top = tk.Toplevel(self.root)
        top.title(title)
        top.geometry("900x600")
        top.grab_set() 
        
        txt = tk.Text(top, font=("Consolas", 10), bg="#1e1e1e", fg="#d4d4d4")
        txt.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        top.update_idletasks()

        creationflags = subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0

        try:
            result = subprocess.run(
                [sys.executable, str(script_path), "--diff"],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                cwd=os.getcwd(),
                creationflags=creationflags,
                encoding='utf-8',
                errors='replace'
            )
            
            output = result.stdout
            txt.insert(tk.END, output)
            txt.see(tk.END)
            top.update_idletasks()
            
            if "[MODIFIED]" in output:
                txt.insert(tk.END, "\n\n⚠️ Awaiting User Confirmation...\n")
                top.update_idletasks()
                
                confirm = messagebox.askyesno(
                    "Danger Zone: Confirm Overwrite", 
                    "Modified files have been detected (see console).\n\n"
                    "Are you absolutely sure you want to push these changes back to the GMP Core Library?\n"
                    "WARNING: This operation is irreversible!", 
                    parent=top, icon='warning'
                )
                
                if confirm:
                    txt.insert(tk.END, "\n>>> Authorization granted. Triggering core overwrite protocol...\n")
                    top.update_idletasks()
                    
                    res_apply = subprocess.run(
                        [sys.executable, str(script_path), "--apply"],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                        text=True,
                        cwd=os.getcwd(),
                        creationflags=creationflags,
                        encoding='utf-8',
                        errors='replace'
                    )
                    txt.insert(tk.END, res_apply.stdout)
                else:
                    txt.insert(tk.END, "\n>>> [ABORT] User cancelled overwrite. Core library remains safe.")
            else:
                txt.insert(tk.END, "\n>>> [INFO] No committable changes detected.")

        except Exception as e:
            txt.insert(tk.END, f"\n[FATAL] Engine crash: {e}")
            
        txt.config(state=tk.DISABLED)

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkUserGUI(root)
    root.mainloop()