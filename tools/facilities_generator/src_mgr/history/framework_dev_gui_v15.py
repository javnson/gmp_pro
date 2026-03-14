import os
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
from pathlib import Path

# Import our customized data model and diagnostic tools
from framework_data_model_v3 import FrameworkDataModel
import framework_validator
import framework_orphan_checker_v4 as framework_orphan_checker

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP Framework Asset Manager V15 (DevOps & English UI)")
        self.root.geometry("1200x900")
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.bind("<Control-s>", self.save_json_to_disk)
        self.root.bind("<Control-S>", self.save_json_to_disk)

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("Error", "Environment variable GMP_PRO_LOCATION not found!")
            self.root.destroy()
            return
            
        json_path = Path(__file__).parent / "gmp_framework_dic.json"
        
        self.model = FrameworkDataModel(self.gmp_location, json_path)
        self.registry = self.model.registry 
        self.current_selected_node = None 
        self.all_available_modules = [] 
        self.is_dirty = False 
        
        self.build_ui()
        self.load_data()

    def mark_dirty(self):
        self.is_dirty = True
        self.root.title("GMP Framework Asset Manager V15 - *Unsaved Changes*")
        self.btn_global_save.config(state=tk.NORMAL)

    def mark_clean(self):
        self.is_dirty = False
        self.root.title("GMP Framework Asset Manager V15 (DevOps & English UI)")
        self.btn_global_save.config(state=tk.DISABLED)

    def on_closing(self):
        if self.is_dirty:
            if messagebox.askyesno("Exit Confirmation", "You have unsaved changes. Force exit will lose them.\nExit anyway?"):
                self.root.destroy()
        else:
            self.root.destroy()

    def build_ui(self):
        # ========== Top Toolbar ==========
        toolbar = ttk.Frame(self.root, padding=5, relief=tk.RAISED)
        toolbar.pack(fill=tk.X)
        self.btn_global_save = ttk.Button(toolbar, text="💾 Save to Disk (Ctrl+S)", command=self.save_json_to_disk, state=tk.DISABLED)
        self.btn_global_save.pack(side=tk.LEFT, padx=5, ipadx=10, ipady=3)
        ttk.Button(toolbar, text="🔄 Discard & Reload", command=self.load_data).pack(side=tk.LEFT, padx=5, ipadx=10, ipady=3)

        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ========== Left Tree Panel ==========
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)
        
        tree_frame = ttk.Frame(left_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        tree_scroll = ttk.Scrollbar(tree_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        self.tree.heading("#0", text="Framework Repository Tree", anchor=tk.W)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        # Basic Operations
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="➕ Add Module", command=self.add_module).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="❌ Delete Node", command=self.delete_node).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        
        ttk.Button(left_frame, text="⚙️ Path Macros Manager", command=self.open_macro_manager).pack(fill=tk.X, pady=2)

        # Diagnostic Tools
        tool_frame = ttk.LabelFrame(left_frame, text="🛠️ Diagnostic Tools", padding=5)
        tool_frame.pack(fill=tk.X, pady=5)
        ttk.Button(tool_frame, text="🔍 Sanity Check (Broken Paths)", command=self.run_validation_tool).pack(fill=tk.X, pady=2)
        ttk.Button(tool_frame, text="👻 Orphan Scan (Untracked Files)", command=self.run_orphan_tool).pack(fill=tk.X, pady=2)

        # Legend Panel
        legend_frame = ttk.LabelFrame(left_frame, text="Status Legend", padding=5)
        legend_frame.pack(fill=tk.X, pady=5)
        ttk.Label(legend_frame, text="🟢 : Ready for Prod (HW Verified)").pack(anchor=tk.W)
        ttk.Label(legend_frame, text="🔷 : Sim/Principle Verified").pack(anchor=tk.W)
        ttk.Label(legend_frame, text="🟨 : Compilation Passed").pack(anchor=tk.W)
        ttk.Label(legend_frame, text="❌ : In Development (Unverified)").pack(anchor=tk.W)

        # ========== Right Tab Panel ==========
        right_frame = ttk.Frame(self.paned_window, padding=5)
        self.paned_window.add(right_frame, weight=3)

        top_info_frame = ttk.Frame(right_frame)
        top_info_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(top_info_frame, text="Selected Node:").pack(side=tk.LEFT)
        self.lbl_module_id = ttk.Label(top_info_frame, font=("Helvetica", 11, "bold"), foreground="blue", text="-")
        self.lbl_module_id.pack(side=tk.LEFT, padx=10)

        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

        # ---------- Tab 1: Rules Config ----------
        self.tab_config = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_config, text="⚙️ Rules Config")
        ttk.Label(self.tab_config, text="Description:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.entry_desc = ttk.Entry(self.tab_config, width=50)
        self.entry_desc.grid(row=0, column=1, sticky=tk.EW, pady=5)

        def create_scrolled_path_editor(row, label_text, is_help_doc=False):
            lbl_frame = ttk.Frame(self.tab_config)
            lbl_frame.grid(row=row, column=0, sticky=tk.NW, pady=5)
            ttk.Label(lbl_frame, text=label_text).pack(anchor=tk.W)
            
            txt_frame = ttk.Frame(self.tab_config)
            txt_frame.grid(row=row, column=1, sticky=tk.EW, pady=5)
            
            scroll = ttk.Scrollbar(txt_frame)
            scroll.pack(side=tk.RIGHT, fill=tk.Y)
            txt_widget = tk.Text(txt_frame, height=4, width=40, yscrollcommand=scroll.set)
            txt_widget.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
            scroll.config(command=txt_widget.yview)
            
            btn_frame_sub = ttk.Frame(lbl_frame)
            btn_frame_sub.pack(anchor=tk.W, pady=2)
            ttk.Button(btn_frame_sub, text="📄 File", width=6, command=lambda: self.browse_path(txt_widget, is_dir=False)).pack(side=tk.LEFT, padx=(0,2))
            ttk.Button(btn_frame_sub, text="📁 Dir", width=6, command=lambda: self.browse_path(txt_widget, is_dir=True)).pack(side=tk.LEFT)
            ttk.Button(lbl_frame, text="📌 Insert Macro", command=lambda event=None, w=txt_widget: self.popup_macro_menu(event, w)).pack(anchor=tk.W, fill=tk.X, pady=2)
            
            if is_help_doc:
                ttk.Button(lbl_frame, text="🌐 Open in Windows", command=lambda: self.open_help_docs(txt_widget)).pack(anchor=tk.W, fill=tk.X, pady=2)
            
            return txt_widget

        self.txt_src = create_scrolled_path_editor(1, "Source Files (src_patterns):")
        self.txt_inc = create_scrolled_path_editor(2, "Header Files (inc_patterns):")
        self.txt_inc_dirs = create_scrolled_path_editor(3, "Include Dirs (inc_dirs):")
        self.txt_help_docs = create_scrolled_path_editor(4, "Help Docs (help_docs):\n(PDF/HTML/MD)", is_help_doc=True)

        ttk.Label(self.tab_config, text="Dependencies (depends_on):\n(Double-click to toggle)").grid(row=5, column=0, sticky=tk.NW, pady=5)
        dep_frame = ttk.Frame(self.tab_config)
        dep_frame.grid(row=5, column=1, sticky=tk.EW, pady=5)
        dep_scroll = ttk.Scrollbar(dep_frame)
        dep_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree_deps = ttk.Treeview(dep_frame, yscrollcommand=dep_scroll.set, selectmode="browse", height=6, show="tree")
        self.tree_deps.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dep_scroll.config(command=self.tree_deps.yview)
        self.tree_deps.bind("<Double-1>", self.on_dep_tree_click)
        self.tree_deps.tag_configure("error", foreground="#d32f2f", font=("Helvetica", 9, "bold"))

        self.tab_config.rowconfigure(5, weight=1) 
        self.tab_config.columnconfigure(1, weight=1)
        self.btn_apply = ttk.Button(self.tab_config, text="✔️ Apply to Memory (Auto-saves on switch)", command=lambda: self.apply_current_module(show_msg=True))
        self.btn_apply.grid(row=6, column=1, sticky=tk.E, pady=10)

        # ---------- Tab 2: Live Preview ----------
        self.tab_preview = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_preview, text="👁️ Live Preview")
        pw_preview = ttk.PanedWindow(self.tab_preview, orient=tk.VERTICAL)
        pw_preview.pack(fill=tk.BOTH, expand=True)

        def create_preview_box(parent, text, color):
            frame = ttk.Frame(parent)
            ttk.Label(frame, text=text, foreground=color).pack(anchor=tk.W)
            scroll = ttk.Scrollbar(frame)
            scroll.pack(side=tk.RIGHT, fill=tk.Y)
            lb = tk.Listbox(frame, height=4, bg="#f9f9f9", yscrollcommand=scroll.set)
            lb.pack(fill=tk.BOTH, expand=True)
            scroll.config(command=lb.yview)
            parent.add(frame, weight=1)
            return lb

        self.list_preview_src = create_preview_box(pw_preview, "Resolved Source Files:", "blue")
        self.list_preview_inc = create_preview_box(pw_preview, "Resolved Header Files:", "green")
        self.list_preview_inc_dirs = create_preview_box(pw_preview, "Resolved Include Dirs (-I):", "#cc6600")

        # ---------- Tab 3: Verification & DevOps Tracking (NEW) ----------
        self.tab_tasks = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_tasks, text="✅ Verification Tracking")
        
        pw_tasks = ttk.PanedWindow(self.tab_tasks, orient=tk.HORIZONTAL)
        pw_tasks.pack(fill=tk.BOTH, expand=True)
        
        # Tasks List (Left)
        task_list_frame = ttk.LabelFrame(pw_tasks, text="Verification Checklist", padding=5)
        pw_tasks.add(task_list_frame, weight=1)
        
        task_scroll = ttk.Scrollbar(task_list_frame)
        task_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.list_tasks = tk.Listbox(task_list_frame, yscrollcommand=task_scroll.set, font=("Helvetica", 10))
        self.list_tasks.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        task_scroll.config(command=self.list_tasks.yview)
        self.list_tasks.bind("<<ListboxSelect>>", self.on_task_select)
        
        task_btn_frame = ttk.Frame(task_list_frame)
        task_btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(task_btn_frame, text="➕ Add Custom", command=self.add_custom_task).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)
        ttk.Button(task_btn_frame, text="❌ Delete", command=self.delete_custom_task).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=1)

        # Task Editor (Right)
        task_edit_frame = ttk.LabelFrame(pw_tasks, text="Task Details Editor", padding=10)
        pw_tasks.add(task_edit_frame, weight=2)
        
        self.var_task_done = tk.BooleanVar()
        self.chk_task_done = ttk.Checkbutton(task_edit_frame, text="Completed (Passed)", variable=self.var_task_done, style="TCheckbutton")
        self.chk_task_done.grid(row=0, column=0, columnspan=2, sticky=tk.W, pady=(0,10))
        
        ttk.Label(task_edit_frame, text="Summary:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.entry_task_summary = ttk.Entry(task_edit_frame, width=40)
        self.entry_task_summary.grid(row=1, column=1, sticky=tk.EW, pady=5)
        
        ttk.Label(task_edit_frame, text="Date (YYYY-MM-DD):").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.entry_task_date = ttk.Entry(task_edit_frame, width=20)
        self.entry_task_date.grid(row=2, column=1, sticky=tk.W, pady=5)
        
        ttk.Label(task_edit_frame, text="Reply/Notes:").grid(row=3, column=0, sticky=tk.NW, pady=5)
        reply_scroll = ttk.Scrollbar(task_edit_frame)
        reply_scroll.grid(row=3, column=2, sticky=tk.NS, pady=5)
        self.txt_task_reply = tk.Text(task_edit_frame, height=8, width=40, yscrollcommand=reply_scroll.set)
        self.txt_task_reply.grid(row=3, column=1, sticky=tk.EW, pady=5)
        reply_scroll.config(command=self.txt_task_reply.yview)
        
        task_edit_frame.rowconfigure(3, weight=1)
        task_edit_frame.columnconfigure(1, weight=1)
        
        ttk.Button(task_edit_frame, text="✔️ Save Task Changes", command=self.save_selected_task).grid(row=4, column=1, sticky=tk.E, pady=10)

        self.current_tasks_buffer = [] # Memory buffer for editing tasks

        self.clear_right_panel() 

    # ================= Core Data Logic =================
    def load_data(self):
        try:
            self.model.load()
        except Exception as e:
            messagebox.showerror("Read Error", f"Parse failed: {e}")
            return
        self.refresh_tree()
        self.mark_clean()

    def update_available_modules(self):
        self.all_available_modules = []
        for root_name in self.registry.get("roots", []):
            for mod_key in self.registry.get("modules", {}).get(root_name, {}).keys():
                self.all_available_modules.append(f"{root_name}|{mod_key}")
        self.all_available_modules.sort()

    def refresh_tree(self):
        for item in self.tree.get_children(): self.tree.delete(item)
        self.update_available_modules()

        for root_name in self.registry.get("roots", []):
            root_id = f"ROOT|{root_name}"
            self.tree.insert("", tk.END, iid=root_id, text=f"📦 {root_name}", open=True)
            
            modules = self.registry.get("modules", {}).get(root_name, {})
            sorted_keys = sorted(modules.keys())
            
            for mod_key in sorted_keys:
                mod_data = modules[mod_key]
                if mod_data.get("type") == "module":
                    parts = mod_key.split('|')
                    parent_id = root_id
                    
                    for i in range(len(parts) - 1):
                        folder_name = parts[i]
                        current_folder_path = "|".join(parts[:i+1])
                        folder_id = f"FOLDER|{root_name}|{current_folder_path}"
                        if not self.tree.exists(folder_id):
                            self.tree.insert(parent_id, tk.END, iid=folder_id, text=f"📁 {folder_name}", open=True)
                        parent_id = folder_id

                    leaf_name = parts[-1]
                    mod_id = f"MOD|{root_name}|{mod_key}"
                    
                    # Fetch dynamic status icon
                    icon, _ = self.model.get_module_status(mod_data)
                    
                    if leaf_name == "_internal":
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{icon} ⚙️ _internal", tags=("internal",))
                    else:
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"{icon} 📄 {leaf_name}")
        self.tree.tag_configure("internal", foreground="#0066cc")

    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        
        if self.current_selected_node:
            self.apply_current_module(show_msg=False)
        
        if node_id.startswith("ROOT|"):
            self.clear_right_panel()
            return
            
        elif node_id.startswith("FOLDER|"):
            _, root_name, folder_path = node_id.split('|', 2)
            self.current_selected_node = None 
            self.set_right_panel_state(tk.NORMAL)
            self.lbl_module_id.config(text=f"📁 {root_name} | {folder_path.replace('|', ' / ')}  (Folder Summary)")
            
            sub_count = 0
            all_src, all_inc, all_inc_dirs, all_help, all_deps = set(), set(), set(), set(), set()
            for k, v in self.registry["modules"][root_name].items():
                if k.startswith(folder_path + '|'):
                    sub_count += 1
                    all_src.update(v.get("src_patterns", []))
                    all_inc.update(v.get("inc_patterns", []))
                    all_inc_dirs.update(v.get("inc_dirs", []))
                    all_help.update(v.get("help_docs", []))
                    all_deps.update(v.get("depends_on", []))
            
            self.entry_desc.delete(0, tk.END)
            self.entry_desc.insert(0, f"Contains {sub_count} independent modules.")
            self.txt_src.delete(1.0, tk.END)
            self.txt_src.insert(tk.END, "\n".join(sorted(list(all_src))))
            self.txt_inc.delete(1.0, tk.END)
            self.txt_inc.insert(tk.END, "\n".join(sorted(list(all_inc))))
            self.txt_inc_dirs.delete(1.0, tk.END)
            self.txt_inc_dirs.insert(tk.END, "\n".join(sorted(list(all_inc_dirs))))
            self.txt_help_docs.delete(1.0, tk.END)
            self.txt_help_docs.insert(tk.END, "\n".join(sorted(list(all_help))))
            
            for item in self.tree_deps.get_children(): self.tree_deps.delete(item)
            
            self.entry_desc.config(state=tk.DISABLED)
            for txt in (self.txt_src, self.txt_inc, self.txt_inc_dirs, self.txt_help_docs):
                txt.config(state=tk.DISABLED)
            self.btn_apply.config(state=tk.DISABLED)
            
            self.list_tasks.delete(0, tk.END)
            self.list_tasks.insert(tk.END, "Please select a specific module to view tasks.")
            self.set_task_editor_state(tk.DISABLED)
            
            if self.notebook.index(self.notebook.select()) == 1:
                self.update_preview()
            return
            
        _, root_name, mod_key = node_id.split('|', 2)
        self.current_selected_node = (root_name, mod_key)
        mod_data = self.registry["modules"][root_name][mod_key]

        icon, status_text = self.model.get_module_status(mod_data)
        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text=f"{icon} {root_name} | {mod_key.replace('|', ' / ')}   [{status_text}]")
        
        self.entry_desc.delete(0, tk.END)
        self.entry_desc.insert(0, mod_data.get("description", ""))
        self.txt_src.delete(1.0, tk.END)
        self.txt_src.insert(tk.END, "\n".join(mod_data.get("src_patterns", [])))
        self.txt_inc.delete(1.0, tk.END)
        self.txt_inc.insert(tk.END, "\n".join(mod_data.get("inc_patterns", [])))
        self.txt_inc_dirs.delete(1.0, tk.END)
        self.txt_inc_dirs.insert(tk.END, "\n".join(mod_data.get("inc_dirs", [])))
        self.txt_help_docs.delete(1.0, tk.END)
        self.txt_help_docs.insert(tk.END, "\n".join(mod_data.get("help_docs", [])))

        self.refresh_dep_tree(mod_data.get("depends_on", []), root_name, mod_key)
        
        # Load tasks into buffer and render
        self.current_tasks_buffer = copy.deepcopy(mod_data.get("tasks", []))
        self.refresh_task_list()

        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    # ================= DevOps Tracking (Tab 3) Logic =================
    def refresh_task_list(self):
        self.list_tasks.delete(0, tk.END)
        for idx, task in enumerate(self.current_tasks_buffer):
            prefix = "☑" if task.get("completed") else "☐"
            disp_text = f"{prefix} {task.get('summary', 'Unknown Task')}"
            self.list_tasks.insert(tk.END, disp_text)
            # Add subtle color coding
            if task.get("id", "").startswith("sys_"):
                self.list_tasks.itemconfig(idx, {'fg': '#004d99'}) # Dark blue for system tasks
                
        self.set_task_editor_state(tk.DISABLED)

    def on_task_select(self, event):
        selection = self.list_tasks.curselection()
        if not selection: return
        idx = selection[0]
        task = self.current_tasks_buffer[idx]
        
        self.set_task_editor_state(tk.NORMAL)
        
        self.var_task_done.set(task.get("completed", False))
        self.entry_task_summary.delete(0, tk.END)
        self.entry_task_summary.insert(0, task.get("summary", ""))
        self.entry_task_date.delete(0, tk.END)
        self.entry_task_date.insert(0, task.get("date", ""))
        self.txt_task_reply.delete(1.0, tk.END)
        self.txt_task_reply.insert(tk.END, task.get("reply", ""))
        
        # Lock summary modification for mandatory system tasks
        if task.get("id", "").startswith("sys_"):
            self.entry_task_summary.config(state="readonly")
            
    def set_task_editor_state(self, state):
        self.chk_task_done.config(state=state)
        self.entry_task_summary.config(state=state)
        self.entry_task_date.config(state=state)
        self.txt_task_reply.config(state=state)

    def save_selected_task(self):
        selection = self.list_tasks.curselection()
        if not selection: return
        idx = selection[0]
        
        task = self.current_tasks_buffer[idx]
        task["completed"] = self.var_task_done.get()
        if not task.get("id", "").startswith("sys_"):
            task["summary"] = self.entry_task_summary.get().strip()
            
        # Auto fill date if completed and empty
        current_date_input = self.entry_task_date.get().strip()
        if task["completed"] and not current_date_input:
            from datetime import date
            current_date_input = str(date.today())
            self.entry_task_date.insert(0, current_date_input)
            
        task["date"] = current_date_input
        task["reply"] = self.txt_task_reply.get(1.0, tk.END).strip()
        
        self.refresh_task_list()
        self.list_tasks.selection_set(idx) # Reselect
        self.apply_current_module(show_msg=False) # Auto sync buffer to memory

    def add_custom_task(self):
        if not self.current_selected_node: return
        new_task = {
            "id": f"custom_{int(time.time())}",
            "summary": "New Custom Task",
            "completed": False,
            "date": "",
            "reply": ""
        }
        self.current_tasks_buffer.append(new_task)
        self.refresh_task_list()
        self.list_tasks.selection_set(tk.END)
        self.on_task_select(None)

    def delete_custom_task(self):
        selection = self.list_tasks.curselection()
        if not selection: return
        idx = selection[0]
        task = self.current_tasks_buffer[idx]
        
        if task.get("id", "").startswith("sys_"):
            messagebox.showwarning("Permission Denied", "System verification tasks are mandatory and cannot be deleted.")
            return
            
        if messagebox.askyesno("Confirm Delete", f"Delete task '{task['summary']}'?"):
            del self.current_tasks_buffer[idx]
            self.refresh_task_list()
            self.apply_current_module(show_msg=False)

    # ================= Dependency Tree =================
    def refresh_dep_tree(self, depends_on_list, current_root, current_mod):
        for item in self.tree_deps.get_children(): self.tree_deps.delete(item)
        valid_deps = set(depends_on_list)
        missing_deps = set(depends_on_list)

        for root_name in self.registry.get("roots", []):
            root_id = f"DEPROOT|{root_name}"
            self.tree_deps.insert("", tk.END, iid=root_id, text=f"☐ 📦 {root_name}", open=False)
            modules = self.registry.get("modules", {}).get(root_name, {})
            
            for mod_key in sorted(modules.keys()):
                if mod_key == f"{current_root}|{current_mod}": continue
                mod_data = modules[mod_key]
                if mod_data.get("type") == "module":
                    parts = mod_key.split('|')
                    parent_id = root_id
                    for i in range(len(parts) - 1):
                        folder_name = parts[i]
                        folder_id = f"DEPFOLDER|{root_name}|{'|'.join(parts[:i+1])}"
                        if not self.tree_deps.exists(folder_id):
                            self.tree_deps.insert(parent_id, tk.END, iid=folder_id, text=f"☐ 📁 {folder_name}", open=False)
                        parent_id = folder_id

                    leaf_name = parts[-1]
                    mod_id = f"DEPMOD|{root_name}|{mod_key}"
                    full_path = f"{root_name}|{mod_key}"
                    is_checked = full_path in valid_deps
                    prefix = "☑ " if is_checked else "☐ "

                    if full_path in missing_deps: missing_deps.remove(full_path)

                    text_disp = f"{prefix}⚙️ _internal" if leaf_name == "_internal" else f"{prefix}📄 {leaf_name}"
                    self.tree_deps.insert(parent_id, tk.END, iid=mod_id, text=text_disp)

        if missing_deps:
            missing_root = "DEPMISSING_ROOT"
            self.tree_deps.insert("", 0, iid=missing_root, text="⚠️ Missing Dependencies (Please uncheck)", open=True, tags=("error",))
            for dep in missing_deps:
                self.tree_deps.insert(missing_root, tk.END, iid=f"DEPUNK|{dep}", text=f"☑ {dep}", tags=("error",))

        self.update_dep_folder_states()

    def update_dep_folder_states(self, node=""):
        children = self.tree_deps.get_children(node)
        if not children:
            if node.startswith("DEPMOD|") or node.startswith("DEPUNK|"):
                return "☑" in self.tree_deps.item(node, "text")
            return False 

        all_checked, any_checked = True, False
        for child in children:
            is_checked = self.update_dep_folder_states(child)
            if is_checked: any_checked = True
            else: all_checked = False

        if node and (node.startswith("DEPFOLDER|") or node.startswith("DEPROOT|")):
            current_text = self.tree_deps.item(node, "text")
            base_text = current_text.replace("☑ ", "").replace("☐ ", "").replace("[-] ", "")
            if all_checked: self.tree_deps.item(node, text=f"☑ {base_text}")
            elif any_checked: self.tree_deps.item(node, text=f"[-] {base_text}")
            else: self.tree.item(node, text=f"☐ {base_text}")
        return all_checked

    def on_dep_tree_click(self, event):
        region = self.tree_deps.identify("region", event.x, event.y)
        if region in ("cell", "tree"):
            item_id = self.tree_deps.focus()
            if not item_id or item_id == "DEPMISSING_ROOT": return
            target_check = not ("☑" in self.tree_deps.item(item_id, "text") or "[-]" in self.tree_deps.item(item_id, "text"))
            self.toggle_dep_node(item_id, target_check)
            self.update_dep_folder_states()

    def toggle_dep_node(self, node_id, target_check):
        current_text = self.tree_deps.item(node_id, "text")
        if any(c in current_text for c in ("☑", "☐", "[-]")):
             base_text = current_text.replace("☑ ", "").replace("☐ ", "").replace("[-] ", "")
             self.tree_deps.item(node_id, text=f"{'☑ ' if target_check else '☐ '}{base_text}")
        for child in self.tree_deps.get_children(node_id):
            self.toggle_dep_node(child, target_check)

    def get_all_checked_deps(self):
        checked = []
        def traverse(node):
            if node.startswith(("DEPMOD|", "DEPUNK|")) and "☑" in self.tree_deps.item(node, "text"):
                if node.startswith("DEPMOD|"): checked.append(f"{node.split('|', 2)[1]}|{node.split('|', 2)[2]}")
                elif node.startswith("DEPUNK|"): checked.append(node.split('|', 1)[1])
            for child in self.tree_deps.get_children(node): traverse(child)
        traverse("")
        return checked

    # ================= General Actions =================
    def clear_right_panel(self):
        self.current_selected_node = None
        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text="-")
        self.entry_desc.delete(0, tk.END)
        for txt in (self.txt_src, self.txt_inc, self.txt_inc_dirs, self.txt_help_docs):
            txt.delete(1.0, tk.END)
        for item in self.tree_deps.get_children(): self.tree_deps.delete(item)
        for lb in (self.list_preview_src, self.list_preview_inc, self.list_preview_inc_dirs, self.list_tasks):
            lb.delete(0, tk.END)
        self.set_right_panel_state(tk.DISABLED)
        self.set_task_editor_state(tk.DISABLED)

    def set_right_panel_state(self, state):
        self.entry_desc.config(state=state)
        for txt in (self.txt_src, self.txt_inc, self.txt_inc_dirs, self.txt_help_docs):
            txt.config(state=state)
        self.btn_apply.config(state=state)

    def apply_current_module(self, show_msg=False):
        if not self.current_selected_node: return
        root_name, mod_key = self.current_selected_node

        if root_name not in self.registry["modules"] or mod_key not in self.registry["modules"][root_name]:
            self.current_selected_node = None
            return

        target_mod = self.registry["modules"][root_name][mod_key]
        target_mod["description"] = self.entry_desc.get().strip()
        target_mod["src_patterns"] = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_patterns"] = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_dirs"] = [f.strip() for f in self.txt_inc_dirs.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["help_docs"] = [f.strip() for f in self.txt_help_docs.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["depends_on"] = self.get_all_checked_deps() 
        target_mod["tasks"] = copy.deepcopy(self.current_tasks_buffer)

        self.mark_dirty()
        if show_msg: messagebox.showinfo("Saved to Memory", f"Changes for '{mod_key}' applied to buffer.")
        
        # Trigger icon update in Treeview dynamically
        icon, status_text = self.model.get_module_status(target_mod)
        leaf_name = mod_key.split('|')[-1]
        mod_id = f"MOD|{root_name}|{mod_key}"
        disp_text = f"{icon} ⚙️ _internal" if leaf_name == "_internal" else f"{icon} 📄 {leaf_name}"
        self.tree.item(mod_id, text=disp_text)
        self.lbl_module_id.config(text=f"{icon} {root_name} | {mod_key.replace('|', ' / ')}   [{status_text}]")

        if self.notebook.index(self.notebook.select()) == 1: self.update_preview()

    def save_json_to_disk(self, event=None):
        if self.current_selected_node:
            self.apply_current_module(show_msg=False)
        try:
            self.model.save()
            self.mark_clean()
            messagebox.showinfo("Success", "✅ Configuration successfully saved to disk!", icon="info")
        except Exception as e:
            messagebox.showerror("Save Failed", f"Error writing to JSON:\n{e}")

    # ================= Help Docs =================
    def open_help_docs(self, txt_widget):
        content = txt_widget.get(1.0, tk.END).strip()
        if not content:
            messagebox.showinfo("Info", "No help documents configured for this module.")
            return
            
        patterns = [f.strip() for f in content.split('\n') if f.strip()]
        abs_paths = self.model.resolve_paths(patterns, return_absolute=True)
        
        if not abs_paths:
            messagebox.showwarning("Warning", "Could not resolve valid paths. Please check macros and patterns.")
            return
            
        for path in abs_paths:
            try:
                os.startfile(path)
            except Exception as e:
                print(f"Failed to open file {path}: {e}")

    # ================= Diagnostic Tools Integration =================
    def display_report_window(self, title, report_text, is_error=False):
        top = tk.Toplevel(self.root)
        top.title(title)
        top.geometry("800x600")
        
        txt = tk.Text(top, font=("Consolas", 10), bg="#1e1e1e", fg="#d4d4d4" if not is_error else "#ffcccc")
        txt.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        txt.insert(tk.END, report_text)
        txt.config(state=tk.DISABLED)

    def run_validation_tool(self):
        if self.current_selected_node: self.apply_current_module(show_msg=False)
        has_error, report = framework_validator.run_validation(self.model)
        self.display_report_window("Sanity Check Report", report, is_error=has_error)

    def run_orphan_tool(self):
        if self.current_selected_node: self.apply_current_module(show_msg=False)
        has_orphans, report = framework_orphan_checker.run_orphan_check(self.model)
        self.display_report_window("Orphan Scan Report", report, is_error=has_orphans)

    # ================= Browsing & Macros =================
    def popup_macro_menu(self, event, target_widget):
        if not event: return
        menu = tk.Menu(self.root, tearoff=0)
        macros = self.registry.get("macros", {})
        if not macros: menu.add_command(label="(No Macros Available)")
        else:
            for mac in sorted(macros.keys()):
                menu.add_command(label=f"${{{mac}}}", command=lambda m=mac: self.insert_macro_to_widget(target_widget, m))
        menu.post(event.x_root, event.y_root)

    def insert_macro_to_widget(self, widget, macro_name):
        content = widget.get(1.0, tk.END).strip()
        widget.insert(tk.END, ("\n" if content else "") + f"${{{macro_name}}}/")

    def browse_path(self, target_widget, is_dir=False):
        initial_dir = Path(self.gmp_location)
        
        chosen_paths = []
        if is_dir: 
            chosen = filedialog.askdirectory(initialdir=initial_dir)
            if chosen: chosen_paths.append(chosen)
        else: 
            chosen_tuple = filedialog.askopenfilenames(initialdir=initial_dir)
            if chosen_tuple: chosen_paths = list(chosen_tuple)
            
        if not chosen_paths: return
        
        macros = sorted(self.registry.get("macros", {}).items(), key=lambda item: len(item[1]), reverse=True)
        final_strings = []
        
        for path_str in chosen_paths:
            chosen_path = Path(path_str)
            final_str = ""
            for mac, val in macros:
                if chosen_path.is_relative_to(Path(val)):
                    rel = chosen_path.relative_to(Path(val)).as_posix()
                    final_str = f"${{{mac}}}/{rel}" if rel != "." else f"${{{mac}}}"
                    break
            if not final_str: final_str = chosen_path.as_posix()
            
            if is_dir:
                if target_widget in (self.txt_src, self.txt_inc):
                    final_str = final_str.rstrip('/') + "/*"
                    
            final_strings.append(final_str)

        content = target_widget.get(1.0, tk.END).strip()
        prefix = "\n" if content else ""
        target_widget.insert(tk.END, prefix + "\n".join(final_strings))
        self.mark_dirty()

    def open_macro_manager(self):
        top = tk.Toplevel(self.root)
        top.title("Global Path Macros Manager")
        top.geometry("600x400")
        top.grab_set()

        tree = ttk.Treeview(top, columns=("Value",), selectmode="browse")
        tree.heading("#0", text="Macro Name")
        tree.heading("Value", text="Absolute Path")
        tree.column("#0", width=200)
        tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        def refresh_macros():
            for item in tree.get_children(): tree.delete(item)
            for k, v in self.registry.get("macros", {}).items():
                tree.insert("", tk.END, text=k, values=(v,))
        refresh_macros()

        btn_frame = ttk.Frame(top)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)

        def add_macro():
            name = simpledialog.askstring("Add Macro", "Macro Name (e.g. C2000WARE):", parent=top)
            if name and (val := filedialog.askdirectory()):
                self.registry["macros"][name] = Path(val).as_posix()
                self.mark_dirty()
                refresh_macros()

        def del_macro():
            sel = tree.selection()
            if not sel: return
            name = tree.item(sel[0], "text")
            if name == "GMP_PRO_LOCATION": return messagebox.showwarning("Access Denied", "Core macro cannot be deleted.", parent=top)
            if messagebox.askyesno("Delete", f"Delete macro '{name}'?", parent=top):
                del self.registry["macros"][name]
                self.mark_dirty()
                refresh_macros()

        ttk.Button(btn_frame, text="➕ Add/Replace", command=add_macro).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="❌ Delete", command=del_macro).pack(side=tk.LEFT)

    # ================= Live Preview =================
    def on_tab_changed(self, event):
        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    def update_preview(self):
        for lb in (self.list_preview_src, self.list_preview_inc, self.list_preview_inc_dirs):
            lb.delete(0, tk.END)
            
        if not self.tree.selection() or self.tree.selection()[0].startswith("ROOT|"): return
        
        src_patterns = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        inc_patterns = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        inc_dirs_patterns = [f.strip() for f in self.txt_inc_dirs.get(1.0, tk.END).split('\n') if f.strip()]

        src_files = self.model.resolve_paths(src_patterns)
        inc_files = self.model.resolve_paths(inc_patterns)
        dirs_files = self.model.resolve_paths(inc_dirs_patterns, is_dir_mode=True)

        for f in (src_files or ["⚠️ No source files matched"]): self.list_preview_src.insert(tk.END, f)
        for f in (inc_files or ["⚠️ No header files matched"]): self.list_preview_inc.insert(tk.END, f)
        for d in (dirs_files or ["ℹ️ No compiler include dirs defined"]): self.list_preview_inc_dirs.insert(tk.END, d)

    # =========== CRUD Operations ===========
    def add_module(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        default_prefix = ""
        
        if node_id.startswith("ROOT|"): root_name = node_id.split('|', 1)[1]
        elif node_id.startswith("FOLDER|"):
            _, root_name, folder_path = node_id.split('|', 2)
            default_prefix = folder_path + "|"
        elif node_id.startswith("MOD|"):
            _, root_name, mod_key = node_id.split('|', 2)
            parts = mod_key.split('|')
            if len(parts) > 1: default_prefix = "|".join(parts[:-1]) + "|"

        mod_key = simpledialog.askstring("Add Sub-module", f"Add under '{root_name}':", initialvalue=default_prefix)
        if mod_key and mod_key.strip():
            mod_key = mod_key.strip()
            if root_name not in self.registry["modules"]: self.registry["modules"][root_name] = {}
            parts = mod_key.split('|')
            for i in range(1, len(parts)):
                parent_key = '|'.join(parts[:i])
                if parent_key in self.registry["modules"][root_name]:
                    if messagebox.askyesno("Convert", "Convert this module into a folder package?"):
                        if self.current_selected_node == (root_name, parent_key):
                            self.current_selected_node = None 
                        old_data = self.registry["modules"][root_name].pop(parent_key)
                        self.registry["modules"][root_name][f"{parent_key}|_internal"] = old_data
                    else: return 
            if mod_key in self.registry["modules"][root_name]: return
            
            self.registry["modules"][root_name][mod_key] = {
                "type": "module", "description": "", 
                "src_patterns": [], "inc_patterns": [], "inc_dirs": [], "help_docs": [], "depends_on": []
            }
            # Inject mandatory tasks
            self.model.inject_default_tasks(self.registry["modules"][root_name][mod_key])
            
            self.mark_dirty()
            if self.model.ensure_internals(): self.mark_dirty()
            self.refresh_tree()
            self.tree.selection_set(f"MOD|{root_name}|{mod_key}")

    def delete_node(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        if node_id.startswith("FOLDER|"):
            _, root_name, folder_path = node_id.split('|', 2)
            if messagebox.askyesno("Danger", f"Delete folder '{folder_path}' and ALL its contents?"):
                self.current_selected_node = None
                keys_to_delete = [k for k in self.registry["modules"][root_name].keys() if k == folder_path or k.startswith(folder_path + '|')]
                for k in keys_to_delete: del self.registry["modules"][root_name][k]
                self.model.check_and_rollback_parent(root_name, folder_path)
                self.mark_dirty()
                if self.model.ensure_internals(): self.mark_dirty()
                self.refresh_tree()
                self.clear_right_panel()
        elif node_id.startswith("MOD|"):
            _, root_name, mod_key = node_id.split('|', 2)
            if messagebox.askyesno("Confirm", f"Delete module '{mod_key}'?"):
                if self.current_selected_node == (root_name, mod_key):
                    self.current_selected_node = None 
                del self.registry["modules"][root_name][mod_key]
                self.model.check_and_rollback_parent(root_name, mod_key)
                self.mark_dirty()
                if self.model.ensure_internals(): self.mark_dirty()
                self.refresh_tree()
                self.clear_right_panel()

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkDevGUI(root)
    root.mainloop()