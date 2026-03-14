import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
from pathlib import Path

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 核心框架资产管理器 V8 (完美版：双击勾选防误触)")
        self.root.geometry("1100x800")
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        self.json_path = Path(__file__).parent / "gmp_framework_dic.json"
        
        self.registry = {"roots": [], "modules": {}, "macros": {}}
        self.current_selected_node = None 
        self.all_available_modules = [] 
        self.is_dirty = False 
        
        self.build_ui()
        self.load_data()

    def mark_dirty(self):
        self.is_dirty = True
        self.root.title("GMP 核心框架资产管理器 V8 - *未保存的修改*")
        self.btn_global_save.config(state=tk.NORMAL)

    def mark_clean(self):
        self.is_dirty = False
        self.root.title("GMP 核心框架资产管理器 V8 (完美版：双击勾选防误触)")
        self.btn_global_save.config(state=tk.DISABLED)

    def on_closing(self):
        if self.is_dirty:
            if messagebox.askyesno("退出提示", "您有未保存的修改，直接退出将丢失所有更改。\n\n确认要强行退出吗？"):
                self.root.destroy()
        else:
            self.root.destroy()

    def build_ui(self):
        # ================= 顶部全局工具栏 =================
        toolbar = ttk.Frame(self.root, padding=5, relief=tk.RAISED)
        toolbar.pack(fill=tk.X)
        
        self.btn_global_save = ttk.Button(toolbar, text="💾 将全部更改保存至 JSON 磁盘", command=self.save_json_to_disk, state=tk.DISABLED)
        self.btn_global_save.pack(side=tk.LEFT, padx=5, ipadx=10, ipady=3)
        
        ttk.Button(toolbar, text="🔄 放弃更改并重新加载", command=self.load_data).pack(side=tk.LEFT, padx=5, ipadx=10, ipady=3)

        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # ================= 左侧：无限层级树形目录 =================
        left_frame = ttk.Frame(self.paned_window)
        self.paned_window.add(left_frame, weight=1)

        tree_frame = ttk.Frame(left_frame)
        tree_frame.pack(fill=tk.BOTH, expand=True)
        
        tree_scroll = ttk.Scrollbar(tree_frame)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree = ttk.Treeview(tree_frame, yscrollcommand=tree_scroll.set, selectmode="browse")
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.config(command=self.tree.yview)
        self.tree.heading("#0", text="框架目录树 (内存视图)", anchor=tk.W)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="➕ 新增子模块", command=self.add_module).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="❌ 删除选中", command=self.delete_node).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(left_frame, text="⚙️ 全局路径宏管理", command=self.open_macro_manager).pack(fill=tk.X, pady=2)

        # ================= 右侧：选项卡面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=5)
        self.paned_window.add(right_frame, weight=3)

        top_info_frame = ttk.Frame(right_frame)
        top_info_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(top_info_frame, text="当前选中节点:").pack(side=tk.LEFT)
        self.lbl_module_id = ttk.Label(top_info_frame, font=("Helvetica", 11, "bold"), foreground="blue", text="-")
        self.lbl_module_id.pack(side=tk.LEFT, padx=10)

        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

        # ---------- 选项卡 1：规则配置 ----------
        self.tab_config = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_config, text="⚙️ 规则配置")

        ttk.Label(self.tab_config, text="模块描述:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.entry_desc = ttk.Entry(self.tab_config, width=50)
        self.entry_desc.grid(row=0, column=1, sticky=tk.EW, pady=5)

        def create_path_editor(row, label_text, target_attr):
            lbl_frame = ttk.Frame(self.tab_config)
            lbl_frame.grid(row=row, column=0, sticky=tk.NW, pady=5)
            ttk.Label(lbl_frame, text=label_text).pack(anchor=tk.W)
            
            txt_widget = tk.Text(self.tab_config, height=3, width=40)
            txt_widget.grid(row=row, column=1, sticky=tk.EW, pady=5)
            
            btn_frame_sub = ttk.Frame(lbl_frame)
            btn_frame_sub.pack(anchor=tk.W, pady=2)
            ttk.Button(btn_frame_sub, text="📄 文件", width=6, command=lambda: self.browse_path(txt_widget, is_dir=False)).pack(side=tk.LEFT, padx=(0,2))
            ttk.Button(btn_frame_sub, text="📁 目录", width=6, command=lambda: self.browse_path(txt_widget, is_dir=True)).pack(side=tk.LEFT)
            
            btn_macro = ttk.Button(lbl_frame, text="📌 插入宏")
            btn_macro.pack(anchor=tk.W, fill=tk.X, pady=2)
            btn_macro.bind("<Button-1>", lambda event, w=txt_widget: self.popup_macro_menu(event, w))
            
            return txt_widget

        self.txt_src = create_path_editor(1, "源文件 (src_patterns):", "src")
        self.txt_inc = create_path_editor(2, "头文件 (inc_patterns):", "inc")
        self.txt_inc_dirs = create_path_editor(3, "编译依赖包含路径\n(inc_dirs - 给编译器):", "inc_dirs")

        # 依赖区 - 全新升级为 Treeview，并修改提示语
        ttk.Label(self.tab_config, text="依赖模块树 (depends_on):\n(双击节点即可勾选/取消)").grid(row=4, column=0, sticky=tk.NW, pady=5)
        dep_frame = ttk.Frame(self.tab_config)
        dep_frame.grid(row=4, column=1, sticky=tk.EW, pady=5)
        dep_scroll = ttk.Scrollbar(dep_frame)
        dep_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.tree_deps = ttk.Treeview(dep_frame, yscrollcommand=dep_scroll.set, selectmode="browse", height=7, show="tree")
        self.tree_deps.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dep_scroll.config(command=self.tree_deps.yview)
        
        # 【终极一击】：将 <ButtonRelease-1> (单击) 改为 <Double-1> (双击)
        self.tree_deps.bind("<Double-1>", self.on_dep_tree_click)
        self.tree_deps.tag_configure("error", foreground="#d32f2f", font=("Helvetica", 9, "bold"))

        self.tab_config.rowconfigure(4, weight=1) 
        self.tab_config.columnconfigure(1, weight=1)
        self.btn_apply = ttk.Button(self.tab_config, text="✔️ 手动应用 (切换节点时也会自动保存)", command=lambda: self.apply_current_module(show_msg=True))
        self.btn_apply.grid(row=5, column=1, sticky=tk.E, pady=10)

        # ---------- 选项卡 2：匹配文件预览 ----------
        self.tab_preview = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_preview, text="👁️ 匹配文件预览 (实时)")
        
        pw_preview = ttk.PanedWindow(self.tab_preview, orient=tk.VERTICAL)
        pw_preview.pack(fill=tk.BOTH, expand=True)

        def create_preview_box(parent, text, color):
            frame = ttk.Frame(parent)
            ttk.Label(frame, text=text, foreground=color).pack(anchor=tk.W)
            lb = tk.Listbox(frame, height=4, bg="#f9f9f9")
            lb.pack(fill=tk.BOTH, expand=True)
            parent.add(frame, weight=1)
            return lb

        self.list_preview_src = create_preview_box(pw_preview, "解析出的物理源文件 (.c / .cpp 等):", "blue")
        self.list_preview_inc = create_preview_box(pw_preview, "解析出的物理头文件 (.h / .hpp 等):", "green")
        self.list_preview_inc_dirs = create_preview_box(pw_preview, "解析出的包含目录依赖 (Include Dirs):", "#cc6600")

        self.clear_right_panel() 

    # ================= 核心逻辑：数据加载与防呆补全 =================
    def load_data(self):
        if not self.json_path.exists():
            self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}, "macros": {}}
        else:
            try:
                with open(self.json_path, 'r', encoding='utf-8') as f:
                    self.registry = json.load(f)
            except Exception as e:
                messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
                return
                
        if "macros" not in self.registry:
            self.registry["macros"] = {}
        self.registry["macros"]["GMP_PRO_LOCATION"] = Path(self.gmp_location).as_posix()
            
        for root_name, modules in self.registry.get("modules", {}).items():
            for mod_data in modules.values():
                mod_data.setdefault("description", "")
                mod_data.setdefault("src_patterns", [])
                mod_data.setdefault("inc_patterns", [])
                mod_data.setdefault("inc_dirs", [])
                mod_data.setdefault("depends_on", [])

        self.ensure_internals()
        self.refresh_tree()
        self.mark_clean()

    def ensure_internals(self):
        for root_name, modules in self.registry.get("modules", {}).items():
            folders = set()
            for mod_key in modules.keys():
                parts = mod_key.split('|')
                for i in range(1, len(parts)):
                    folders.add('|'.join(parts[:i]))
                    
            for folder in folders:
                internal_key = f"{folder}|_internal"
                if internal_key not in modules:
                    modules[internal_key] = {
                        "type": "module", "description": "自动生成的包基础配置",
                        "src_patterns": [], "inc_patterns": [], "inc_dirs": [], "depends_on": []
                    }
                    self.mark_dirty()

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
            self.tree.insert("", tk.END, iid=root_id, text=root_name, open=True)
            
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
                    if leaf_name == "_internal":
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text="⚙️ _internal (基础配置)", tags=("internal",))
                    else:
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"📄 {leaf_name}")

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
            self.lbl_module_id.config(text=f"📁 {root_name} | {folder_path.replace('|', ' / ')}  (汇总)")
            
            modules = self.registry["modules"][root_name]
            sub_modules_count = 0
            all_src, all_inc, all_inc_dirs, all_deps = set(), set(), set(), set()
            
            for k, v in modules.items():
                if k.startswith(folder_path + '|'):
                    sub_modules_count += 1
                    all_src.update(v.get("src_patterns", []))
                    all_inc.update(v.get("inc_patterns", []))
                    all_inc_dirs.update(v.get("inc_dirs", []))
                    all_deps.update(v.get("depends_on", []))
            
            self.entry_desc.delete(0, tk.END)
            self.entry_desc.insert(0, f"本文件夹包含 {sub_modules_count} 个独立模块/配置。")
            self.txt_src.delete(1.0, tk.END)
            self.txt_src.insert(tk.END, "\n".join(sorted(list(all_src))))
            self.txt_inc.delete(1.0, tk.END)
            self.txt_inc.insert(tk.END, "\n".join(sorted(list(all_inc))))
            self.txt_inc_dirs.delete(1.0, tk.END)
            self.txt_inc_dirs.insert(tk.END, "\n".join(sorted(list(all_inc_dirs))))
            
            for item in self.tree_deps.get_children(): self.tree_deps.delete(item)
                
            self.entry_desc.config(state=tk.DISABLED)
            self.txt_src.config(state=tk.DISABLED)
            self.txt_inc.config(state=tk.DISABLED)
            self.txt_inc_dirs.config(state=tk.DISABLED)
            self.btn_apply.config(state=tk.DISABLED)
            
            self.list_preview_src.delete(0, tk.END)
            self.list_preview_inc.delete(0, tk.END)
            self.list_preview_inc_dirs.delete(0, tk.END)
            return
            
        # 单个模块逻辑
        _, root_name, mod_key = node_id.split('|', 2)
        self.current_selected_node = (root_name, mod_key)
        mod_data = self.registry["modules"][root_name][mod_key]

        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text=f"{root_name} | {mod_key.replace('|', ' / ')}")
        
        self.entry_desc.delete(0, tk.END)
        self.entry_desc.insert(0, mod_data.get("description", ""))
        self.txt_src.delete(1.0, tk.END)
        self.txt_src.insert(tk.END, "\n".join(mod_data.get("src_patterns", [])))
        self.txt_inc.delete(1.0, tk.END)
        self.txt_inc.insert(tk.END, "\n".join(mod_data.get("inc_patterns", [])))
        self.txt_inc_dirs.delete(1.0, tk.END)
        self.txt_inc_dirs.insert(tk.END, "\n".join(mod_data.get("inc_dirs", [])))

        self.refresh_dep_tree(mod_data.get("depends_on", []), root_name, mod_key)

        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    # ================= 依赖树 (Checkbox Tree) 核心逻辑 =================
    def refresh_dep_tree(self, depends_on_list, current_root, current_mod):
        for item in self.tree_deps.get_children(): self.tree_deps.delete(item)

        valid_deps = set(depends_on_list)
        missing_deps = set(depends_on_list)

        for root_name in self.registry.get("roots", []):
            root_id = f"DEPROOT|{root_name}"
            self.tree_deps.insert("", tk.END, iid=root_id, text=f"☐ 📦 {root_name}", open=False)

            modules = self.registry.get("modules", {}).get(root_name, {})
            sorted_keys = sorted(modules.keys())

            for mod_key in sorted_keys:
                if mod_key == f"{current_root}|{current_mod}": continue
                mod_data = modules[mod_key]
                if mod_data.get("type") == "module":
                    parts = mod_key.split('|')
                    parent_id = root_id

                    for i in range(len(parts) - 1):
                        folder_name = parts[i]
                        current_folder_path = "|".join(parts[:i+1])
                        folder_id = f"DEPFOLDER|{root_name}|{current_folder_path}"
                        if not self.tree_deps.exists(folder_id):
                            self.tree_deps.insert(parent_id, tk.END, iid=folder_id, text=f"☐ 📁 {folder_name}", open=False)
                        parent_id = folder_id

                    leaf_name = parts[-1]
                    mod_id = f"DEPMOD|{root_name}|{mod_key}"
                    full_mod_path = f"{root_name}|{mod_key}"

                    is_checked = full_mod_path in valid_deps
                    prefix = "☑ " if is_checked else "☐ "

                    if full_mod_path in missing_deps:
                        missing_deps.remove(full_mod_path)

                    if leaf_name == "_internal":
                        self.tree_deps.insert(parent_id, tk.END, iid=mod_id, text=f"{prefix}⚙️ _internal")
                    else:
                        self.tree_deps.insert(parent_id, tk.END, iid=mod_id, text=f"{prefix}📄 {leaf_name}")

        if missing_deps:
            missing_root = "DEPMISSING_ROOT"
            self.tree_deps.insert("", 0, iid=missing_root, text="⚠️ 缺失/失效的关联 (建议取消勾选或修正)", open=True, tags=("error",))
            for dep in missing_deps:
                dep_id = f"DEPUNK|{dep}"
                self.tree_deps.insert(missing_root, tk.END, iid=dep_id, text=f"☑ {dep}", tags=("error",))

        self.update_dep_folder_states()

    def update_dep_folder_states(self, node=""):
        children = self.tree_deps.get_children(node)
        if not children:
            if node.startswith("DEPMOD|") or node.startswith("DEPUNK|"):
                return "☑" in self.tree_deps.item(node, "text")
            return False 

        all_checked = True
        any_checked = False

        for child in children:
            is_checked = self.update_dep_folder_states(child)
            if is_checked: any_checked = True
            else: all_checked = False

        if node and (node.startswith("DEPFOLDER|") or node.startswith("DEPROOT|")):
            current_text = self.tree_deps.item(node, "text")
            base_text = current_text.replace("☑ ", "").replace("☐ ", "").replace("[-] ", "")
            if all_checked: self.tree_deps.item(node, text=f"☑ {base_text}")
            elif any_checked: self.tree_deps.item(node, text=f"[-] {base_text}")
            else: self.tree_deps.item(node, text=f"☐ {base_text}")

        return all_checked

    def on_dep_tree_click(self, event):
        """双击触发勾选逻辑：有效防误触"""
        region = self.tree_deps.identify("region", event.x, event.y)
        if region == "cell" or region == "tree":
            item_id = self.tree_deps.focus()
            if not item_id or item_id == "DEPMISSING_ROOT": return

            current_text = self.tree_deps.item(item_id, "text")
            target_check = False if "☑" in current_text or "[-]" in current_text else True

            self.toggle_dep_node(item_id, target_check)
            self.update_dep_folder_states()

    def toggle_dep_node(self, node_id, target_check):
        current_text = self.tree_deps.item(node_id, "text")
        if "☑" in current_text or "☐" in current_text or "[-]" in current_text:
             base_text = current_text.replace("☑ ", "").replace("☐ ", "").replace("[-] ", "")
             new_prefix = "☑ " if target_check else "☐ "
             self.tree_deps.item(node_id, text=f"{new_prefix}{base_text}")

        for child in self.tree_deps.get_children(node_id):
            self.toggle_dep_node(child, target_check)

    def get_all_checked_deps(self):
        checked = []
        def traverse(node):
            if node.startswith("DEPMOD|") or node.startswith("DEPUNK|"):
                if "☑" in self.tree_deps.item(node, "text"):
                    if node.startswith("DEPMOD|"):
                        _, root_name, mod_key = node.split('|', 2)
                        checked.append(f"{root_name}|{mod_key}")
                    elif node.startswith("DEPUNK|"):
                        _, dep_name = node.split('|', 1)
                        checked.append(dep_name)
            for child in self.tree_deps.get_children(node):
                traverse(child)
        traverse("")
        return checked

    def clear_right_panel(self):
        self.current_selected_node = None
        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text="-")
        self.entry_desc.delete(0, tk.END)
        self.txt_src.delete(1.0, tk.END)
        self.txt_inc.delete(1.0, tk.END)
        self.txt_inc_dirs.delete(1.0, tk.END)
        for item in self.tree_deps.get_children(): self.tree_deps.delete(item)
        self.list_preview_src.delete(0, tk.END)
        self.list_preview_inc.delete(0, tk.END)
        self.list_preview_inc_dirs.delete(0, tk.END)
        self.set_right_panel_state(tk.DISABLED)

    def set_right_panel_state(self, state):
        self.entry_desc.config(state=state)
        self.txt_src.config(state=state)
        self.txt_inc.config(state=state)
        self.txt_inc_dirs.config(state=state)
        self.btn_apply.config(state=state)

    def apply_current_module(self, show_msg=False):
        if not self.current_selected_node: return
        root_name, mod_key = self.current_selected_node

        target_mod = self.registry["modules"][root_name][mod_key]
        target_mod["description"] = self.entry_desc.get().strip()
        target_mod["src_patterns"] = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_patterns"] = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_dirs"] = [f.strip() for f in self.txt_inc_dirs.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["depends_on"] = self.get_all_checked_deps() 

        self.mark_dirty()
        if show_msg:
            messagebox.showinfo("暂存成功", f"模块 '{mod_key}' 的修改已暂存内存。\n别忘了点击顶部的【保存全部更改】写入磁盘！")
        
        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    def save_json_to_disk(self):
        if self.current_selected_node:
            self.apply_current_module(show_msg=False)
            
        try:
            with open(self.json_path, 'w', encoding='utf-8') as f:
                json.dump(self.registry, f, indent=4, ensure_ascii=False)
            self.mark_clean()
            messagebox.showinfo("成功", "✅ 所有配置已成功永久保存至 JSON 磁盘文件中！")
        except Exception as e:
            messagebox.showerror("保存失败", f"写入 JSON 发生错误:\n{e}")

    # ================= 智能浏览、宏插入与路径转换 =================
    def popup_macro_menu(self, event, target_widget):
        menu = tk.Menu(self.root, tearoff=0)
        macros = self.registry.get("macros", {})
        if not macros:
            menu.add_command(label="(暂无可用宏)")
        else:
            for mac in sorted(macros.keys()):
                menu.add_command(label=f"${{{mac}}}", command=lambda m=mac: self.insert_macro_to_widget(target_widget, m))
        menu.post(event.x_root, event.y_root)

    def insert_macro_to_widget(self, widget, macro_name):
        content = widget.get(1.0, tk.END).strip()
        insert_str = f"${{{macro_name}}}/"
        if content: widget.insert(tk.END, "\n" + insert_str)
        else: widget.insert(tk.END, insert_str)

    def browse_path(self, target_widget, is_dir=False):
        if not self.current_selected_node: return
        initial_dir = Path(self.gmp_location)
            
        if is_dir: chosen = filedialog.askdirectory(initialdir=initial_dir)
        else: chosen = filedialog.askopenfilename(initialdir=initial_dir)
            
        if not chosen: return
        chosen_path = Path(chosen)
        
        macros = self.registry.get("macros", {})
        sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)
        
        final_str = ""
        for mac, val in sorted_macros:
            mac_path = Path(val)
            if chosen_path.is_relative_to(mac_path):
                rel = chosen_path.relative_to(mac_path).as_posix()
                final_str = f"${{{mac}}}/{rel}" if rel != "." else f"${{{mac}}}"
                break
                
        if not final_str:
            final_str = chosen_path.as_posix()

        if is_dir:
            if target_widget in (self.txt_src, self.txt_inc):
                final_str = final_str.rstrip('/') + "/*"

        target_widget.insert(tk.END, ("\n" if target_widget.get(1.0, tk.END).strip() else "") + final_str)

    # ================= 宏管理器 (Macro Manager) =================
    def open_macro_manager(self):
        top = tk.Toplevel(self.root)
        top.title("全局路径宏管理")
        top.geometry("600x400")
        top.transient(self.root)
        top.grab_set()

        ttk.Label(top, text="宏变量可作为路径基准。\n注意: GMP_PRO_LOCATION 是系统级核心宏，不可删除。").pack(pady=10)

        tree = ttk.Treeview(top, columns=("Value",), selectmode="browse")
        tree.heading("#0", text="宏变量名 (不含 ${})")
        tree.heading("Value", text="对应绝对路径")
        tree.column("#0", width=200)
        tree.column("Value", width=350)
        tree.pack(fill=tk.BOTH, expand=True, padx=10)

        def refresh_macros():
            for item in tree.get_children(): tree.delete(item)
            for k, v in self.registry.get("macros", {}).items():
                tree.insert("", tk.END, text=k, values=(v,))

        refresh_macros()

        btn_frame = ttk.Frame(top)
        btn_frame.pack(fill=tk.X, padx=10, pady=10)

        def add_macro():
            name = simpledialog.askstring("新增宏", "宏变量名 (例: C2000WARE):", parent=top)
            if not name: return
            val = filedialog.askdirectory(title="选择该宏指向的真实目录")
            if not val: return
            self.registry["macros"][name] = Path(val).as_posix()
            self.mark_dirty()
            refresh_macros()

        def del_macro():
            sel = tree.selection()
            if not sel: return
            name = tree.item(sel[0], "text")
            if name == "GMP_PRO_LOCATION":
                messagebox.showwarning("禁止", "GMP_PRO_LOCATION 是系统的命脉，不可删除！", parent=top)
                return
            if messagebox.askyesno("删除", f"确定删除宏 '{name}' 吗？", parent=top):
                del self.registry["macros"][name]
                self.mark_dirty()
                refresh_macros()

        ttk.Button(btn_frame, text="➕ 新增/覆盖", command=add_macro).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="❌ 删除", command=del_macro).pack(side=tk.LEFT)
        ttk.Button(btn_frame, text="关闭", command=top.destroy).pack(side=tk.RIGHT)

    # ================= 实时匹配预览 (Live Preview) =================
    def on_tab_changed(self, event):
        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    def update_preview(self):
        self.list_preview_src.delete(0, tk.END)
        self.list_preview_inc.delete(0, tk.END)
        self.list_preview_inc_dirs.delete(0, tk.END)

        if not self.current_selected_node: return
        
        src_patterns = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        inc_patterns = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        inc_dirs_patterns = [f.strip() for f in self.txt_inc_dirs.get(1.0, tk.END).split('\n') if f.strip()]

        def resolve_and_glob(patterns, is_dir_mode=False):
            matched = set()
            for pat in patterns:
                resolved_pat = pat
                for mac, val in self.registry.get("macros", {}).items():
                    resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
                
                pat_obj = Path(resolved_pat)
                
                if is_dir_mode:
                    if pat_obj.exists() and pat_obj.is_dir():
                        matched.add(pat_obj.as_posix())
                else:
                    if pat_obj.is_absolute():
                        anchor = pat_obj.anchor
                        rest = str(pat_obj.relative_to(anchor))
                        for f in Path(anchor).glob(rest):
                            if f.is_file(): matched.add(f.as_posix())
            return sorted(list(matched))

        src_files = resolve_and_glob(src_patterns)
        if not src_files: self.list_preview_src.insert(tk.END, "⚠️ 未匹配到源文件")
        for f in src_files: self.list_preview_src.insert(tk.END, f)

        inc_files = resolve_and_glob(inc_patterns)
        if not inc_files: self.list_preview_inc.insert(tk.END, "⚠️ 未匹配到头文件")
        for f in inc_files: self.list_preview_inc.insert(tk.END, f)
            
        dirs_files = resolve_and_glob(inc_dirs_patterns, is_dir_mode=True)
        if not dirs_files: self.list_preview_inc_dirs.insert(tk.END, "ℹ️ 无特定的编译器依赖目录")
        for d in dirs_files: self.list_preview_inc_dirs.insert(tk.END, d)

    # =========== 增删逻辑 ===========
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

        mod_key = simpledialog.askstring("新增子模块", f"在 '{root_name}' 下新增:\n(多级请用 '|' 分隔)", initialvalue=default_prefix)
        if mod_key and mod_key.strip():
            mod_key = mod_key.strip()
            if root_name not in self.registry["modules"]: self.registry["modules"][root_name] = {}
            parts = mod_key.split('|')
            for i in range(1, len(parts)):
                parent_key = '|'.join(parts[:i])
                if parent_key in self.registry["modules"][root_name]:
                    msg = (f"检测到冲突：'{parent_key}' 是独立模块。\n是否迁移为 '{parent_key}|_internal' 并升级为文件夹？")
                    if messagebox.askyesno("智能转换", msg):
                        old_data = self.registry["modules"][root_name].pop(parent_key)
                        self.registry["modules"][root_name][f"{parent_key}|_internal"] = old_data
                    else: return 
            for existing_key in self.registry["modules"][root_name].keys():
                if existing_key.startswith(mod_key + '|'):
                    messagebox.showwarning("冲突", f"'{mod_key}' 已是文件夹名称。请配置 '_internal'。")
                    return
            if mod_key in self.registry["modules"][root_name]: return
            
            self.registry["modules"][root_name][mod_key] = {
                "type": "module", "description": "新模块", 
                "src_patterns": [], "inc_patterns": [], "inc_dirs": [], "depends_on": []
            }
            self.mark_dirty()
            self.ensure_internals()
            self.refresh_tree()
            self.tree.selection_set(f"MOD|{root_name}|{mod_key}")

    def delete_node(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        if node_id.startswith("ROOT|"): return
        elif node_id.startswith("FOLDER|"):
            _, root_name, folder_path = node_id.split('|', 2)
            if messagebox.askyesno("危险确认", f"确定要删除文件夹 '{folder_path}' 及其【所有】配置吗？"):
                keys_to_delete = [k for k in self.registry["modules"][root_name].keys() if k == folder_path or k.startswith(folder_path + '|')]
                for k in keys_to_delete: del self.registry["modules"][root_name][k]
                self._check_and_rollback_parent(root_name, folder_path)
                self.mark_dirty()
                self.ensure_internals()
                self.refresh_tree()
                self.clear_right_panel()
            return
        elif node_id.startswith("MOD|"):
            _, root_name, mod_key = node_id.split('|', 2)
            if messagebox.askyesno("确认", f"确定要彻底删除 '{mod_key}' 吗？"):
                del self.registry["modules"][root_name][mod_key]
                self._check_and_rollback_parent(root_name, mod_key)
                self.mark_dirty()
                self.ensure_internals()
                self.refresh_tree()
                self.clear_right_panel()

    def _check_and_rollback_parent(self, root_name, deleted_path):
        parts = deleted_path.split('|')
        if len(parts) > 1:
            parent_folder = '|'.join(parts[:-1])
            remaining = [k for k in self.registry["modules"][root_name].keys() if k.startswith(parent_folder + '|')]
            if len(remaining) == 1 and remaining[0] == f"{parent_folder}|_internal":
                internal_data = self.registry["modules"][root_name].pop(remaining[0])
                self.registry["modules"][root_name][parent_folder] = internal_data

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkDevGUI(root)
    root.mainloop()