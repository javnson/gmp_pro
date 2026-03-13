import os
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
from pathlib import Path

# 引入数据模型
from framework_data_model_v1 import FrameworkDataModel

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 核心框架资产管理器 V10 (支持文件多选批量添加)")
        self.root.geometry("1100x800")
        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # 全局 Ctrl+S 快捷键绑定
        self.root.bind("<Control-s>", self.save_json_to_disk)
        self.root.bind("<Control-S>", self.save_json_to_disk)

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        json_path = Path(__file__).parent / "gmp_framework_dic.json"
        
        # 初始化分离出去的数据模型
        self.model = FrameworkDataModel(self.gmp_location, json_path)
        self.registry = self.model.registry 
        
        self.current_selected_node = None 
        self.all_available_modules = [] 
        self.is_dirty = False 
        
        self.build_ui()
        self.load_data()

    def mark_dirty(self):
        self.is_dirty = True
        self.root.title("GMP 核心框架资产管理器 V10 - *未保存的修改*")
        self.btn_global_save.config(state=tk.NORMAL)

    def mark_clean(self):
        self.is_dirty = False
        self.root.title("GMP 核心框架资产管理器 V10 (支持文件多选批量添加)")
        self.btn_global_save.config(state=tk.DISABLED)

    def on_closing(self):
        if self.is_dirty:
            if messagebox.askyesno("退出提示", "您有未保存的修改，直接退出将丢失所有更改。\n\n确认要强行退出吗？"):
                self.root.destroy()
        else:
            self.root.destroy()

    def build_ui(self):
        # 顶部全局工具栏
        toolbar = ttk.Frame(self.root, padding=5, relief=tk.RAISED)
        toolbar.pack(fill=tk.X)
        self.btn_global_save = ttk.Button(toolbar, text="💾 保存至磁盘 (Ctrl+S)", command=self.save_json_to_disk, state=tk.DISABLED)
        self.btn_global_save.pack(side=tk.LEFT, padx=5, ipadx=10, ipady=3)
        ttk.Button(toolbar, text="🔄 放弃更改并重新加载", command=self.load_data).pack(side=tk.LEFT, padx=5, ipadx=10, ipady=3)

        self.paned_window = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        self.paned_window.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 左侧树形目录
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

        # 右侧选项卡面板
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

        # 选项卡 1：配置
        self.tab_config = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_config, text="⚙️ 规则配置")
        ttk.Label(self.tab_config, text="模块描述:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.entry_desc = ttk.Entry(self.tab_config, width=50)
        self.entry_desc.grid(row=0, column=1, sticky=tk.EW, pady=5)

        def create_path_editor(row, label_text):
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

        self.txt_src = create_path_editor(1, "源文件 (src_patterns):")
        self.txt_inc = create_path_editor(2, "头文件 (inc_patterns):")
        self.txt_inc_dirs = create_path_editor(3, "编译包含路径 (inc_dirs):")

        ttk.Label(self.tab_config, text="依赖模块树:\n(双击节点勾选)").grid(row=4, column=0, sticky=tk.NW, pady=5)
        dep_frame = ttk.Frame(self.tab_config)
        dep_frame.grid(row=4, column=1, sticky=tk.EW, pady=5)
        dep_scroll = ttk.Scrollbar(dep_frame)
        dep_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree_deps = ttk.Treeview(dep_frame, yscrollcommand=dep_scroll.set, selectmode="browse", height=7, show="tree")
        self.tree_deps.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dep_scroll.config(command=self.tree_deps.yview)
        self.tree_deps.bind("<Double-1>", self.on_dep_tree_click)
        self.tree_deps.tag_configure("error", foreground="#d32f2f", font=("Helvetica", 9, "bold"))

        self.tab_config.rowconfigure(4, weight=1) 
        self.tab_config.columnconfigure(1, weight=1)
        self.btn_apply = ttk.Button(self.tab_config, text="✔️ 手动应用暂存 (切换时自动保存)", command=lambda: self.apply_current_module(show_msg=True))
        self.btn_apply.grid(row=5, column=1, sticky=tk.E, pady=10)

        # 选项卡 2：预览
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

        self.list_preview_src = create_preview_box(pw_preview, "解析出的源文件:", "blue")
        self.list_preview_inc = create_preview_box(pw_preview, "解析出的头文件:", "green")
        self.list_preview_inc_dirs = create_preview_box(pw_preview, "解析出的包含目录 (-I):", "#cc6600")

        self.clear_right_panel() 

    # ================= 核心操作逻辑 =================
    def load_data(self):
        try:
            self.model.load()
        except Exception as e:
            messagebox.showerror("读取错误", f"解析失败: {e}")
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
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text="⚙️ _internal", tags=("internal",))
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
            
            sub_count = 0
            all_src, all_inc, all_inc_dirs, all_deps = set(), set(), set(), set()
            for k, v in self.registry["modules"][root_name].items():
                if k.startswith(folder_path + '|'):
                    sub_count += 1
                    all_src.update(v.get("src_patterns", []))
                    all_inc.update(v.get("inc_patterns", []))
                    all_inc_dirs.update(v.get("inc_dirs", []))
                    all_deps.update(v.get("depends_on", []))
            
            self.entry_desc.delete(0, tk.END)
            self.entry_desc.insert(0, f"本文件夹包含 {sub_count} 个独立模块/配置。")
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

    # ================= 依赖树控制 =================
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
            self.tree_deps.insert("", 0, iid=missing_root, text="⚠️ 缺失/失效的关联 (建议取消勾选)", open=True, tags=("error",))
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
            else: self.tree_deps.item(node, text=f"☐ {base_text}")
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

        if root_name not in self.registry["modules"] or mod_key not in self.registry["modules"][root_name]:
            self.current_selected_node = None
            return

        target_mod = self.registry["modules"][root_name][mod_key]
        target_mod["description"] = self.entry_desc.get().strip()
        target_mod["src_patterns"] = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_patterns"] = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_dirs"] = [f.strip() for f in self.txt_inc_dirs.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["depends_on"] = self.get_all_checked_deps() 

        self.mark_dirty()
        if show_msg: messagebox.showinfo("暂存成功", f"模块 '{mod_key}' 的修改已暂存内存。")
        if self.notebook.index(self.notebook.select()) == 1: self.update_preview()

    def save_json_to_disk(self, event=None):
        if self.current_selected_node:
            self.apply_current_module(show_msg=False)
        try:
            self.model.save()
            self.mark_clean()
            messagebox.showinfo("成功", "✅ 配置已保存至磁盘！", icon="info")
        except Exception as e:
            messagebox.showerror("保存失败", f"写入 JSON 发生错误:\n{e}")

    # ================= 智能浏览、多选与路径转换 =================
    def popup_macro_menu(self, event, target_widget):
        menu = tk.Menu(self.root, tearoff=0)
        macros = self.registry.get("macros", {})
        if not macros: menu.add_command(label="(暂无可用宏)")
        else:
            for mac in sorted(macros.keys()):
                menu.add_command(label=f"${{{mac}}}", command=lambda m=mac: self.insert_macro_to_widget(target_widget, m))
        menu.post(event.x_root, event.y_root)

    def insert_macro_to_widget(self, widget, macro_name):
        content = widget.get(1.0, tk.END).strip()
        widget.insert(tk.END, ("\n" if content else "") + f"${{{macro_name}}}/")

    def browse_path(self, target_widget, is_dir=False):
        """核心升级：支持文件多选批量解析"""
        if not self.current_selected_node: return
        initial_dir = Path(self.gmp_location)
        
        chosen_paths = []
        if is_dir: 
            chosen = filedialog.askdirectory(initialdir=initial_dir)
            if chosen: chosen_paths.append(chosen)
        else: 
            # 【重要】启用 askopenfilenames，返回的是元组
            chosen_tuple = filedialog.askopenfilenames(initialdir=initial_dir)
            if chosen_tuple: chosen_paths = list(chosen_tuple)
            
        if not chosen_paths: return
        
        # 预先按照路径长度降序排列宏，以保证最长匹配优先
        macros = sorted(self.registry.get("macros", {}).items(), key=lambda item: len(item[1]), reverse=True)
        
        final_strings = []
        
        # 批量处理所有被选中的文件/文件夹
        for path_str in chosen_paths:
            chosen_path = Path(path_str)
            final_str = ""
            
            # 宏替换逻辑
            for mac, val in macros:
                if chosen_path.is_relative_to(Path(val)):
                    rel = chosen_path.relative_to(Path(val)).as_posix()
                    final_str = f"${{{mac}}}/{rel}" if rel != "." else f"${{{mac}}}"
                    break
                    
            if not final_str: 
                final_str = chosen_path.as_posix()

            if is_dir:
                if target_widget in (self.txt_src, self.txt_inc):
                    final_str = final_str.rstrip('/') + "/*"
                    
            final_strings.append(final_str)

        # 批量写入文本框
        content = target_widget.get(1.0, tk.END).strip()
        prefix = "\n" if content else ""
        target_widget.insert(tk.END, prefix + "\n".join(final_strings))
        self.mark_dirty() # 自动标记为修改状态

    # ================= 宏管理器 (Macro Manager) =================
    def open_macro_manager(self):
        top = tk.Toplevel(self.root)
        top.title("全局路径宏管理")
        top.geometry("600x400")
        top.grab_set()

        tree = ttk.Treeview(top, columns=("Value",), selectmode="browse")
        tree.heading("#0", text="宏变量名")
        tree.heading("Value", text="绝对路径")
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
            name = simpledialog.askstring("新增宏", "宏名:", parent=top)
            if name and (val := filedialog.askdirectory()):
                self.registry["macros"][name] = Path(val).as_posix()
                self.mark_dirty()
                refresh_macros()

        def del_macro():
            sel = tree.selection()
            if not sel: return
            name = tree.item(sel[0], "text")
            if name == "GMP_PRO_LOCATION": return messagebox.showwarning("禁止", "核心宏不可删", parent=top)
            if messagebox.askyesno("删除", f"确定删除宏 '{name}' 吗？", parent=top):
                del self.registry["macros"][name]
                self.mark_dirty()
                refresh_macros()

        ttk.Button(btn_frame, text="➕ 新增", command=add_macro).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="❌ 删除", command=del_macro).pack(side=tk.LEFT)

    # ================= 实时匹配预览 (Live Preview) =================
    def on_tab_changed(self, event):
        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    def update_preview(self):
        for lb in (self.list_preview_src, self.list_preview_inc, self.list_preview_inc_dirs):
            lb.delete(0, tk.END)
        if not self.current_selected_node: return
        
        src_files = self.model.resolve_paths([f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()])
        inc_files = self.model.resolve_paths([f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()])
        dirs_files = self.model.resolve_paths([f.strip() for f in self.txt_inc_dirs.get(1.0, tk.END).split('\n') if f.strip()], is_dir_mode=True)

        for f in (src_files or ["⚠️ 未匹配到源文件"]): self.list_preview_src.insert(tk.END, f)
        for f in (inc_files or ["⚠️ 未匹配到头文件"]): self.list_preview_inc.insert(tk.END, f)
        for d in (dirs_files or ["ℹ️ 无编译器依赖目录"]): self.list_preview_inc_dirs.insert(tk.END, d)

    # =========== 增删安全拦截 ===========
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

        mod_key = simpledialog.askstring("新增", f"在 '{root_name}' 下新增:", initialvalue=default_prefix)
        if mod_key and mod_key.strip():
            mod_key = mod_key.strip()
            if root_name not in self.registry["modules"]: self.registry["modules"][root_name] = {}
            parts = mod_key.split('|')
            for i in range(1, len(parts)):
                parent_key = '|'.join(parts[:i])
                if parent_key in self.registry["modules"][root_name]:
                    if messagebox.askyesno("转换", "是否将此模块转换为文件夹？"):
                        if self.current_selected_node == (root_name, parent_key):
                            self.current_selected_node = None 
                        old_data = self.registry["modules"][root_name].pop(parent_key)
                        self.registry["modules"][root_name][f"{parent_key}|_internal"] = old_data
                    else: return 
            if mod_key in self.registry["modules"][root_name]: return
            
            self.registry["modules"][root_name][mod_key] = {
                "type": "module", "description": "", "src_patterns": [], "inc_patterns": [], "inc_dirs": [], "depends_on": []
            }
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
            if messagebox.askyesno("危险", f"删除文件夹 '{folder_path}' 及其所有内容？"):
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
            if messagebox.askyesno("确认", f"删除 '{mod_key}'？"):
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