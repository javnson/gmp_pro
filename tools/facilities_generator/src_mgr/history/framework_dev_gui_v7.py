import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
from pathlib import Path

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 核心框架资产管理器 V6 (缓冲沙盒与完整路径依赖)")
        self.root.geometry("1100x800")
        
        # 拦截关闭事件，防止未保存的数据丢失
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
        self.is_dirty = False # 记录是否发生过修改但未保存
        
        self.build_ui()
        self.load_data()

    def mark_dirty(self):
        """标记配置已被修改（未保存至磁盘）"""
        self.is_dirty = True
        self.root.title("GMP 核心框架资产管理器 V6 (缓冲沙盒与完整路径依赖) - *未保存的修改*")
        self.btn_global_save.config(state=tk.NORMAL)

    def mark_clean(self):
        """标记配置已与磁盘同步"""
        self.is_dirty = False
        self.root.title("GMP 核心框架资产管理器 V6 (缓冲沙盒与完整路径依赖)")
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

        # 辅助函数：创建带 浏览 和 插入宏 按钮的输入区域
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
            
            # 插入宏按钮 (弹出菜单)
            btn_macro = ttk.Button(lbl_frame, text="📌 插入宏")
            btn_macro.pack(anchor=tk.W, fill=tk.X, pady=2)
            btn_macro.bind("<Button-1>", lambda event, w=txt_widget: self.popup_macro_menu(event, w))
            
            return txt_widget

        self.txt_src = create_path_editor(1, "源文件 (src_patterns):", "src")
        self.txt_inc = create_path_editor(2, "头文件 (inc_patterns):", "inc")
        self.txt_inc_dirs = create_path_editor(3, "编译依赖包含路径\n(inc_dirs - 给编译器):", "inc_dirs")

        # 依赖区
        ttk.Label(self.tab_config, text="依赖模块 (depends_on):\n(Ctrl 多选)").grid(row=4, column=0, sticky=tk.NW, pady=5)
        dep_frame = ttk.Frame(self.tab_config)
        dep_frame.grid(row=4, column=1, sticky=tk.EW, pady=5)
        dep_scroll = ttk.Scrollbar(dep_frame)
        dep_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_deps = tk.Listbox(dep_frame, selectmode=tk.MULTIPLE, yscrollcommand=dep_scroll.set, height=5, exportselection=False)
        self.listbox_deps.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dep_scroll.config(command=self.listbox_deps.yview)

        self.tab_config.rowconfigure(4, weight=1) 
        self.tab_config.columnconfigure(1, weight=1)
        self.btn_apply = ttk.Button(self.tab_config, text="✔️ 应用此节点的修改 (暂存至内存)", command=self.apply_current_module)
        self.btn_apply.grid(row=5, column=1, sticky=tk.E, pady=10)

        # ---------- 选项卡 2：匹配文件预览 ----------
        self.tab_preview = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_preview, text="👁️ 匹配文件预览 (实时)")
        
        # 使用 PanedWindow 划分三块预览区域
        pw_preview = ttk.PanedWindow(self.tab_preview, orient=tk.VERTICAL)
        pw_preview.pack(fill=tk.BOTH, expand=True)

        def create_preview_box(parent, text, color):
            frame = ttk.Frame(parent)
            ttk.Label(frame, text=text, foreground=color).pack(anchor=tk.W)
            lb = tk.Listbox(frame, height=4, bg="#f9f9f9")
            lb.pack(fill=tk.BOTH, expand=True)
            parent.add(frame, weight=1)
            return lb

        self.list_preview_src = create_preview_box(pw_preview, "解析出的物理源文件 (.c / .cpp):", "blue")
        self.list_preview_inc = create_preview_box(pw_preview, "解析出的物理头文件 (.h / .hpp):", "green")
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
                
        # === 防呆机制与环境宏强制注入 ===
        if "macros" not in self.registry:
            self.registry["macros"] = {}
        # 永远保证系统宏的绝对正确性
        self.registry["macros"]["GMP_PRO_LOCATION"] = Path(self.gmp_location).as_posix()
            
        # 补全旧版本 JSON 中缺失的字段
        for root_name, modules in self.registry.get("modules", {}).items():
            for mod_data in modules.values():
                mod_data.setdefault("description", "")
                mod_data.setdefault("src_patterns", [])
                mod_data.setdefault("inc_patterns", [])
                mod_data.setdefault("inc_dirs", []) # 新增的编译包含路径
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
                    self.mark_dirty() # 因为自动补充了数据，标记为脏

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
            
            self.listbox_deps.delete(0, tk.END)
            for dep in sorted(list(all_deps)): self.listbox_deps.insert(tk.END, dep)
                
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

        self.listbox_deps.delete(0, tk.END)
        depends_on = mod_data.get("depends_on", [])
        for idx, available_mod in enumerate(self.all_available_modules):
            if available_mod == f"{root_name}|{mod_key}": continue
            self.listbox_deps.insert(tk.END, available_mod)
            if available_mod in depends_on:
                self.listbox_deps.selection_set(tk.END)

        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    def clear_right_panel(self):
        self.current_selected_node = None
        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text="-")
        self.entry_desc.delete(0, tk.END)
        self.txt_src.delete(1.0, tk.END)
        self.txt_inc.delete(1.0, tk.END)
        self.txt_inc_dirs.delete(1.0, tk.END)
        self.listbox_deps.delete(0, tk.END)
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

    def apply_current_module(self):
        """将右侧的修改只应用到内存中的 registry 字典"""
        if not self.current_selected_node: return
        root_name, mod_key = self.current_selected_node

        target_mod = self.registry["modules"][root_name][mod_key]
        target_mod["description"] = self.entry_desc.get().strip()
        target_mod["src_patterns"] = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_patterns"] = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_dirs"] = [f.strip() for f in self.txt_inc_dirs.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["depends_on"] = [self.listbox_deps.get(i) for i in self.listbox_deps.curselection()]

        self.mark_dirty()
        messagebox.showinfo("暂存成功", f"模块 '{mod_key}' 的修改已暂存。\n别忘了点击顶部的【保存全部更改】写入磁盘！")
        self.update_preview()

    def save_json_to_disk(self):
        """真正的全局写盘操作"""
        try:
            with open(self.json_path, 'w', encoding='utf-8') as f:
                json.dump(self.registry, f, indent=4, ensure_ascii=False)
            self.mark_clean()
            messagebox.showinfo("成功", "✅ 所有配置已成功永久保存至 JSON 磁盘文件中！")
        except Exception as e:
            messagebox.showerror("保存失败", f"写入 JSON 发生错误:\n{e}")

    # ================= 智能浏览、宏插入与路径转换 =================
    def popup_macro_menu(self, event, target_widget):
        """在鼠标点击处弹出系统的宏选择菜单"""
        menu = tk.Menu(self.root, tearoff=0)
        macros = self.registry.get("macros", {})
        if not macros:
            menu.add_command(label="(暂无可用宏)")
        else:
            for mac in sorted(macros.keys()):
                # 闭包绑定当前宏
                menu.add_command(label=f"${{{mac}}}", command=lambda m=mac: self.insert_macro_to_widget(target_widget, m))
        menu.post(event.x_root, event.y_root)

    def insert_macro_to_widget(self, widget, macro_name):
        # 如果是最后一行有内容，先回车换行，再插入宏
        content = widget.get(1.0, tk.END).strip()
        insert_str = f"${{{macro_name}}}/"
        if content:
            widget.insert(tk.END, "\n" + insert_str)
        else:
            widget.insert(tk.END, insert_str)

    def browse_path(self, target_widget, is_dir=False):
        if not self.current_selected_node: return
        
        # 默认起点：优先基于 GMP_PRO_LOCATION，而不是当前模块。
        initial_dir = Path(self.gmp_location)
            
        if is_dir:
            chosen = filedialog.askdirectory(initialdir=initial_dir)
        else:
            chosen = filedialog.askopenfilename(initialdir=initial_dir)
            
        if not chosen: return
        chosen_path = Path(chosen)
        
        # 智能宏替换匹配 (按照路径长度降序，优先匹配最长的子路径宏)
        macros = self.registry.get("macros", {})
        sorted_macros = sorted(macros.items(), key=lambda item: len(item[1]), reverse=True)
        
        final_str = ""
        for mac, val in sorted_macros:
            mac_path = Path(val)
            if chosen_path.is_relative_to(mac_path):
                rel = chosen_path.relative_to(mac_path).as_posix()
                final_str = f"${{{mac}}}/{rel}" if rel != "." else f"${{{mac}}}"
                break
                
        # 如果依然没匹配到任何宏，就保留绝对路径
        if not final_str:
            final_str = chosen_path.as_posix()

        # 根据规则种类，追加通配符
        if is_dir:
            if target_widget == self.txt_src:
                final_str = final_str.rstrip('/') + "/**/*.c"
            elif target_widget == self.txt_inc:
                final_str = final_str.rstrip('/') + "/**/*.h"
            # 对于 inc_dirs (包含目录)，我们不需要追加通配符，只要路径本身

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
                    # 对于目录依赖，只要它存在，就直接加入清单
                    if pat_obj.exists() and pat_obj.is_dir():
                        matched.add(pat_obj.as_posix())
                else:
                    # 对于文件抓取，执行 glob 通配符解析
                    if pat_obj.is_absolute():
                        anchor = pat_obj.anchor
                        rest = str(pat_obj.relative_to(anchor))
                        for f in Path(anchor).glob(rest):
                            if f.is_file(): matched.add(f.as_posix())
            return sorted(list(matched))

        # 渲染 SRC 预览
        src_files = resolve_and_glob(src_patterns)
        if not src_files: self.list_preview_src.insert(tk.END, "⚠️ 未匹配到源文件")
        for f in src_files: self.list_preview_src.insert(tk.END, f)

        # 渲染 INC 预览
        inc_files = resolve_and_glob(inc_patterns)
        if not inc_files: self.list_preview_inc.insert(tk.END, "⚠️ 未匹配到头文件")
        for f in inc_files: self.list_preview_inc.insert(tk.END, f)
            
        # 渲染 INC DIRS 预览
        dirs_files = resolve_and_glob(inc_dirs_patterns, is_dir_mode=True)
        if not dirs_files: self.list_preview_inc_dirs.insert(tk.END, "ℹ️ 无特定的编译器依赖目录")
        for d in dirs_files: self.list_preview_inc_dirs.insert(tk.END, d)

    # =========== 增删逻辑 (全部修改为调用 self.mark_dirty) ===========
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
            
            # 防呆：创建时给足所有默认属性
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