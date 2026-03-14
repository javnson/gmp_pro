import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
from pathlib import Path

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 核心框架资产管理器 V5 (宏支持与实时预览)")
        self.root.geometry("1100x750")

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        self.json_path = Path(__file__).parent / "gmp_framework_dic.json"
        
        self.registry = {"roots": [], "modules": {}, "macros": {}}
        self.current_selected_node = None 
        self.all_available_modules = [] 
        
        self.build_ui()
        self.load_data()

    def build_ui(self):
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
        self.tree.heading("#0", text="框架目录树", anchor=tk.W)
        self.tree.bind("<<TreeviewSelect>>", self.on_tree_select)

        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="➕ 新增子模块", command=self.add_module).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="❌ 删除选中", command=self.delete_node).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(left_frame, text="⚙️ 全局路径宏管理", command=self.open_macro_manager).pack(fill=tk.X, pady=2)

        # ================= 右侧：选项卡面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=5)
        self.paned_window.add(right_frame, weight=3)

        # 顶部始终显示的标识栏
        top_info_frame = ttk.Frame(right_frame)
        top_info_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Label(top_info_frame, text="当前选中节点:").pack(side=tk.LEFT)
        self.lbl_module_id = ttk.Label(top_info_frame, font=("Helvetica", 11, "bold"), foreground="blue", text="-")
        self.lbl_module_id.pack(side=tk.LEFT, padx=10)

        # 选项卡控件 (Notebook)
        self.notebook = ttk.Notebook(right_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # 绑定选项卡切换事件，切换到预览时自动刷新
        self.notebook.bind("<<NotebookTabChanged>>", self.on_tab_changed)

        # ---------- 选项卡 1：规则配置 ----------
        self.tab_config = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_config, text="⚙️ 规则配置")

        ttk.Label(self.tab_config, text="模块描述:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.entry_desc = ttk.Entry(self.tab_config, width=50)
        self.entry_desc.grid(row=0, column=1, sticky=tk.EW, pady=5)

        # Source 规则区
        src_lbl_frame = ttk.Frame(self.tab_config)
        src_lbl_frame.grid(row=1, column=0, sticky=tk.NW, pady=5)
        ttk.Label(src_lbl_frame, text="源文件 (src_patterns):").pack(anchor=tk.W)
        ttk.Button(src_lbl_frame, text="📄 添文件", width=8, command=lambda: self.browse_path(self.txt_src, is_dir=False)).pack(anchor=tk.W, pady=2)
        ttk.Button(src_lbl_frame, text="📁 添目录", width=8, command=lambda: self.browse_path(self.txt_src, is_dir=True)).pack(anchor=tk.W)
        
        self.txt_src = tk.Text(self.tab_config, height=5, width=40)
        self.txt_src.grid(row=1, column=1, sticky=tk.EW, pady=5)

        # Header 规则区
        inc_lbl_frame = ttk.Frame(self.tab_config)
        inc_lbl_frame.grid(row=2, column=0, sticky=tk.NW, pady=5)
        ttk.Label(inc_lbl_frame, text="头文件 (inc_patterns):").pack(anchor=tk.W)
        ttk.Button(inc_lbl_frame, text="📄 添文件", width=8, command=lambda: self.browse_path(self.txt_inc, is_dir=False)).pack(anchor=tk.W, pady=2)
        ttk.Button(inc_lbl_frame, text="📁 添目录", width=8, command=lambda: self.browse_path(self.txt_inc, is_dir=True)).pack(anchor=tk.W)

        self.txt_inc = tk.Text(self.tab_config, height=5, width=40)
        self.txt_inc.grid(row=2, column=1, sticky=tk.EW, pady=5)

        # 依赖区
        ttk.Label(self.tab_config, text="底层依赖 (depends_on):\n(按住 Ctrl 可多选)").grid(row=3, column=0, sticky=tk.NW, pady=5)
        dep_frame = ttk.Frame(self.tab_config)
        dep_frame.grid(row=3, column=1, sticky=tk.EW, pady=5)
        dep_scroll = ttk.Scrollbar(dep_frame)
        dep_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        self.listbox_deps = tk.Listbox(dep_frame, selectmode=tk.MULTIPLE, yscrollcommand=dep_scroll.set, height=6, exportselection=False)
        self.listbox_deps.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dep_scroll.config(command=self.listbox_deps.yview)

        self.tab_config.rowconfigure(4, weight=1) 
        self.tab_config.columnconfigure(1, weight=1)
        self.btn_save = ttk.Button(self.tab_config, text="💾 保存并更新配置", command=self.save_current_module)
        self.btn_save.grid(row=5, column=1, sticky=tk.E, pady=10)

        # ---------- 选项卡 2：匹配文件预览 ----------
        self.tab_preview = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(self.tab_preview, text="👁️ 匹配文件预览 (实时)")
        
        ttk.Label(self.tab_preview, text="解析到的源文件 (.c / .cpp):", foreground="blue").pack(anchor=tk.W)
        self.list_preview_src = tk.Listbox(self.tab_preview, height=8, bg="#f9f9f9")
        self.list_preview_src.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        ttk.Label(self.tab_preview, text="解析到的头文件 (.h / .hpp / .inl):", foreground="green").pack(anchor=tk.W)
        self.list_preview_inc = tk.Listbox(self.tab_preview, height=8, bg="#f9f9f9")
        self.list_preview_inc.pack(fill=tk.BOTH, expand=True)

        self.clear_right_panel() 

    # ================= 数据加载与树形逻辑 =================
    def load_data(self):
        if not self.json_path.exists():
            self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}, "macros": {}}
            self.save_json()
        else:
            try:
                with open(self.json_path, 'r', encoding='utf-8') as f:
                    self.registry = json.load(f)
            except Exception as e:
                messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
                return
                
        if "macros" not in self.registry:
            self.registry["macros"] = {}
            
        self.ensure_internals()
        self.refresh_tree()

    def ensure_internals(self):
        changed = False
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
                        "src_patterns": [], "inc_patterns": [], "depends_on": []
                    }
                    changed = True
        if changed: self.save_json()

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
            # 文件夹汇总逻辑，在主界面显示文本即可，禁用保存
            _, root_name, folder_path = node_id.split('|', 2)
            self.current_selected_node = None 
            self.set_right_panel_state(tk.NORMAL)
            self.lbl_module_id.config(text=f"📁 {root_name} | {folder_path.replace('|', ' / ')}  (汇总)")
            self.entry_desc.delete(0, tk.END)
            self.entry_desc.insert(0, "这是一个目录分类，请点击其内部具体模块或 _internal 进行配置。")
            self.txt_src.delete(1.0, tk.END)
            self.txt_inc.delete(1.0, tk.END)
            self.listbox_deps.delete(0, tk.END)
            self.set_right_panel_state(tk.DISABLED)
            
            # 清空预览
            self.list_preview_src.delete(0, tk.END)
            self.list_preview_inc.delete(0, tk.END)
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

        self.listbox_deps.delete(0, tk.END)
        depends_on = mod_data.get("depends_on", [])
        for idx, available_mod in enumerate(self.all_available_modules):
            if available_mod == f"{root_name}|{mod_key}": continue
            self.listbox_deps.insert(tk.END, available_mod)
            if available_mod in depends_on:
                self.listbox_deps.selection_set(tk.END)

        # 触发预览刷新
        if self.notebook.index(self.notebook.select()) == 1:
            self.update_preview()

    def clear_right_panel(self):
        self.current_selected_node = None
        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text="-")
        self.entry_desc.delete(0, tk.END)
        self.txt_src.delete(1.0, tk.END)
        self.txt_inc.delete(1.0, tk.END)
        self.listbox_deps.delete(0, tk.END)
        self.list_preview_src.delete(0, tk.END)
        self.list_preview_inc.delete(0, tk.END)
        self.set_right_panel_state(tk.DISABLED)

    def set_right_panel_state(self, state):
        self.entry_desc.config(state=state)
        self.txt_src.config(state=state)
        self.txt_inc.config(state=state)
        self.btn_save.config(state=state)

    def save_current_module(self):
        if not self.current_selected_node: return
        root_name, mod_key = self.current_selected_node

        target_mod = self.registry["modules"][root_name][mod_key]
        target_mod["description"] = self.entry_desc.get().strip()
        target_mod["src_patterns"] = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["inc_patterns"] = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        target_mod["depends_on"] = [self.listbox_deps.get(i) for i in self.listbox_deps.curselection()]

        self.save_json()
        messagebox.showinfo("成功", f"配置已保存！")
        
        # 实时刷新预览
        self.update_preview()

    # ================= 智能浏览与路径推导 =================
    def browse_path(self, target_widget, is_dir=False):
        if not self.current_selected_node: return
        root_name, mod_key = self.current_selected_node
        
        # 默认起点：当前模块物理目录
        mod_physical_dir = Path(self.gmp_location) / root_name / mod_key.replace('|', '/')
        if not mod_physical_dir.exists():
            mod_physical_dir = Path(self.gmp_location) # 退而求其次
            
        if is_dir:
            chosen = filedialog.askdirectory(initialdir=mod_physical_dir)
        else:
            chosen = filedialog.askopenfilename(initialdir=mod_physical_dir)
            
        if not chosen: return
        chosen_path = Path(chosen)
        
        final_str = ""
        # 1. 尝试匹配宏替换
        for mac, val in self.registry.get("macros", {}).items():
            mac_path = Path(val)
            if chosen_path.is_relative_to(mac_path):
                rel = chosen_path.relative_to(mac_path).as_posix()
                final_str = f"${{{mac}}}/{rel}" if rel != "." else f"${{{mac}}}"
                break
                
        # 2. 如果没匹配到宏，尝试相对路径转换 (相对于当前模块物理目录)
        if not final_str:
            target_mod_dir = Path(self.gmp_location) / root_name / mod_key.replace('|', '/')
            try:
                final_str = os.path.relpath(chosen_path, target_mod_dir).replace('\\', '/')
            except ValueError:
                # 跨盘符无法转换相对路径，保留绝对路径
                final_str = chosen_path.as_posix()

        # 如果选的是文件夹，自动追加通配符
        if is_dir:
            if target_widget == self.txt_src:
                final_str = final_str.rstrip('/') + "/**/*.c"
            else:
                final_str = final_str.rstrip('/') + "/**/*.h"

        # 写入文本框
        target_widget.insert(tk.END, ("\n" if target_widget.get(1.0, tk.END).strip() else "") + final_str)

    # ================= 宏管理器 (Macro Manager) =================
    def open_macro_manager(self):
        top = tk.Toplevel(self.root)
        top.title("全局路径宏管理")
        top.geometry("600x400")
        top.transient(self.root)
        top.grab_set()

        ttk.Label(top, text="定义路径宏，以便在配置通配符时使用 (如 ${C2000WARE})").pack(pady=10)

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
            self.save_json()
            refresh_macros()

        def del_macro():
            sel = tree.selection()
            if not sel: return
            name = tree.item(sel[0], "text")
            if messagebox.askyesno("删除", f"确定删除宏 '{name}' 吗？", parent=top):
                del self.registry["macros"][name]
                self.save_json()
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

        if not self.current_selected_node: return
        root_name, mod_key = self.current_selected_node
        
        # 实时读取文本框里的规则（不需要先保存）
        src_patterns = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        inc_patterns = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]

        mod_dir = Path(self.gmp_location) / root_name / mod_key.replace('|', '/')

        def resolve_and_glob(patterns):
            matched = set()
            for pat in patterns:
                resolved_pat = pat
                # 1. 替换宏
                for mac, val in self.registry.get("macros", {}).items():
                    resolved_pat = resolved_pat.replace(f"${{{mac}}}", val)
                
                # 2. 执行物理匹配
                pat_obj = Path(resolved_pat)
                if pat_obj.is_absolute():
                    # 绝对路径带通配符：利用 anchor 解析
                    anchor = pat_obj.anchor
                    rest = str(pat_obj.relative_to(anchor))
                    for f in Path(anchor).glob(rest):
                        if f.is_file(): matched.add(f.as_posix())
                else:
                    # 相对路径带通配符 (相对于当前模块目录)
                    if mod_dir.exists():
                        for f in mod_dir.glob(resolved_pat):
                            if f.is_file(): matched.add(f.as_posix())
            return sorted(list(matched))

        # 渲染 SRC 预览
        src_files = resolve_and_glob(src_patterns)
        if not src_files:
            self.list_preview_src.insert(tk.END, "⚠️ 未匹配到任何源文件 (目录不存在或通配符无命中)")
        for f in src_files: self.list_preview_src.insert(tk.END, f)

        # 渲染 INC 预览
        inc_files = resolve_and_glob(inc_patterns)
        if not inc_files:
            self.list_preview_inc.insert(tk.END, "⚠️ 未匹配到任何头文件")
        for f in inc_files: self.list_preview_inc.insert(tk.END, f)

    # =========== (保留原有的增删节点与保存逻辑，无改变) ===========
    def add_module(self):
        # [逻辑同上一个版本，保留了包转换与前缀补全]
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
                    msg = (f"检测到冲突：节点 '{parent_key}' 是独立模块。\n是否迁移为 '{parent_key}|_internal' 并升级为文件夹？")
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
                "type": "module", "description": "新模块", "src_patterns": [], "inc_patterns": [], "depends_on": []
            }
            self.save_json()
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
                self.save_json()
                self.ensure_internals()
                self.refresh_tree()
                self.clear_right_panel()
            return
        elif node_id.startswith("MOD|"):
            _, root_name, mod_key = node_id.split('|', 2)
            if messagebox.askyesno("确认", f"确定要彻底删除 '{mod_key}' 吗？"):
                del self.registry["modules"][root_name][mod_key]
                self._check_and_rollback_parent(root_name, mod_key)
                self.save_json()
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

    def save_json(self):
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkDevGUI(root)
    root.mainloop()