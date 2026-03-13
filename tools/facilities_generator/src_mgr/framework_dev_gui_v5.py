import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from pathlib import Path

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 核心框架资产管理器 V4 (包机制与智能汇总)")
        self.root.geometry("1050x700")

        self.gmp_location = os.environ.get('GMP_PRO_LOCATION')
        if not self.gmp_location:
            messagebox.showerror("错误", "未找到环境变量 GMP_PRO_LOCATION")
            self.root.destroy()
            return
            
        self.json_path = Path(__file__).parent / "gmp_framework_dic.json"
        
        self.registry = {"roots": [], "modules": {}}
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

        # ================= 右侧：高级编辑面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=2)

        ttk.Label(right_frame, text="节点标识:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.lbl_module_id = ttk.Label(right_frame, font=("Helvetica", 10, "bold"), foreground="blue", text="-")
        self.lbl_module_id.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Label(right_frame, text="节点描述:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.entry_desc = ttk.Entry(right_frame, width=40)
        self.entry_desc.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="源文件 (src_patterns):").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_src = tk.Text(right_frame, height=4, width=40)
        self.txt_src.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="头文件 (inc_patterns):").grid(row=3, column=0, sticky=tk.NW, pady=5)
        self.txt_inc = tk.Text(right_frame, height=4, width=40)
        self.txt_inc.grid(row=3, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="底层依赖 (depends_on):\n(按住 Ctrl 可多选)").grid(row=4, column=0, sticky=tk.NW, pady=5)
        
        dep_frame = ttk.Frame(right_frame)
        dep_frame.grid(row=4, column=1, sticky=tk.EW, pady=5)
        
        dep_scroll = ttk.Scrollbar(dep_frame)
        dep_scroll.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.listbox_deps = tk.Listbox(dep_frame, selectmode=tk.MULTIPLE, yscrollcommand=dep_scroll.set, height=6, exportselection=False)
        self.listbox_deps.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        dep_scroll.config(command=self.listbox_deps.yview)

        right_frame.rowconfigure(5, weight=1) 
        self.btn_save = ttk.Button(right_frame, text="💾 保存并更新配置", command=self.save_current_module)
        self.btn_save.grid(row=6, column=1, sticky=tk.E, pady=10)

        right_frame.columnconfigure(1, weight=1)
        self.clear_right_panel() 

    # ================= 核心逻辑：数据加载与智能处理 =================
    def load_data(self):
        if not self.json_path.exists():
            self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}}
            self.save_json()
        else:
            try:
                with open(self.json_path, 'r', encoding='utf-8') as f:
                    self.registry = json.load(f)
            except Exception as e:
                messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
                return
        
        # 加载完成后，扫描并补全所有的 _internal 节点
        self.ensure_internals()
        self.refresh_tree()

    def ensure_internals(self):
        """遍历所有模块，确保所有具备子模块的'文件夹'都拥有一个 _internal 配置"""
        changed = False
        for root_name, modules in self.registry.get("modules", {}).items():
            folders = set()
            # 提取出所有的文件夹前缀
            for mod_key in modules.keys():
                parts = mod_key.split('|')
                for i in range(1, len(parts)):
                    folders.add('|'.join(parts[:i]))
                    
            # 检查每个文件夹是否拥有 _internal
            for folder in folders:
                internal_key = f"{folder}|_internal"
                if internal_key not in modules:
                    modules[internal_key] = {
                        "type": "module",
                        "description": "自动生成的包基础配置",
                        "src_patterns": [],
                        "inc_patterns": [],
                        "depends_on": []
                    }
                    changed = True
        
        if changed:
            self.save_json()

    def update_available_modules(self):
        self.all_available_modules = []
        for root_name in self.registry.get("roots", []):
            modules = self.registry.get("modules", {}).get(root_name, {})
            for mod_key in modules.keys():
                self.all_available_modules.append(f"{root_name}|{mod_key}")
        self.all_available_modules.sort()

    def refresh_tree(self):
        for item in self.tree.get_children():
            self.tree.delete(item)

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
                        self.tree.insert(parent_id, tk.END, iid=mod_id, text="⚙️ _internal (包基础配置)", tags=("internal",))
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
            
        # ================= 文件夹汇总逻辑 =================
        elif node_id.startswith("FOLDER|"):
            _, root_name, folder_path = node_id.split('|', 2)
            self.current_selected_node = None # 禁止保存操作
            
            all_src = set()
            all_inc = set()
            all_deps = set()
            
            modules = self.registry["modules"][root_name]
            sub_modules_count = 0
            
            # 汇总该文件夹下的所有项
            for k, v in modules.items():
                if k.startswith(folder_path + '|'):
                    sub_modules_count += 1
                    all_src.update(v.get("src_patterns", []))
                    all_inc.update(v.get("inc_patterns", []))
                    all_deps.update(v.get("depends_on", []))
                    
            self.set_right_panel_state(tk.NORMAL)
            self.lbl_module_id.config(text=f"📁 {root_name} | {folder_path.replace('|', ' / ')}  (文件夹汇总)")
            
            self.entry_desc.delete(0, tk.END)
            self.entry_desc.insert(0, f"本文件夹及子目录共包含 {sub_modules_count} 个独立模块/配置")
            
            self.txt_src.delete(1.0, tk.END)
            self.txt_src.insert(tk.END, "\n".join(sorted(list(all_src))))
            
            self.txt_inc.delete(1.0, tk.END)
            self.txt_inc.insert(tk.END, "\n".join(sorted(list(all_inc))))
            
            self.listbox_deps.delete(0, tk.END)
            for dep in sorted(list(all_deps)):
                self.listbox_deps.insert(tk.END, dep)
                
            # 设置为只读状态
            self.entry_desc.config(state=tk.DISABLED)
            self.txt_src.config(state=tk.DISABLED)
            self.txt_inc.config(state=tk.DISABLED)
            self.btn_save.config(state=tk.DISABLED)
            return
            
        # ================= 单个模块展示逻辑 =================
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

    def clear_right_panel(self):
        self.current_selected_node = None
        self.set_right_panel_state(tk.NORMAL)
        self.lbl_module_id.config(text="-")
        self.entry_desc.delete(0, tk.END)
        self.txt_src.delete(1.0, tk.END)
        self.txt_inc.delete(1.0, tk.END)
        self.listbox_deps.delete(0, tk.END)
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

    def add_module(self):
        selected = self.tree.selection()
        if not selected: return
            
        node_id = selected[0]
        default_prefix = ""
        
        if node_id.startswith("ROOT|"):
            root_name = node_id.split('|', 1)[1]
        elif node_id.startswith("FOLDER|"):
            _, root_name, folder_path = node_id.split('|', 2)
            default_prefix = folder_path + "|"
        elif node_id.startswith("MOD|"):
            _, root_name, mod_key = node_id.split('|', 2)
            parts = mod_key.split('|')
            if len(parts) > 1:
                default_prefix = "|".join(parts[:-1]) + "|"

        mod_key = simpledialog.askstring("新增子模块", f"在 '{root_name}' 下新增:\n(多级请用 '|' 分隔)", initialvalue=default_prefix)
        
        if mod_key and mod_key.strip():
            mod_key = mod_key.strip()
            
            if root_name not in self.registry["modules"]:
                self.registry["modules"][root_name] = {}
                
            # 智能转换：若父节点为普通模块，提醒升级为 _internal 包结构
            parts = mod_key.split('|')
            for i in range(1, len(parts)):
                parent_key = '|'.join(parts[:i])
                if parent_key in self.registry["modules"][root_name]:
                    msg = (f"检测到冲突：节点 '{parent_key}' 是独立模块。\n"
                           f"是否允许系统自动将 '{parent_key}' 迁移为 '{parent_key}|_internal'，并升级为文件夹包？")
                    if messagebox.askyesno("智能转换", msg):
                        old_data = self.registry["modules"][root_name].pop(parent_key)
                        self.registry["modules"][root_name][f"{parent_key}|_internal"] = old_data
                    else:
                        return 
            
            for existing_key in self.registry["modules"][root_name].keys():
                if existing_key.startswith(mod_key + '|'):
                    messagebox.showwarning("冲突", f"'{mod_key}' 已是文件夹名称。\n如需配置基础属性，请配置它的 '_internal' 节点。")
                    return

            if mod_key in self.registry["modules"][root_name]: return
            
            self.registry["modules"][root_name][mod_key] = {
                "type": "module", "description": "新模块", "src_patterns": [], "inc_patterns": [], "depends_on": []
            }
            
            self.save_json()
            # 补齐可能新生成的文件夹的 _internal
            self.ensure_internals()
            self.refresh_tree()
            self.tree.selection_set(f"MOD|{root_name}|{mod_key}")

    def delete_node(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        
        if node_id.startswith("ROOT|"):
            return
            
        elif node_id.startswith("FOLDER|"):
            _, root_name, folder_path = node_id.split('|', 2)
            if messagebox.askyesno("危险确认", f"确定要删除文件夹 '{folder_path}' 及其【所有】配置吗？"):
                keys_to_delete = [k for k in self.registry["modules"][root_name].keys() 
                                  if k == folder_path or k.startswith(folder_path + '|')]
                for k in keys_to_delete:
                    del self.registry["modules"][root_name][k]
                    
                # 【机制触发】：检查父文件夹是否需要回退
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
                
                # 【机制触发】：检查当前被删除模块的父文件夹是否需要回退
                self._check_and_rollback_parent(root_name, mod_key)

                self.save_json()
                self.ensure_internals()
                self.refresh_tree()
                self.clear_right_panel()

    def _check_and_rollback_parent(self, root_name, deleted_path):
        """核心机制：如果父文件夹下只剩 _internal，则将其退回为普通独立模块"""
        parts = deleted_path.split('|')
        if len(parts) > 1:
            parent_folder = '|'.join(parts[:-1])
            # 查找该父文件夹下还剩哪些模块
            remaining = [k for k in self.registry["modules"][root_name].keys() if k.startswith(parent_folder + '|')]
            
            # 如果只剩唯一的一个，且正是 _internal
            if len(remaining) == 1 and remaining[0] == f"{parent_folder}|_internal":
                internal_data = self.registry["modules"][root_name].pop(remaining[0])
                self.registry["modules"][root_name][parent_folder] = internal_data
                messagebox.showinfo("智能回退", f"文件夹 '{parent_folder}' 已无其他子模块，系统已将其 _internal 配置退回为独立模块。")

    def save_json(self):
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkDevGUI(root)
    root.mainloop()