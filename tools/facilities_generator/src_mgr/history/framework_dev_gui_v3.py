import os
import json
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog
from pathlib import Path

class FrameworkDevGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("GMP 核心框架资产管理器 (多级树形版)")
        self.root.geometry("1000x700")

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

        # 左侧底部操作按钮
        btn_frame = ttk.Frame(left_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        ttk.Button(btn_frame, text="➕ 新增子模块", command=self.add_module).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        ttk.Button(btn_frame, text="❌ 删除选中", command=self.delete_node).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)

        # ================= 右侧：高级编辑面板 =================
        right_frame = ttk.Frame(self.paned_window, padding=10)
        self.paned_window.add(right_frame, weight=2)

        ttk.Label(right_frame, text="模块标识:").grid(row=0, column=0, sticky=tk.W, pady=5)
        self.lbl_module_id = ttk.Label(right_frame, font=("Helvetica", 10, "bold"), foreground="blue", text="-")
        self.lbl_module_id.grid(row=0, column=1, sticky=tk.W, pady=5)

        ttk.Label(right_frame, text="模块描述:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.entry_desc = ttk.Entry(right_frame, width=40)
        self.entry_desc.grid(row=1, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="源文件 (src_patterns):\n(支持通配符, 每行一条)").grid(row=2, column=0, sticky=tk.NW, pady=5)
        self.txt_src = tk.Text(right_frame, height=4, width=40)
        self.txt_src.grid(row=2, column=1, sticky=tk.EW, pady=5)

        ttk.Label(right_frame, text="头文件 (inc_patterns):\n(支持通配符, 每行一条)").grid(row=3, column=0, sticky=tk.NW, pady=5)
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
        self.btn_save = ttk.Button(right_frame, text="💾 保存并更新模块", command=self.save_current_module)
        self.btn_save.grid(row=6, column=1, sticky=tk.E, pady=10)

        right_frame.columnconfigure(1, weight=1)
        self.clear_right_panel() 

    # ================= 核心逻辑：数据加载与动态树生成 =================
    def load_data(self):
        if not self.json_path.exists():
            print(f"警告：未找到 {self.json_path}，初始化空配置。")
            self.registry = {"roots": ["core", "ctl", "cctl", "csp", "vctl"], "modules": {}}
            self.save_json()
        else:
            try:
                with open(self.json_path, 'r', encoding='utf-8') as f:
                    self.registry = json.load(f)
            except Exception as e:
                messagebox.showerror("读取错误", f"读取 JSON 失败: {e}")
                return

        self.refresh_tree()

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

        # 遍历所有的 Roots
        for root_name in self.registry.get("roots", []):
            root_id = f"ROOT|{root_name}"
            # 插入根节点 (如 ctl)
            self.tree.insert("", tk.END, iid=root_id, text=root_name, open=True)
            
            # 获取该 root 下所有的模块
            modules = self.registry.get("modules", {}).get(root_name, {})
            
            # 核心升级：动态解析 | 生成中间文件夹节点
            for mod_key, mod_data in modules.items():
                if mod_data.get("type") == "module":
                    # 将 "component|interface" 切割为 ["component", "interface"]
                    parts = mod_key.split('|')
                    
                    parent_id = root_id
                    
                    # 1. 生成中间的文件夹节点 (除最后一个元素外的所有部分)
                    for i in range(len(parts) - 1):
                        folder_name = parts[i]
                        # 生成文件夹的绝对路径ID，例如: "FOLDER|ctl|component"
                        current_folder_path = "|".join(parts[:i+1])
                        folder_id = f"FOLDER|{root_name}|{current_folder_path}"
                        
                        # 如果这个文件夹还没在树上创建过，就创建它
                        if not self.tree.exists(folder_id):
                            # 使用 📁 图标提示这是个分类目录
                            self.tree.insert(parent_id, tk.END, iid=folder_id, text=f"📁 {folder_name}", open=True)
                        
                        # 将当前文件夹设为下一级的父节点
                        parent_id = folder_id

                    # 2. 生成最终的模块叶子节点
                    leaf_name = parts[-1]
                    mod_id = f"MOD|{root_name}|{mod_key}"
                    # 使用 📄 图标提示这是具体模块
                    self.tree.insert(parent_id, tk.END, iid=mod_id, text=f"📄 {leaf_name}")

    def on_tree_select(self, event):
        selected_items = self.tree.selection()
        if not selected_items: return
        node_id = selected_items[0]
        
        # 如果选中的是根模块 (ROOT) 或者中间文件夹 (FOLDER)，清空右侧面板
        if node_id.startswith("ROOT|") or node_id.startswith("FOLDER|"):
            self.clear_right_panel()
            return
            
        # 如果选中的是实体模块 (MOD)，最多切割2次以提取完整的 mod_key
        # 例如 MOD|ctl|component|interface -> root_name: ctl, mod_key: component|interface
        _, root_name, mod_key = node_id.split('|', 2)
        
        self.current_selected_node = (root_name, mod_key)
        mod_data = self.registry["modules"][root_name][mod_key]

        self.set_right_panel_state(tk.NORMAL)
        # 顶部提示完整的 ID，方便用户确认
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
            # 防止循环依赖自己
            if available_mod == f"{root_name}|{mod_key}":
                continue
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

        desc = self.entry_desc.get().strip()
        src_patterns = [f.strip() for f in self.txt_src.get(1.0, tk.END).split('\n') if f.strip()]
        inc_patterns = [f.strip() for f in self.txt_inc.get(1.0, tk.END).split('\n') if f.strip()]
        
        selected_indices = self.listbox_deps.curselection()
        depends_on = [self.listbox_deps.get(i) for i in selected_indices]

        target_mod = self.registry["modules"][root_name][mod_key]
        target_mod["description"] = desc
        target_mod["src_patterns"] = src_patterns
        target_mod["inc_patterns"] = inc_patterns
        target_mod["depends_on"] = depends_on

        self.save_json()
        messagebox.showinfo("成功", f"模块 '{mod_key}' 配置已保存！")

    def add_module(self):
        selected = self.tree.selection()
        if not selected:
            messagebox.showwarning("提示", "请先在左侧选择一个根节点或目录！")
            return
            
        node_id = selected[0]
        default_prefix = ""
        
        # 智能预填充：根据你选中的节点，自动填好新建模块的前缀
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

        mod_key = simpledialog.askstring(
            "新增子模块", 
            f"在根模块 '{root_name}' 下新增:\n(多级目录请用 '|' 分隔)", 
            initialvalue=default_prefix
        )
        
        if mod_key and mod_key.strip():
            mod_key = mod_key.strip()
            
            if root_name not in self.registry["modules"]:
                self.registry["modules"][root_name] = {}
                
            if mod_key in self.registry["modules"][root_name]:
                messagebox.showwarning("提示", "该模块已存在！")
                return
            
            self.registry["modules"][root_name][mod_key] = {
                "type": "module",
                "description": "新模块",
                "src_patterns": ["src/*.c"],
                "inc_patterns": ["**/*.h"],
                "depends_on": []
            }
            self.save_json()
            self.refresh_tree()
            self.tree.selection_set(f"MOD|{root_name}|{mod_key}")

    def delete_node(self):
        selected = self.tree.selection()
        if not selected: return
        node_id = selected[0]
        
        if node_id.startswith("ROOT|"):
            messagebox.showwarning("拒绝", "系统级根模块 (Roots) 不允许在此直接删除。")
            return
        elif node_id.startswith("FOLDER|"):
            messagebox.showwarning("提示", "为了安全起见，暂不支持直接删除整个分类文件夹。\n请展开文件夹，逐个删除其内部的具体模块。")
            return
            
        if node_id.startswith("MOD|"):
            _, root_name, mod_key = node_id.split('|', 2)
            if messagebox.askyesno("确认", f"确定要彻底删除模块 '{mod_key}' 的配置吗？"):
                del self.registry["modules"][root_name][mod_key]
                self.save_json()
                self.refresh_tree()
                self.clear_right_panel()

    def save_json(self):
        with open(self.json_path, 'w', encoding='utf-8') as f:
            json.dump(self.registry, f, indent=4, ensure_ascii=False)

if __name__ == "__main__":
    root = tk.Tk()
    app = FrameworkDevGUI(root)
    root.mainloop()
